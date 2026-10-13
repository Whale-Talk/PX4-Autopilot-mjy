/* ============================================================================
 * main.c — 原始震动采集程序（香橙派 5 Pro + ICM-42688-P）
 *
 * 目标：8 kHz 连续排空 ICM-42688-P 的 FIFO，按 `数据格式规范.md` v2 落盘。
 *
 * 线程模型：
 *   主线程      = 采集（SCHED_FIFO + 绑核）：只做 FIFO 排空 + 序列化 + 投递
 *   writer 线程 = 落盘 + 周期 fdatasync（见 writer.c）
 *
 * 为什么主线程不写盘：fdatasync 会造成毫秒级停顿，而 8 kHz 下 FIFO 仅 16 ms
 * 窗口，一次停顿就可能溢出。所有 I/O 交给 writer 线程，主线程只投递内存块。
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "util.h"
#include "vib_format.h"
#include "spi_dev.h"
#include "icm42688p.h"
#include "writer.h"

#define TMST_HIST_N 4096

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

typedef struct {
    const char *dev;
    uint32_t    spi_hz;
    uint8_t     spi_mode;
    const char *outdir;
    char        name[128];
    double      seconds;
    int         cpu;
    int         prio;
    uint32_t    fsync_ms;
    int         frame;
    int         do_selftest;
    int         selftest_only;
} run_cfg_t;

/* ---------------------------------------------------------------------------
 * 时间戳统计：Δtmst、实测 ODR、越窗比例（数据格式规范.md §4 要求解析器给出）
 * -------------------------------------------------------------------------*/
typedef struct {
    uint64_t n;                  /* 参与统计的记录数 */
    int64_t  prev;               /* 上一条原始 tmst */
    uint64_t sum;
    int      dmin, dmax;
    uint64_t hist[TMST_HIST_N];
    uint64_t big;                /* Δ ≥ TMST_HIST_N 的次数（字节序配错时会爆这里） */
    uint64_t out_of_window;      /* |Δ−125| > 1 µs 的次数 */
} tmst_stat_t;

static void tmst_stat_update(tmst_stat_t *ts, uint16_t tmst)
{
    if (ts->n == 0) {
        ts->prev = tmst;
        ts->n = 1;
        return;
    }
    int d = (int)tmst - (int)ts->prev;
    if (d < 0)
        d += (int)VIB_TMST_WRAP;          /* 回绕展开（格式规范 §4） */
    ts->prev = tmst;
    ts->n++;
    if (d < 0)
        return;
    ts->sum += (uint64_t)d;
    if (d < ts->dmin) ts->dmin = d;
    if (d > ts->dmax) ts->dmax = d;
    if (d < (int)TMST_HIST_N)
        ts->hist[d]++;
    else
        ts->big++;

    const int lo = (int)VIB_TMST_NOMINAL_US - (int)VIB_TMST_TOL_US;
    const int hi = (int)VIB_TMST_NOMINAL_US + (int)VIB_TMST_TOL_US;
    if (d < lo || d > hi)
        ts->out_of_window++;
}

static int tmst_stat_median(const tmst_stat_t *ts, int *median, double *measured_odr)
{
    uint64_t cnt = ts->n > 1 ? ts->n - 1 : 0;
    if (cnt == 0)
        return -1;
    uint64_t acc = 0;
    for (int i = 0; i < (int)TMST_HIST_N; i++) {
        acc += ts->hist[i];
        if (acc * 2 >= cnt) {
            *median = i;
            *measured_odr = (i > 0) ? 1e6 / (double)i : 0.0;
            return 0;
        }
    }
    *median = -1;                         /* 全部落在 big 桶 */
    *measured_odr = 0.0;
    return -1;
}

/* ---------------------------------------------------------------------------
 * 采集统计与单次排空
 * -------------------------------------------------------------------------*/
typedef struct {
    uint32_t block_index;
    uint32_t host_dropped_blocks;
    uint32_t device_lost_pkt;
    uint64_t samples;
    uint64_t clip_samples;
    uint32_t bad_records;
} run_stat_t;

static int drain_once(icm_t *icm, writer_t *w, run_stat_t *rs, tmst_stat_t *ts)
{
    int recs = 0;
    if (icm_fifo_count_records(icm, &recs) != 0)
        return -1;
    if (recs <= 0)
        return 0;

    const int n = recs > VIB_SAMPLES_PER_BLOCK ? VIB_SAMPLES_PER_BLOCK : recs;
    const size_t nbytes = (size_t)n * VIB_RECORD_SIZE;

    uint8_t rawbuf[VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE];
    if (icm_read_fifo(icm, rawbuf, nbytes) != 0)
        return -1;
    const uint64_t t_read = mono_ns();    /* 批量读完成时刻（仅作校验，非采样时刻） */

    uint8_t  st = 0;
    uint32_t lost = 0;
    (void)icm_read_status(icm, &st);
    (void)icm_read_lost(icm, &lost);
    rs->device_lost_pkt = lost;

    vib_record_t recs_out[VIB_SAMPLES_PER_BLOCK];
    uint32_t bad = 0;
    const size_t got = icm_parse_fifo(rawbuf, nbytes, recs_out, VIB_SAMPLES_PER_BLOCK, &bad);
    if (bad) {
        rs->bad_records += bad;
        icm->bad_header_records += bad;
    }

    uint32_t clip = 0;
    for (size_t i = 0; i < got; i++) {
        const int16_t v[6] = { recs_out[i].accel[0], recs_out[i].accel[1], recs_out[i].accel[2],
                               recs_out[i].gyro[0],  recs_out[i].gyro[1],  recs_out[i].gyro[2] };
        for (int a = 0; a < 6; a++) {
            if (v[a] >= VIB_CLIP_THRESHOLD || v[a] <= -VIB_CLIP_THRESHOLD) {
                clip++;
                break;
            }
        }
        tmst_stat_update(ts, recs_out[i].tmst_us);
    }
    rs->clip_samples += clip;
    rs->samples += got;

    vib_block_header_t bh;
    memset(&bh, 0, sizeof(bh));
    bh.block_type          = VIB_BLOCK_TYPE_DATA;
    bh.n_samples           = (uint16_t)got;
    bh.block_index         = rs->block_index++;
    bh.t_read_monotonic_ns = t_read;
    bh.lost_pkt_total      = lost;
    bh.clip_count          = clip;
    if (st & ICM_STATUS_FIFO_FULL)
        bh.flags |= VIB_BF_FIFO_FULL;
    if (recs < (int)(icm->cfg.wm_bytes / VIB_RECORD_SIZE))
        bh.flags |= VIB_BF_SHORT_READ;

    if (writer_submit(w, &bh, recs_out, got * VIB_RECORD_SIZE) != 0)
        rs->host_dropped_blocks++;        /* ⇒ 解析器会看到 block_index 缺号 */

    return (int)got;
}

/* ---------------------------------------------------------------------------
 * 小工具
 * -------------------------------------------------------------------------*/
static void iso8601_utc(uint64_t ns, char *out, size_t n)
{
    const time_t sec = (time_t)(ns / 1000000000ull);
    struct tm tmv;
    gmtime_r(&sec, &tmv);
    strftime(out, n, "%Y-%m-%dT%H:%M:%SZ", &tmv);
}

static void json_escape(const char *in, char *out, size_t n)
{
    size_t j = 0;
    for (size_t i = 0; in && in[i] && j + 2 < n; i++) {
        const char c = in[i];
        if (c == '"' || c == '\\') {
            out[j++] = '\\';
            out[j++] = c;
        } else if (c == '\n') {
            out[j++] = '\\';
            out[j++] = 'n';
        } else {
            out[j++] = c;
        }
    }
    out[j] = '\0';
}

static int ensure_dir(const char *d)
{
    struct stat sb;
    if (stat(d, &sb) == 0)
        return S_ISDIR(sb.st_mode) ? 0 : -1;
    if (mkdir(d, 0755) == 0)
        return 0;
    return (errno == EEXIST) ? 0 : -1;
}

static void usage(const char *p)
{
    fprintf(stderr,
        "用法: %s [选项]\n"
        "  -d, --dev PATH        spidev 节点（默认 /dev/spidev4.0）\n"
        "  -s, --spi-hz HZ       SPI 时钟（默认 24000000，IMU 上限，见手册 p.1）\n"
        "      --spi-mode 0|3    SPI 模式（默认 0；手册 p.52/p.63 只允许 0 或 3）\n"
        "  -o, --out DIR         输出目录（默认 .）\n"
        "  -n, --name NAME       输出文件名前缀（默认 vib_<UTC 时间戳>）\n"
        "  -t, --seconds S       采集时长秒；0=直到 Ctrl-C（默认 0）\n"
        "  -c, --cpu N           采集线程绑核（默认 4；-1 不绑）\n"
        "  -p, --prio N          SCHED_FIFO 优先级（默认 80；0 不用 RT）\n"
        "      --fsync-ms N      fdatasync 周期（默认 5000；0 不主动 fsync）\n"
        "      --frame 0|1       0=SENSOR, 1=FRD 写入文件头（默认 0）\n"
        "      --no-selftest     跳过自检（不建议；批次须标注未过 G1）\n"
        "      --selftest-only   只自检不采集\n"
        "  -h, --help\n", p);
}

static const struct option long_opts[] = {
    { "dev",           required_argument, NULL, 'd' },
    { "spi-hz",        required_argument, NULL, 's' },
    { "spi-mode",      required_argument, NULL, 1000 },
    { "out",           required_argument, NULL, 'o' },
    { "name",          required_argument, NULL, 'n' },
    { "seconds",       required_argument, NULL, 't' },
    { "cpu",           required_argument, NULL, 'c' },
    { "prio",          required_argument, NULL, 'p' },
    { "fsync-ms",      required_argument, NULL, 1001 },
    { "frame",         required_argument, NULL, 1002 },
    { "no-selftest",   no_argument,       NULL, 1003 },
    { "selftest-only", no_argument,       NULL, 1004 },
    { "help",          no_argument,       NULL, 'h' },
    { NULL, 0, NULL, 0 }
};

int main(int argc, char **argv)
{
    run_cfg_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.dev         = "/dev/spidev4.0";
    rc.spi_hz      = 24000000;
    rc.spi_mode    = 0;
    rc.outdir      = ".";
    rc.seconds     = 0.0;
    rc.cpu         = 4;
    rc.prio        = 80;
    rc.fsync_ms    = 5000;
    rc.frame       = 0;
    rc.do_selftest = 1;

    int c;
    while ((c = getopt_long(argc, argv, "d:s:o:n:t:c:p:h", long_opts, NULL)) != -1) {
        switch (c) {
        case 'd': rc.dev = optarg; break;
        case 's': rc.spi_hz = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 'o': rc.outdir = optarg; break;
        case 'n': snprintf(rc.name, sizeof(rc.name), "%s", optarg); break;
        case 't': rc.seconds = strtod(optarg, NULL); break;
        case 'c': rc.cpu = atoi(optarg); break;
        case 'p': rc.prio = atoi(optarg); break;
        case 1000: rc.spi_mode = (uint8_t)atoi(optarg); break;
        case 1001: rc.fsync_ms = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 1002: rc.frame = atoi(optarg); break;
        case 1003: rc.do_selftest = 0; break;
        case 1004: rc.selftest_only = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }
    if (rc.spi_mode != 0 && rc.spi_mode != 3) {
        fprintf(stderr, "SPI 模式只能是 0 或 3（手册 p.52/p.63）\n");
        return 1;
    }
    if (ensure_dir(rc.outdir) != 0) {
        fprintf(stderr, "输出目录不可用: %s\n", rc.outdir);
        return 1;
    }
    if (rc.name[0] == '\0') {
        const time_t now = time(NULL);
        struct tm tmv;
        gmtime_r(&now, &tmv);
        strftime(rc.name, sizeof(rc.name), "vib_%y%m%dT%H%M%SZ", &tmv);
    }

    fprintf(stderr, "=== 原始震动采集 v0.1.0（数据格式 v%d）===\n", VIB_FORMAT_VERSION);
    fprintf(stderr, "设备=%s  SPI=%u Hz  mode=%u  输出=%s/%s.bin\n",
            rc.dev, rc.spi_hz, rc.spi_mode, rc.outdir, rc.name);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* ---- SPI + IMU ---- */
    static spi_dev_t spi;
    if (spi_open(&spi, rc.dev, rc.spi_hz, rc.spi_mode, 8) != 0)
        return 1;

    static icm_t icm;
    memset(&icm, 0, sizeof(icm));
    icm.spi = &spi;
    icm_default_cfg(&icm.cfg);
    icm.count_div = 16;                 /* 自检会实测确认 */

    char selftest_report[2048];
    memset(selftest_report, 0, sizeof(selftest_report));
    int selftest_pass = 1;

    if (rc.do_selftest) {
        if (icm_selftest(&icm, selftest_report, sizeof(selftest_report)) != 0) {
            selftest_pass = 0;
            fprintf(stderr, "%s", selftest_report);
            fprintf(stderr, "[main] 自检未通过；该批次必须标注为「未通过 G1」\n");
            if (rc.selftest_only)
                return 1;
        } else {
            fprintf(stderr, "%s", selftest_report);
        }
    } else {
        snprintf(selftest_report, sizeof(selftest_report), "[自检] 已跳过（--no-selftest）\n");
        if (icm_soft_reset(&icm) != 0 || icm_apply_config(&icm) != 0) {
            fprintf(stderr, "[main] 配置写入失败\n");
            return 1;
        }
        (void)icm_fifo_flush(&icm);
    }

    if (rc.selftest_only) {
        fprintf(stderr, "[main] --selftest-only，正常退出\n");
        spi_close(&spi);
        return selftest_pass ? 0 : 1;
    }

    if (icm_fifo_flush(&icm) != 0)
        fprintf(stderr, "[main] 警告：FIFO 清空失败\n");
    uint32_t lost0 = 0;
    if (icm_read_lost(&icm, &lost0) == 0)
        fprintf(stderr, "[main] FIFO_LOST_PKT 基线 = %u\n", lost0);

    /* ---- 文件头 ---- */
    vib_file_header_t fh;
    memset(&fh, 0, sizeof(fh));
    fh.t_start_monotonic_ns  = mono_ns();
    fh.t_start_utc_ns        = utc_ns();
    fh.who_am_i              = icm.who_am_i;
    fh.frame                 = (uint8_t)(rc.frame ? 1 : 0);
    fh.reg_config_hash       = icm.reg_hash;
    fh.odr_hz                = icm.cfg.odr_hz;
    fh.fs_accel_g            = icm.cfg.fs_accel_g;
    fh.fs_gyro_dps           = icm.cfg.fs_gyro_dps;
    fh.sample_count_expected = rc.seconds > 0
        ? (uint64_t)(rc.seconds * (double)icm.cfg.odr_hz) : 0;

    /* ---- writer ---- */
    char binpath[768];
    snprintf(binpath, sizeof(binpath), "%s/%s.bin", rc.outdir, rc.name);

    static writer_t wr;
    uint64_t prealloc = 0;
    if (rc.seconds > 0) {
        const uint64_t samples = (uint64_t)(rc.seconds * (double)icm.cfg.odr_hz);
        const uint64_t blocks  = samples / VIB_SAMPLES_PER_BLOCK + 2;
        prealloc = VIB_HEADER_SIZE + blocks * WRITER_SLOT_BYTES + VIB_BLOCK_HEADER_SIZE;
    }
    if (writer_start(&wr, binpath, &fh, prealloc, rc.fsync_ms) != 0)
        return 1;

    /* ---- 实时化（放在 writer 起线程之后，避免把约束加到写盘线程）---- */
    if (rt_setup(rc.cpu, rc.prio, 1) != 0)
        fprintf(stderr, "[main] 实时化未完全成功——该批次有效性取决于"
                        "FIFO_LOST_PKT 是否保持 0（见 时间戳与丢包验证.md）\n");

    /* ---- 主循环 ---- */
    run_stat_t  rs;
    tmst_stat_t ts;
    memset(&rs, 0, sizeof(rs));
    memset(&ts, 0, sizeof(ts));
    ts.dmin = 1 << 20;
    ts.dmax = -(1 << 20);

    const uint64_t wm_records = icm.cfg.wm_bytes / VIB_RECORD_SIZE;
    const uint64_t t0 = mono_ns();
    uint64_t last_drain = t0;
    uint64_t last_print = t0;

    fprintf(stderr, "[main] 开始采集：水位 %llu 条 / %u B，FIFO 满窗 16 ms，"
                    "drain 兜底周期 3 ms\n",
            (unsigned long long)wm_records, icm.cfg.wm_bytes);

    while (!g_stop) {
        int recs = 0;
        if (icm_fifo_count_records(&icm, &recs) != 0) {
            fprintf(stderr, "[main] FIFO_COUNT 读取失败，重试\n");
            sleep_us(1000);
            continue;
        }

        const uint64_t now = mono_ns();
        const int need = (recs > 0) &&
                         ((uint64_t)recs >= wm_records || (now - last_drain) > 3000000ull);
        if (need) {
            if (drain_once(&icm, &wr, &rs, &ts) < 0)
                fprintf(stderr, "[main] 排空失败\n");
            last_drain = mono_ns();
        } else {
            sleep_us(250);
        }

        if (rc.seconds > 0 && (double)(mono_ns() - t0) / 1e9 >= rc.seconds)
            break;

        if (mono_ns() - last_print >= 5000000000ull) {
            int median = -1;
            double odr = 0.0;
            (void)tmst_stat_median(&ts, &median, &odr);
            fprintf(stderr,
                    "[main] %6.1f s | 样本 %llu | 块 %u | 丢块 %u | 器件丢包 %u | "
                    "削顶 %llu | 实测ODR %.1f Hz | 越窗 %llu\n",
                    (double)(mono_ns() - t0) / 1e9,
                    (unsigned long long)rs.samples, rs.block_index,
                    rs.host_dropped_blocks, rs.device_lost_pkt,
                    (unsigned long long)rs.clip_samples, odr,
                    (unsigned long long)ts.out_of_window);
            last_print = mono_ns();
        }
    }

    /* ---- 收尾：把 FIFO 剩余数据排空 ---- */
    for (int i = 0; i < 64; i++) {
        int recs = 0;
        if (icm_fifo_count_records(&icm, &recs) != 0 || recs <= 0)
            break;
        if (drain_once(&icm, &wr, &rs, &ts) < 0)
            break;
    }

    uint32_t lost_final = rs.device_lost_pkt;
    (void)icm_read_lost(&icm, &lost_final);
    writer_finish(&wr, lost_final);

    /* ---- 统计输出 ---- */
    int    median = -1;
    double odr = 0.0;
    (void)tmst_stat_median(&ts, &median, &odr);
    const uint64_t total_records = ts.n > 0 ? ts.n - 1 : 0;
    const double duration = (double)(mono_ns() - t0) / 1e9;

    fprintf(stderr, "\n=== 采集结束 ===\n");
    fprintf(stderr, "文件        : %s\n", binpath);
    fprintf(stderr, "时长        : %.2f s\n", duration);
    fprintf(stderr, "记录数      : %llu（块 %u）\n",
            (unsigned long long)rs.samples, rs.block_index);
    fprintf(stderr, "实测 ODR    : %.2f Hz（中位 Δtmst = %d µs，标称 %u Hz）\n",
            odr, median, icm.cfg.odr_hz);
    fprintf(stderr, "Δtmst 范围  : [%d, %d] µs；越窗 %llu / %llu\n",
            ts.n > 1 ? ts.dmin : 0, ts.n > 1 ? ts.dmax : 0,
            (unsigned long long)ts.out_of_window, (unsigned long long)total_records);
    fprintf(stderr, "器件丢包    : %u（基线 %u）\n", lost_final, lost0);
    fprintf(stderr, "主机丢块    : %u\n", rs.host_dropped_blocks);
    fprintf(stderr, "削顶样本    : %llu\n", (unsigned long long)rs.clip_samples);
    fprintf(stderr, "包头异常    : %u\n", rs.bad_records);
    fprintf(stderr, "写盘        : %llu B / %llu 块 / fsync %llu 次 / errno %d\n",
            (unsigned long long)wr.bytes_written, (unsigned long long)wr.blocks_written,
            (unsigned long long)wr.fsync_count, writer_error(&wr));

    /* ---- 运行日志：与 .bin 同名的 .run.json，作为归档 MANIFEST 的证据 ---- */
    char runlog[768], esc[4096], iso[64];
    snprintf(runlog, sizeof(runlog), "%s/%s.run.json", rc.outdir, rc.name);
    iso8601_utc(fh.t_start_utc_ns, iso, sizeof(iso));
    json_escape(selftest_report, esc, sizeof(esc));

    FILE *f = fopen(runlog, "w");
    if (f) {
        fprintf(f,
            "{\n"
            "  \"bin\": \"%s\",\n"
            "  \"started_utc\": \"%s\",\n"
            "  \"duration_s\": %.3f,\n"
            "  \"spidev\": \"%s\",\n"
            "  \"spi_hz\": %u,\n"
            "  \"spi_mode\": %u,\n"
            "  \"odr_hz_nominal\": %u,\n"
            "  \"odr_hz_measured\": %.3f,\n"
            "  \"watermark_bytes\": %u,\n"
            "  \"reg_config_hash\": %u,\n"
            "  \"who_am_i\": %u,\n"
            "  \"frame\": %u,\n"
            "  \"selftest_pass\": %s,\n"
            "  \"selftest_report\": \"%s\",\n"
            "  \"stats\": {\n"
            "    \"records\": %llu,\n"
            "    \"blocks\": %u,\n"
            "    \"host_dropped_blocks\": %u,\n"
            "    \"device_lost_pkt_baseline\": %u,\n"
            "    \"device_lost_pkt_final\": %u,\n"
            "    \"clip_samples\": %llu,\n"
            "    \"bad_fifo_headers\": %u,\n"
            "    \"tmst_median_us\": %d,\n"
            "    \"tmst_min_us\": %d,\n"
            "    \"tmst_max_us\": %d,\n"
            "    \"tmst_out_of_window\": %llu,\n"
            "    \"tmst_big_delta\": %llu,\n"
            "    \"writer_errno\": %d,\n"
            "    \"fsync_count\": %llu\n"
            "  }\n"
            "}\n",
            binpath, iso, duration, rc.dev, rc.spi_hz, rc.spi_mode,
            icm.cfg.odr_hz, odr, icm.cfg.wm_bytes, icm.reg_hash,
            icm.who_am_i, rc.frame,
            selftest_pass ? "true" : "false", esc,
            (unsigned long long)rs.samples, rs.block_index,
            rs.host_dropped_blocks, lost0, lost_final,
            (unsigned long long)rs.clip_samples, rs.bad_records,
            median, ts.n > 1 ? ts.dmin : 0, ts.n > 1 ? ts.dmax : 0,
            (unsigned long long)ts.out_of_window, (unsigned long long)ts.big,
            writer_error(&wr), (unsigned long long)wr.fsync_count);
        fclose(f);
        fprintf(stderr, "运行日志    : %s\n", runlog);
    } else {
        fprintf(stderr, "[main] 运行日志写入失败: %s\n", strerror(errno));
    }

    fprintf(stderr, "\n归档（见 ../../../_data/README.md §1/§3/§4）：\n"
                    "  sha256sum %s > %s.sha256\n"
                    "  并把上述统计与本 .run.json 抄进 <批次>/MANIFEST.json\n",
            binpath, binpath);

    spi_close(&spi);

    const int fatal = writer_error(&wr) != 0 || rs.host_dropped_blocks > 0 ||
                      rs.device_lost_pkt > lost0 || rs.bad_records > 0;
    return fatal ? 2 : 0;
}
