/* ============================================================================
 * run.c — 采集流程实现（自检 → 准备 → 清空+基线 → 排空循环 → 收尾）
 *
 * 目标：8 kHz 连续排空 ICM-42688-P FIFO，按 `数据格式规范.md` v3 落盘。
 *
 * 线程模型：主线程 = 采集（SCHED_FIFO + 绑核），只做 FIFO 排空 + 序列化 + 投递；
 * writer 线程 = 落盘 + 周期 fdatasync（见 writer.c）。主线程不写盘，避免 fdatasync
 * 造成毫秒级停顿切断 16 ms 的 FIFO 满窗。
 *
 * 启动顺序（关键，测试覆盖）：
 *   1) 自检（失败默认中止，不带病采集）
 *   2) writer 预分配 + 起线程
 *   3) 实时化
 *   4) 「已验证清空 + 丢包基线」——放在所有慢速准备之后，准备期溢出不会进入采集
 *   5) 排空循环
 * 时间戳口径：记录内 16-bit 是 raw ticks，µs = raw*32/30（Rev 1.6 §12.7）。
 * **时基口径**：summary 里的 odr/measured_odr_hz 是「芯片内部标称时基换算速率」
 * （内部时钟、未接 CLKIN、未做绝对时钟校准），不能读成独立绝对实测 ODR；独立参考
 * 量是 host_rate_hz（数据块 t_read 窗口）。两者不一致不换算系数、不放宽漂移门限。
 * 主循环是纯轮询（FIFO_COUNT + sleep），未接 GPIO 中断线，勿宣称 IRQ 驱动。
 * ==========================================================================*/
#define _GNU_SOURCE
#include "run.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include "util.h"
#include "vib_format.h"
#include "spi_dev.h"
#include "icm42688p.h"
#include "writer.h"
#include "tmst.h"

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

/* FIFO 满窗：2 KB / 16 B = 128 条 @8 kHz ≈ 16 ms。用作主机采样率的边界不确定度
 * 说明——块头的 t_read 是「批量读完成时刻」，不是采样时刻，块内样本可能早至
 * 满窗之前生成。 */
#define VIB_FIFO_FULL_WINDOW_US 16000u

typedef struct {
    uint32_t block_index;
    uint32_t host_dropped_blocks;
    uint32_t device_lost_pkt;
    uint64_t samples;             /* 采集到的记录数（collected，含 invalid） */
    uint64_t invalid_samples;     /* 其中被标记 VIB_RF_INVALID（哨兵 -32768）的记录数 */
    uint64_t first_batch_valid;   /* 首批（第一次排空）有效记录数，主机采样率分母要扣除 */
    uint64_t clip_samples;
    uint32_t bad_records;
    uint32_t spi_read_errors;     /* status/lost/count 读取失败次数（不得当作 0 健康） */
    uint32_t short_blocks;        /* records < 水位的块数：信息级（正常不足水位排空），
                                   * 不是丢包证据；真实丢失看 device_lost_pkt /
                                   * host_dropped_blocks / spi_read_errors / runtime_fault */
    int      runtime_fault;       /* 运行时 count/read_fifo/lost 失败：不能正常结束 */
    uint64_t first_read_ns;       /* 首次排空读完成时刻（0=未发生） */
    uint64_t last_read_ns;        /* 最近一次排空读完成时刻 */
    uint32_t drains;              /* 排空次数（测试钩子 max_drains 用） */
} run_stat_t;

static int drain_once(icm_t *icm, writer_t *w, run_stat_t *rs, tmst_accum_t *ts)
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
    if (rs->first_read_ns == 0)
        rs->first_read_ns = t_read;
    rs->last_read_ns = t_read;
    rs->drains++;

    /* status/lost 读取失败必须显式标记，绝不能默认 0=健康 */
    uint8_t  st = 0;
    uint32_t lost = 0;
    const int st_ok  = icm_read_status(icm, &st);
    const int lost_ok = icm_read_lost(icm, &lost);
    if (st_ok != 0 || lost_ok != 0)
        rs->spi_read_errors++;
    rs->device_lost_pkt = lost_ok == 0 ? lost : rs->device_lost_pkt;

    vib_record_t recs_out[VIB_SAMPLES_PER_BLOCK];
    uint32_t bad = 0;
    const size_t got = icm_parse_fifo(rawbuf, nbytes, recs_out, VIB_SAMPLES_PER_BLOCK, &bad);
    if (bad) {
        rs->bad_records += bad;
        icm->bad_header_records += bad;
    }

    uint32_t clip = 0;
    uint32_t invalid_here = 0;
    for (size_t i = 0; i < got; i++) {
        if (recs_out[i].rec_flags & VIB_RF_INVALID)
            invalid_here++;
        const int16_t v[6] = { recs_out[i].accel[0], recs_out[i].accel[1], recs_out[i].accel[2],
                               recs_out[i].gyro[0],  recs_out[i].gyro[1],  recs_out[i].gyro[2] };
        for (int a = 0; a < 6; a++) {
            if (v[a] >= VIB_CLIP_THRESHOLD || v[a] <= -VIB_CLIP_THRESHOLD) {
                clip++;
                break;
            }
        }
        tmst_accum_add(ts, recs_out[i].tmst_raw);
    }
    rs->clip_samples += clip;
    rs->invalid_samples += invalid_here;
    rs->samples += got;
    if (rs->drains == 1)                     /* 首批样本在 first_read_ns 之前生成，主机采样率要扣 */
        rs->first_batch_valid = got - invalid_here;

    vib_block_header_t bh;
    memset(&bh, 0, sizeof(bh));
    bh.block_type          = VIB_BLOCK_TYPE_DATA;
    bh.n_samples           = (uint16_t)got;
    bh.block_index         = rs->block_index++;
    bh.t_read_monotonic_ns = t_read;
    bh.lost_pkt_total      = lost_ok == 0 ? lost : 0;
    bh.clip_count          = clip;
    if (st_ok == 0 && (st & ICM_STATUS_FIFO_FULL))
        bh.flags |= VIB_BF_FIFO_FULL;
    if (st_ok != 0 || lost_ok != 0)
        bh.flags |= VIB_BF_SPI_ERR;        /* 本块 FIFO 健康未知 */
    if (recs < (int)(icm->cfg.wm_bytes / VIB_RECORD_SIZE)) {
        bh.flags |= VIB_BF_SHORT_READ;   /* 信息级：本批不足水位即排空（正常收尾/兜底排空
                                          * 都会命中），**不含**丢包信息——器件丢包看
                                          * FIFO_LOST_PKT，主机丢块看 block_index，传输
                                          * 失败走 VIB_BF_SPI_ERR / runtime_fault。 */
        rs->short_blocks++;
    }

    if (writer_submit(w, &bh, recs_out, got * VIB_RECORD_SIZE) != 0)
        rs->host_dropped_blocks++;        /* ⇒ 解析器会看到 block_index 缺号 */

    return (int)got;
}

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

int vib_run(const vib_run_args_t *a)
{
    g_stop = 0;   /* 每次运行重置（同进程多次调用时不清除上次信号） */
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* ---- SPI + IMU ---- */
    spi_dev_t local_spi;
    spi_dev_t *spi = a->spi;
    if (!spi) {
        spi = &local_spi;
        memset(spi, 0, sizeof(*spi));
        if (spi_open(spi, a->dev, a->spi_hz, a->spi_mode, 8) != 0)
            return 1;
    }

    static icm_t icm;
    memset(&icm, 0, sizeof(icm));
    icm.spi = spi;
    icm_default_cfg(&icm.cfg);
    icm.count_div   = 1;                 /* cfg.intf_config0=0x70 → COUNT_REC=1（按记录）；自检回读确认 */
    icm.count_endian = 1;                /* COUNT_ENDIAN=1（大端） */

    char selftest_report[2048];
    memset(selftest_report, 0, sizeof(selftest_report));
    int selftest_pass = 1;

    if (a->do_selftest) {
        if (icm_selftest(&icm, selftest_report, sizeof(selftest_report)) != 0) {
            selftest_pass = 0;
            fprintf(stderr, "%s", selftest_report);
            fprintf(stderr, "[run] 自检未通过——默认中止采集（不带病采集；无硬件同步/校验宣称）\n");
            if (!a->spi)
                spi_close(spi);
            return 1;
        }
        fprintf(stderr, "%s", selftest_report);
    } else {
        snprintf(selftest_report, sizeof(selftest_report), "[自检] 已跳过（--no-selftest）\n");
        if (icm_soft_reset(&icm) != 0 || icm_apply_config(&icm) != 0) {
            fprintf(stderr, "[run] 配置写入失败\n");
            if (!a->spi)
                spi_close(spi);
            return 1;
        }
    }

    if (a->selftest_only) {
        fprintf(stderr, "[run] --selftest-only，正常退出\n");
        if (!a->spi)
            spi_close(spi);
        return selftest_pass ? 0 : 1;
    }

    /* ---- 文件头（先建好，供 writer 预分配与落盘）---- */
    vib_file_header_t fh;
    memset(&fh, 0, sizeof(fh));
    fh.t_start_monotonic_ns  = mono_ns();
    fh.t_start_utc_ns        = utc_ns();
    fh.who_am_i              = icm.who_am_i;
    fh.frame                 = (uint8_t)(a->frame ? 1 : 0);
    fh.reg_config_hash       = icm.reg_hash;
    fh.odr_hz                = icm.cfg.odr_hz;
    fh.fs_accel_g            = icm.cfg.fs_accel_g;
    fh.fs_gyro_dps           = icm.cfg.fs_gyro_dps;
    fh.sample_count_expected = a->seconds > 0
        ? (uint64_t)(a->seconds * (double)icm.cfg.odr_hz) : 0;
    /* 时间戳换算元数据：raw ticks → µs = *32/30，内部时钟、RES=0 */
    fh.tmst_scale_num = VIB_TMST_SCALE_NUM;
    fh.tmst_scale_den = VIB_TMST_SCALE_DEN;
    fh.tmst_source    = VIB_TMST_SOURCE_INTERNAL;
    fh.tmst_res       = VIB_TMST_RES_1US;
    fh.count_unit     = (uint8_t)icm.count_div;
    fh.count_endian   = icm.count_endian;

    /* ---- writer（预分配 + 起线程）---- */
    char binpath[768];
    snprintf(binpath, sizeof(binpath), "%s/%s.bin", a->outdir, a->name);

    static writer_t wr;
    uint64_t prealloc = 0;
    if (a->seconds > 0) {
        const uint64_t samples = (uint64_t)(a->seconds * (double)icm.cfg.odr_hz);
        const uint64_t blocks  = samples / VIB_SAMPLES_PER_BLOCK + 2;
        prealloc = VIB_HEADER_SIZE + blocks * WRITER_SLOT_BYTES + VIB_BLOCK_HEADER_SIZE;
    }
    if (writer_start(&wr, binpath, &fh, prealloc, a->fsync_ms) != 0) {
        if (!a->spi)
            spi_close(spi);
        return 1;
    }

    /* ---- 实时化（放在 writer 起线程之后，避免把约束加到写盘线程）---- */
    if (rt_setup(a->cpu, a->prio, 1) != 0)
        fprintf(stderr, "[run] 实时化未完全成功——该批次有效性取决于"
                        "FIFO_LOST_PKT 是否保持基线不变\n");

    /* ---- 有效采集起点：所有慢速准备（预分配/线程/RT）都完成之后，
     * 才做「已验证清空 + 丢包基线」，随即立刻进入采集循环。
     * 这样准备阶段的任何耗时都不会把 FIFO 灌满（旧顺序会在准备期间溢出）。
     * 失败用 writer_abort 截断，不留「空但健康」的结束块。 ---- */
    if (icm_fifo_flush(&icm) != 0) {
        fprintf(stderr, "[run] FIFO 清空失败——中止（不能以未知健康状态开始采集）\n");
        writer_abort(&wr);
        if (!a->spi)
            spi_close(spi);
        return 1;
    }
    uint32_t lost0 = 0;
    if (icm_read_lost(&icm, &lost0) != 0) {
        fprintf(stderr, "[run] 读 FIFO_LOST_PKT 基线失败——中止\n");
        writer_abort(&wr);
        if (!a->spi)
            spi_close(spi);
        return 1;
    }
    fprintf(stderr, "[run] FIFO 已清空，丢包基线 = %u（采集起点）\n", lost0);

    /* ---- 主循环 ---- */
    run_stat_t  rs;
    tmst_accum_t ts;
    memset(&rs, 0, sizeof(rs));
    tmst_accum_init(&ts);

    const uint64_t wm_records = icm.cfg.wm_bytes / VIB_RECORD_SIZE;
    const uint64_t t0 = mono_ns();
    uint64_t last_drain = t0;
    uint64_t last_print = t0;

    fprintf(stderr, "[run] 开始采集：水位 %llu 条 / %u B，FIFO 满窗 16 ms，"
                    "drain 兜底周期 3 ms\n",
            (unsigned long long)wm_records, icm.cfg.wm_bytes);

    while (!g_stop) {
        int recs = 0;
        if (icm_fifo_count_records(&icm, &recs) != 0) {
            /* 运行时 FIFO_COUNT 读失败：FIFO 健康未知，不能静默重试后「正常结束」。
             * 记录并中止采集，最终以 writer_abort 截断（无结束块），解析器拒绝该文件。 */
            rs.spi_read_errors++;
            rs.runtime_fault = 1;
            fprintf(stderr, "[run] FIFO_COUNT 读取失败——中止采集（健康未知，不静默重试）\n");
            break;
        } else {
            const uint64_t now = mono_ns();
            const int need = (recs > 0) &&
                             ((uint64_t)recs >= wm_records || (now - last_drain) > 3000000ull);
            if (need) {
                if (drain_once(&icm, &wr, &rs, &ts) < 0) {
                    rs.spi_read_errors++;
                    rs.runtime_fault = 1;
                    fprintf(stderr, "[run] 排空（FIFO 读）失败——中止采集（健康未知）\n");
                    break;
                }
                last_drain = mono_ns();
            } else {
                sleep_us(250);
            }
        }

        if (a->seconds > 0 && (double)(mono_ns() - t0) / 1e9 >= a->seconds)
            break;
        if (a->max_drains > 0 && rs.drains >= a->max_drains)
            break;

        if (mono_ns() - last_print >= 5000000000ull) {
            const double odr = tmst_accum_rate_hz(&ts);
            fprintf(stderr,
                    "[run] %6.1f s | 样本 %llu | 块 %u | 丢块 %u | 器件丢包 %u | "
                    "削顶 %llu | 时基换算 %.1f Hz | 越窗 %llu | 重复 %llu | SPI读错 %u\n",
                    (double)(mono_ns() - t0) / 1e9,
                    (unsigned long long)rs.samples, rs.block_index,
                    rs.host_dropped_blocks, rs.device_lost_pkt,
                    (unsigned long long)rs.clip_samples, odr,
                    (unsigned long long)ts.out_of_window,
                    (unsigned long long)ts.dup, rs.spi_read_errors);
            last_print = mono_ns();
        }
    }

    /* ---- 收尾：把 FIFO 剩余数据排空（运行时已发生故障则不再排空） ---- */
    if (!rs.runtime_fault) {
        for (int i = 0; i < 64; i++) {
            int recs = 0;
            if (icm_fifo_count_records(&icm, &recs) != 0 || recs <= 0)
                break;
            if (drain_once(&icm, &wr, &rs, &ts) < 0) {
                rs.spi_read_errors++;
                rs.runtime_fault = 1;
                break;
            }
        }
    }

    /* 主机采样率：按「排空读时刻」定义的采集窗口，不含 writer 关闭耗时；
     * 分母用**有效记录**（collected − invalid），并扣除首批（首读前已生成）样本——
     * 首批样本在 first_read_ns 之前生成，不能算进窗口内。
     * 边界不确定度 = FIFO 满窗（读时刻非采样时刻，块内样本可早至 16 ms 前）。 */
    const uint64_t host_window_ns = (rs.last_read_ns > rs.first_read_ns)
        ? (rs.last_read_ns - rs.first_read_ns) : 0;
    const uint64_t valid_total = rs.samples > rs.invalid_samples
        ? (rs.samples - rs.invalid_samples) : 0;
    const uint64_t valid_in_window = valid_total > rs.first_batch_valid
        ? (valid_total - rs.first_batch_valid) : 0;
    const double host_rate_hz = host_window_ns
        ? (double)valid_in_window * 1e9 / (double)host_window_ns : 0.0;

    uint32_t lost_final = rs.device_lost_pkt;
    if (icm_read_lost(&icm, &lost_final) != 0) {
        /* 最后读 FIFO_LOST_PKT 失败：不能以旧丢包总数写正常结束块。 */
        rs.spi_read_errors++;
        rs.runtime_fault = 1;
        fprintf(stderr, "[run] 最后读 FIFO_LOST_PKT 失败——中止（不写正常结束块）\n");
    }
    /* Tail drops have no later block index to expose the loss. Omit END even
     * when earlier blocks were written correctly, so file-only checks fail. */
    if (rs.runtime_fault || rs.host_dropped_blocks > 0 || writer_error(&wr) != 0)
        writer_abort(&wr);          /* 无结束块：解析器据此拒绝「非正常结束」的文件 */
    else
        writer_finish(&wr, lost_final);

    /* 采集到的记录（collected，含 invalid）与已写入（written）区分：差值 ≈ 环满丢块 */
    const uint64_t collected = rs.samples;
    const uint64_t invalid   = rs.invalid_samples;
    const uint64_t written   = wr.samples_written;
    const uint64_t unwritten = (collected > written) ? (collected - written) : 0;

    /* ---- 统计输出 ---- */
    const int    median_raw  = tmst_accum_median_raw(&ts);
    const double odr         = tmst_accum_rate_hz(&ts);
    const uint64_t total_deltas = ts.n_deltas;
    const int    untrusted   = tmst_accum_untrusted(&ts);

    fprintf(stderr, "\n=== 采集结束 ===\n");
    fprintf(stderr, "文件        : %s\n", binpath);
    fprintf(stderr, "采集窗口(主机): %.6f s（排空读时刻区间，不含 writer 关闭）\n",
            (double)host_window_ns / 1e9);
    fprintf(stderr, "记录数      : %llu 采集 / %llu 有效 / %llu 无效（哨兵） / %llu 写入 / %llu 未写入（块 %u）\n",
            (unsigned long long)collected, (unsigned long long)(valid_total),
            (unsigned long long)invalid, (unsigned long long)written,
            (unsigned long long)unwritten, rs.block_index);
    fprintf(stderr, "时基换算速率: %.2f Hz（总 raw ticks/总时间；raw Δ 中位=%d，标称 %u Hz；"
                    "字段名沿用 odr/measured_odr_hz 以保持兼容）\n",
            odr, median_raw, icm.cfg.odr_hz);
    fprintf(stderr, "            ↑ 口径：芯片内部标称时基换算（raw*32/30，未接 CLKIN，"
                    "**未做绝对时钟校准**），不是独立绝对实测；独立参考见下行主机采样率\n");
    fprintf(stderr, "主机采样率  : %.2f Hz（采集窗口 %llu ns；独立于芯片时基；"
                    "边界不确定度 ±%u µs = FIFO 满窗）\n",
            host_rate_hz, (unsigned long long)host_window_ns, VIB_FIFO_FULL_WINDOW_US);
    fprintf(stderr, "raw Δ 范围  : [%d, %d]；越窗 %llu/%llu；重复 %llu；大间隔 %llu；缺口 %llu；断裂 %llu；%s\n",
            ts.n_deltas ? ts.dmin : 0, ts.n_deltas ? ts.dmax : 0,
            (unsigned long long)ts.out_of_window, (unsigned long long)total_deltas,
            (unsigned long long)ts.dup, (unsigned long long)ts.big,
            (unsigned long long)ts.gap, (unsigned long long)ts.breaks,
            untrusted ? "时间轴不可信" : "时间轴连续");
    fprintf(stderr, "器件丢包    : %u（基线 %u）\n", lost_final, lost0);
    fprintf(stderr, "主机丢块    : %u\n", rs.host_dropped_blocks);
    fprintf(stderr, "削顶样本    : %llu\n", (unsigned long long)rs.clip_samples);
    fprintf(stderr, "包头异常    : %u\n", rs.bad_records);
    fprintf(stderr, "SPI 读错    : %u（status/lost 读取失败，健康未知）\n", rs.spi_read_errors);
    fprintf(stderr, "短读块      : %u（records<水位的正常不足水位排空，信息级提示；"
                    "非数据丢失证据——器件丢包 %u、主机丢块 %u）\n",
            rs.short_blocks, rs.device_lost_pkt, rs.host_dropped_blocks);
    fprintf(stderr, "写盘        : %llu B / %llu 块 / fsync %llu 次 / errno %d\n",
            (unsigned long long)wr.bytes_written, (unsigned long long)wr.blocks_written,
            (unsigned long long)wr.fsync_count, writer_error(&wr));

    /* ---- 运行日志：与 .bin 同名的 .run.json，作为归档 MANIFEST 的证据 ---- */
    char runlog[768], esc[4096], iso[64];
    snprintf(runlog, sizeof(runlog), "%s/%s.run.json", a->outdir, a->name);
    iso8601_utc(fh.t_start_utc_ns, iso, sizeof(iso));
    json_escape(selftest_report, esc, sizeof(esc));

    FILE *f = fopen(runlog, "w");
    if (f) {
        fprintf(f,
            "{\n"
            "  \"bin\": \"%s\",\n"
            "  \"started_utc\": \"%s\",\n"
            "  \"capture_window_s\": %.6f,\n"
            "  \"host_rate_hz\": %.3f,\n"
            "  \"host_rate_boundary_uncertainty_us\": %u,\n"
            "  \"spidev\": \"%s\",\n"
            "  \"spi_hz\": %u,\n"
            "  \"spi_mode\": %u,\n"
            "  \"odr_hz_nominal\": %u,\n"
            "  \"odr_hz_measured\": %.3f,\n"
            "  \"timebase\": {\n"
            "    \"kind\": \"chip_nominal_scale\",\n"
            "    \"tmst_scale_num\": %u,\n"
            "    \"tmst_scale_den\": %u,\n"
            "    \"tmst_source\": %u,\n"
            "    \"tmst_res\": %u,\n"
            "    \"raw_delta_window_lo\": %u,\n"
            "    \"raw_delta_window_hi\": %u,\n"
            "    \"absolute_clock_calibrated\": false,\n"
            "    \"note\": \"odr_hz_measured 是芯片内部标称时基的换算速率(raw*32/30，未接 CLKIN)，未做绝对时钟校准，不得声明绝对物理 8000 Hz 已校准；独立参考量见 host_rate_hz\",\n"
            "    \"reference_independent_rate\": \"host_rate_hz\"\n"
            "  },\n"
            "  \"notices\": [\n"
            "    {\"kind\": \"short_read\", \"severity\": \"info\",\n"
            "     \"note\": \"records<水位的正常不足水位排空（如收尾批次），非数据丢失证据；器件丢包看 device_lost_pkt_final，主机丢块看 host_dropped_blocks，传输失败看 spi_read_errors/runtime_fault\",\n"
            "     \"blocks\": %u}\n"
            "  ],\n"
            "  \"watermark_bytes\": %u,\n"
            "  \"reg_config_hash\": %u,\n"
            "  \"who_am_i\": %u,\n"
            "  \"frame\": %u,\n"
            "  \"selftest_pass\": %s,\n"
            "  \"selftest_report\": \"%s\",\n"
            "  \"stats\": {\n"
            "    \"records_collected\": %llu,\n"
            "    \"records_valid\": %llu,\n"
            "    \"records_invalid\": %llu,\n"
            "    \"records_first_batch_valid\": %llu,\n"
            "    \"records_written\": %llu,\n"
            "    \"records_unwritten\": %llu,\n"
            "    \"blocks\": %u,\n"
            "    \"host_dropped_blocks\": %u,\n"
            "    \"device_lost_pkt_baseline\": %u,\n"
            "    \"device_lost_pkt_final\": %u,\n"
            "    \"clip_samples\": %llu,\n"
            "    \"bad_fifo_headers\": %u,\n"
            "    \"tmst_median_raw\": %d,\n"
            "    \"tmst_raw_min\": %d,\n"
            "    \"tmst_raw_max\": %d,\n"
            "    \"tmst_out_of_window\": %llu,\n"
            "    \"tmst_big_delta\": %llu,\n"
            "    \"tmst_gap\": %llu,\n"
            "    \"tmst_dup_timestamp\": %llu,\n"
            "    \"tmst_breaks\": %llu,\n"
            "    \"tmst_untrusted\": %d,\n"
            "    \"runtime_fault\": %d,\n"
            "    \"spi_read_errors\": %u,\n"
            "    \"short_read_blocks\": %u,\n"
            "    \"writer_errno\": %d,\n"
            "    \"fsync_count\": %llu\n"
            "  }\n"
            "}\n",
            binpath, iso, (double)host_window_ns / 1e9, host_rate_hz,
            VIB_FIFO_FULL_WINDOW_US,
            a->dev, a->spi_hz, a->spi_mode,
            icm.cfg.odr_hz, odr,
            VIB_TMST_SCALE_NUM, VIB_TMST_SCALE_DEN,
            (unsigned)fh.tmst_source, (unsigned)fh.tmst_res,
            VIB_TMST_RAW_LO, VIB_TMST_RAW_HI,
            rs.short_blocks,
            icm.cfg.wm_bytes, icm.reg_hash,
            icm.who_am_i, a->frame,
            selftest_pass ? "true" : "false", esc,
            (unsigned long long)collected, (unsigned long long)valid_total,
            (unsigned long long)invalid, (unsigned long long)rs.first_batch_valid,
            (unsigned long long)written, (unsigned long long)unwritten, rs.block_index,
            rs.host_dropped_blocks, lost0, lost_final,
            (unsigned long long)rs.clip_samples, rs.bad_records,
            median_raw, ts.n_deltas ? ts.dmin : 0, ts.n_deltas ? ts.dmax : 0,
            (unsigned long long)ts.out_of_window, (unsigned long long)ts.big,
            (unsigned long long)ts.gap, (unsigned long long)ts.dup,
            (unsigned long long)ts.breaks,
            untrusted ? 1 : 0,
            rs.runtime_fault ? 1 : 0,
            rs.spi_read_errors,
            rs.short_blocks,
            writer_error(&wr), (unsigned long long)wr.fsync_count);
        fclose(f);
        fprintf(stderr, "运行日志    : %s\n", runlog);
    } else {
        fprintf(stderr, "[run] 运行日志写入失败: %s\n", strerror(errno));
    }

    fprintf(stderr, "\n归档（见 ../../../_data/README.md §1/§3/§4）：\n"
                    "  sha256sum %s > %s.sha256\n"
                    "  并把上述统计与本 .run.json 抄进 <批次>/MANIFEST.json\n",
            binpath, binpath);

    if (!a->spi)
        spi_close(spi);

    /* 致命判据：真实错误/丢包/时间轴断裂才 exit 2。
     * 短读（rs.short_blocks）**故意不在**这里：生产 drain 读到 recs<水位是正常排空，
     * 不构成丢失证据；若把短读也算致命，收尾批与兜底排空会把健康采集误判为失败。
     * 真实 SPI 错误（spi_read_errors/runtime_fault）、FIFO 满/丢块（host_dropped_blocks）、
     * 器件丢包（device_lost_pkt）、坏包头（bad_records）、坏 CRC/未结束（由解析器负责）
     * 仍然一律致命。 */
    const int fatal = writer_error(&wr) != 0 || rs.host_dropped_blocks > 0 ||
                      rs.device_lost_pkt > lost0 || rs.bad_records > 0 ||
                      rs.spi_read_errors > 0 || rs.runtime_fault ||
                      untrusted || collected == 0;
    return fatal ? 2 : 0;
}
