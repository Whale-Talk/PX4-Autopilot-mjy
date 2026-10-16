/* ============================================================================
 * tests/startup_test.c — 生产启动顺序的确定性 mock 回归（fix 3/5/6）
 *
 * 通过 spi_dev_t.xfer_fn 注入一个**确定性虚拟时钟**的 ICM-42688-P 模型，
 * 直接调用 run.c 的 vib_run()（与 main() 完全同一生产代码路径），验证：
 *
 *   1) 自检 sub-full 窗口不引发 FIFO 溢出；自检失败默认中止（返回 1，不留文件）
 *   2) 准备阶段（apply_config + verify_config）的 FIFO 溢出（>16 ms 满窗）被
 *      启动时的「已验证清空 + 丢包基线」吸收，采集期丢包增量 == 0（返回 0）
 *   3) 启动 flush 失败 / 丢包基线读取失败 → 返回 1，且 writer_abort 截断文件
 *      （无结束块，只有 256 B 头），解析器不会把空/中止采集当健康
 *
 * 虚拟时钟：每次 SPI 事务推进 g.advance_us（µs），按 8 kHz 换算成 FIFO 样本。
 * 不用 mono_ns，完全确定、可复现。
 *
 * 用法: startup_test   （退出码 0=通过，1=失败）
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include "run.h"
#include "spi_dev.h"
#include "icm42688p.h"
#include "vib_format.h"
#include "util.h"
#include "writer.h"

#define MOCK_FIFO_CAP 128
#define MOCK_RECORD   16

static struct {
    uint8_t  reg[3][256];
    uint8_t  cur_bank;
    uint8_t  enabled;
    uint8_t  fifo[MOCK_FIFO_CAP * MOCK_RECORD];
    uint32_t fifo_count;
    uint32_t lost;
    uint64_t raw_cum;
    uint64_t last_ns;           /* 上次 SPI 事务时刻（mono_ns） */
    uint64_t advance_thresh_ns; /* 低于此时长不填 FIFO（默认 100 µs） */

    /* 故障注入 */
    int fail_whoami;
    int flush_count, fail_flush_at;
    int lost_count,   fail_lost_at;
    int count_reads,  fail_count_after;   /* 第 >fail_count_after 次 FIFO_COUNT 读失败 */
    int fifo_read_count, fail_fifo_read_at; /* 第 fail_fifo_read_at 次 FIFO_DATA 读失败（先采样本再失败） */
} g;

/* Simulate ring exhaustion after one successfully queued data block. */
static int inject_host_tail_drop;
static unsigned int host_tail_drops;
int __real_writer_submit(writer_t *w, const vib_block_header_t *bh,
                         const void *payload, size_t len);
int __wrap_writer_submit(writer_t *w, const vib_block_header_t *bh,
                         const void *payload, size_t len)
{
    if (inject_host_tail_drop && bh->block_index >= 1) {
        host_tail_drops++;
        return -1;
    }
    return __real_writer_submit(w, bh, payload, len);
}

static void g_device_reset(void);

static void g_reset(void)
{
    memset(&g, 0, sizeof(g));
    g.advance_thresh_ns = 100000;   /* 100 µs：低于此不填 FIFO，避免背靠背事务噪声 */
    g_device_reset();
}

/* 器件软复位：只清器件状态与计数，**保留**测试注入（thresh / fail_*）。 */
static void g_device_reset(void)
{
    memset(g.reg, 0, sizeof(g.reg));
    g.reg[0][ICM_REG_INTF_CONFIG0] = 0x30;
    g.reg[0][ICM_REG_INT_CONFIG1]  = 0x10;
    g.reg[0][ICM_REG_INT_SOURCE0]  = 0x10;
    g.reg[0][ICM_REG_GYRO_CONFIG1] = 0x16;
    g.reg[0][ICM_REG_ACCEL_CONFIG1] = 0x0D;
    g.reg[0][ICM_REG_GYRO_ACCEL_CONFIG0] = 0x11;
    g.cur_bank = 0;
    g.enabled = 0;
    g.fifo_count = 0;
    g.lost = 0;
    g.raw_cum = 0;
    g.flush_count = 0;
    g.lost_count = 0;
    g.count_reads = 0;
    g.fifo_read_count = 0;
    g.last_ns = mono_ns();
}

static void g_advance(void)
{
    const uint64_t now = mono_ns();
    const uint64_t el_ns = now - g.last_ns;
    g.last_ns = now;
    if (!g.enabled || el_ns < g.advance_thresh_ns)
        return;
    if (((g.reg[0][ICM_REG_FIFO_CONFIG] >> 6) & 0x03u) != 1u)   /* Bypass 不填 FIFO */
        return;
    const uint64_t samples = el_ns / 125000ull;   /* 8 kHz：125 µs/样本 */
    for (uint64_t i = 0; i < samples; i++) {
        g.raw_cum += 125u * 30u;
        const uint16_t tmst = (uint16_t)((g.raw_cum / 32u) & 0xFFFFu);
        if (g.fifo_count >= MOCK_FIFO_CAP) {
            g.lost++;
            continue;
        }
        uint8_t *r = g.fifo + g.fifo_count * MOCK_RECORD;
        r[0] = ICM_FIFO_HDR_ACCEL | ICM_FIFO_HDR_GYRO | ICM_FIFO_HDR_TS_ODR;
        /* 传感器数据按 DATA_ENDIAN=1（**大端**）出线缆。 */
        const int16_t ax = 1, ay = 2, az = 3, gx = 4, gy = 5, gz = 6;
        r[1] = (uint8_t)((uint16_t)ax >> 8); r[2]  = (uint8_t)ax;
        r[3] = (uint8_t)((uint16_t)ay >> 8); r[4]  = (uint8_t)ay;
        r[5] = (uint8_t)((uint16_t)az >> 8); r[6]  = (uint8_t)az;
        r[7] = (uint8_t)((uint16_t)gx >> 8); r[8]  = (uint8_t)gx;
        r[9] = (uint8_t)((uint16_t)gy >> 8); r[10] = (uint8_t)gy;
        r[11] = (uint8_t)((uint16_t)gz >> 8); r[12] = (uint8_t)gz;
        r[13] = 21;
        r[14] = (uint8_t)(tmst >> 8);
        r[15] = (uint8_t)(tmst & 0xFF);
        g.fifo_count++;
    }
}

static uint16_t g_count_value(void)
{
    if (g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_REC)
        return (uint16_t)g.fifo_count;
    return (uint16_t)(g.fifo_count * 16u);
}

static uint8_t g_read_byte(uint8_t addr)
{
    switch (addr) {
    case ICM_REG_WHO_AM_I:    return g.fail_whoami ? 0x00 : ICM_WHO_AM_I_VALUE;
    case ICM_REG_REG_BANK_SEL: return g.cur_bank;
    case ICM_REG_FIFO_COUNTH: {
        const uint16_t v = g_count_value();
        return (g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_ENDIAN)
               ? (uint8_t)(v >> 8) : (uint8_t)(v & 0xFF);
    }
    case ICM_REG_FIFO_COUNTL: {
        const uint16_t v = g_count_value();
        return (g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_ENDIAN)
               ? (uint8_t)(v & 0xFF) : (uint8_t)(v >> 8);
    }
    case ICM_REG_FIFO_LOST_PKT0: {
        g.lost_count++;
        return (uint8_t)(g.lost & 0xFF);
    }
    case ICM_REG_FIFO_LOST_PKT1:  return (uint8_t)(g.lost >> 8);
    case ICM_REG_INT_STATUS:      return 0x00;
    default:
        return g.reg[g.cur_bank][addr];
    }
}

static void g_write_byte(uint8_t addr, uint8_t val)
{
    if (addr == ICM_REG_REG_BANK_SEL) { g.cur_bank = (uint8_t)(val & 0x07); return; }
    if (addr == ICM_REG_DEVICE_CONFIG && (val & 0x01)) { g_device_reset(); return; }
    if (addr == ICM_REG_SIGNAL_PATH_RESET) {
        /* bit1=0x02 是 FIFO_FLUSH；0x04 是 TMST_STROBE，不清 FIFO。写 1 是自清 strobe。 */
        if (val & ICM_SPR_FIFO_FLUSH) {
            g.flush_count++;
            g.fifo_count = 0;
        }
        return;
    }
    if (addr == ICM_REG_PWR_MGMT0 && (val & 0x0F) != 0)
        g.enabled = 1;
    g.reg[g.cur_bank][addr] = val;
}

static int mock_xfer(spi_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)d;
    g_advance();
    if (len == 0)
        return 0;
    const uint8_t first = tx[0];
    const uint8_t addr = (uint8_t)(first & 0x7Fu);

    if (first & 0x80u) {                       /* 读 */
        if (addr == ICM_REG_SIGNAL_PATH_RESET) {
            /* flush 由写触发；读不会发生 */
        }
        if (addr == ICM_REG_FIFO_LOST_PKT0 && g.fail_lost_at &&
            g.lost_count + 1 == g.fail_lost_at)
            return -1;                          /* 第 fail_lost_at 次 lost 读失败 */
        if (addr == ICM_REG_FIFO_COUNTH) {
            g.count_reads++;
            if (g.fail_count_after && g.count_reads > g.fail_count_after)
                return -1;                      /* 运行时 FIFO_COUNT 读失败 */
        }
        if (addr == ICM_REG_FIFO_DATA) {
            g.fifo_read_count++;
            if (g.fail_fifo_read_at && g.fifo_read_count == g.fail_fifo_read_at)
                return -1;                      /* 采过样本后 read_fifo 失败 */
            const size_t nbytes = len - 1;
            const size_t nrec = nbytes / MOCK_RECORD;
            size_t take = nrec < g.fifo_count ? nrec : g.fifo_count;
            if (take && rx)
                memcpy(rx + 1, g.fifo, take * MOCK_RECORD);
            if (take < g.fifo_count)
                memmove(g.fifo, g.fifo + take * MOCK_RECORD,
                        (g.fifo_count - take) * MOCK_RECORD);
            g.fifo_count -= (uint32_t)take;
            return 0;
        }
        for (size_t i = 0; i + 1 < len && rx; i++)
            rx[i + 1] = g_read_byte((uint8_t)(addr + i));
    } else {                                   /* 写 */
        if (addr == ICM_REG_SIGNAL_PATH_RESET && (tx[1] & ICM_SPR_FIFO_FLUSH) &&
            g.fail_flush_at && g.flush_count + 1 == g.fail_flush_at)
            return -1;                          /* 第 fail_flush_at 次 flush 写失败 */
        if (len >= 2)
            g_write_byte(addr, tx[1]);
    }
    return 0;
}

static int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; fprintf(stderr, "  ✗ " __VA_ARGS__); fprintf(stderr, "\n"); } \
    else { printf("  ✓ " __VA_ARGS__); printf("\n"); } \
} while (0)

static long file_size(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0)
        return -1;
    return (long)st.st_size;
}

/* 扫描块头看是否存在结束块（0xFFFF）。用于断言中止采集不写干净结束块。 */
static int has_end_block(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return 0;
    if (fseek(f, VIB_HEADER_SIZE, SEEK_SET) != 0) {
        fclose(f);
        return 0;
    }
    int found = 0;
    for (;;) {
        uint16_t btype = 0, ns = 0;
        if (fread(&btype, 2, 1, f) != 1)
            break;
        if (fread(&ns, 2, 1, f) != 1)
            break;
        if (btype == VIB_BLOCK_TYPE_END) {
            found = 1;
            break;
        }
        /* 跳过块头剩余 28 B + payload（ns*16 B） */
        if (fseek(f, (long)(28 + (size_t)ns * VIB_RECORD_SIZE), SEEK_CUR) != 0)
            break;
    }
    fclose(f);
    return found;
}

static void mk_args(vib_run_args_t *a, spi_dev_t *spi, const char *outdir)
{
    memset(a, 0, sizeof(*a));
    a->dev = "/dev/null";
    a->spi_hz = 24000000;
    a->spi_mode = 0;
    a->outdir = outdir;
    snprintf(a->name, sizeof(a->name), "startup_test");
    a->seconds = 0.0;
    a->cpu = -1;                 /* 不绑核（测试环境） */
    a->prio = 0;                 /* 不 RT（避免需要权限） */
    a->fsync_ms = 0;
    a->frame = 0;
    a->do_selftest = 1;
    a->selftest_only = 0;
    a->spi = spi;
}

static spi_dev_t g_spi;

/* 运行日志（.run.json）可用性：所有 "{filename}.run.json" 中
 *   - braces 配平、无 NUL 以外的非法字节
 *   - 必须带机器可读时基元数据：kind=chip_nominal_scale、absolute_clock_calibrated=false
 *     （不得宣称绝对时钟已校准）、tmst_scale 32/30、raw 窗口 [115,120]
 *   - 必须带 notices（短读=信息级，不是异常）
 * 只做结构/契约检查，不引入 JSON 依赖。 */
static int runjson_selfcheck(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return -1;
    char buf[16384];
    const size_t rd = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[rd] = '\0';

    int depth = 0, neg = 0;
    for (size_t i = 0; i < rd; i++) {
        if (buf[i] == '{') depth++;
        else if (buf[i] == '}') { depth--; if (depth < 0) return -2; }
    }
    if (depth != 0)
        return -2;
    (void)neg;

    static const char *need[] = {
        "\"timebase\"", "\"kind\": \"chip_nominal_scale\"",
        "\"absolute_clock_calibrated\": false",
        "\"tmst_scale_num\": 32", "\"tmst_scale_den\": 30",
        "\"raw_delta_window_lo\": 115", "\"raw_delta_window_hi\": 120",
        "\"notices\"", "\"short_read\"", "\"short_read_blocks\"",
    };
    for (size_t i = 0; i < sizeof(need) / sizeof(need[0]); i++) {
        if (!strstr(buf, need[i]))
            return -3;
    }
    return 0;
}

static int run_healthy(char *out, uint32_t max_drains, uint32_t *lost_out)
{
    g_reset();
    memset(&g_spi, 0, sizeof(g_spi));
    g_spi.xfer_fn = mock_xfer;
    vib_run_args_t a;
    mk_args(&a, &g_spi, out);
    a.max_drains = max_drains;
    int rc = vib_run(&a);
    if (lost_out)
        *lost_out = g.lost;
    return rc;
}

int main(void)
{
    char out[] = "/tmp/startup_test_XXXXXX";
    char *dir = mkdtemp(out);
    if (!dir) { fprintf(stderr, "mkdtemp 失败\n"); return 1; }

    printf("== 1) 健康采集：准备期溢出被清空，采集期丢包增量 == 0 ==\n");
    {
        uint32_t lost_after = 0;
        const int rc = run_healthy(dir, /*max_drains=*/3, &lost_after);
        CHECK(rc == 0, "vib_run 返回 0（采集健康：无器件丢包增量/无丢块/时间轴连续）");
        CHECK(lost_after > 0, "准备阶段确实发生 FIFO 溢出（lost=%u > 0）", lost_after);
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_test.bin", dir);
        CHECK(file_size(p) > (long)(VIB_HEADER_SIZE + VIB_BLOCK_HEADER_SIZE),
              "健康采集产生含数据块的文件（%ld B）", file_size(p));
        snprintf(p, sizeof(p), "%s/startup_test.run.json", dir);
        const int jrc = runjson_selfcheck(p);
        CHECK(jrc == 0,
              "健康采集的 .run.json 结构/时基契约通过（braces 配平 + timebase 未校准声明 + 32/30 + 窗口[115,120] + notices，rc=%d）",
              jrc);
    }

    printf("== 2) 自检失败默认中止（不留文件）==\n");
    {
        g_reset();
        g.fail_whoami = 1;
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        a.name[0] = '\0';
        snprintf(a.name, sizeof(a.name), "startup_selftest_fail");
        const int rc = vib_run(&a);
        CHECK(rc == 1, "自检失败返回 1");
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_selftest_fail.bin", dir);
        CHECK(file_size(p) < 0, "自检失败不创建数据文件");
    }

    printf("== 3) 启动 flush 失败 → 中止 + writer_abort 截断（无结束块）==\n");
    {
        g_reset();
        g.fail_flush_at = 2;       /* 第 2 次 flush（启动 flush；第 1 次在自检里） */
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        a.name[0] = '\0';
        snprintf(a.name, sizeof(a.name), "startup_flush_fail");
        const int rc = vib_run(&a);
        CHECK(rc == 1, "flush 失败返回 1");
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_flush_fail.bin", dir);
        CHECK(file_size(p) == (long)VIB_HEADER_SIZE,
              "flush 失败后文件只有 256 B 头（截断，无结束块，实际 %ld B）", file_size(p));
    }

    printf("== 4) 丢包基线读取失败 → 中止 + writer_abort 截断 ==\n");
    {
        g_reset();
        g.fail_lost_at = 3;        /* 第 3 次 lost 读（自检读 2 次，启动基线是第 3 次） */
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        a.name[0] = '\0';
        snprintf(a.name, sizeof(a.name), "startup_baseline_fail");
        const int rc = vib_run(&a);
        CHECK(rc == 1, "丢包基线读取失败返回 1");
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_baseline_fail.bin", dir);
        CHECK(file_size(p) == (long)VIB_HEADER_SIZE,
              "基线失败后文件只有 256 B 头（截断，无结束块，实际 %ld B）", file_size(p));
    }

    printf("== 5) 运行时 FIFO_COUNT 读失败 → 中止（不当 0 健康，不留结束块）==\n");
    {
        g_reset();
        /* count 读序：自检 flush 验证 1 + 自检窗口 1 + 启动 flush 验证 1 = 3 次，
         * 第 4 次（主循环第一次）即失败 → 中止。 */
        g.fail_count_after = 3;
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        a.name[0] = '\0';
        snprintf(a.name, sizeof(a.name), "startup_count_fail");
        a.seconds = 0.03;
        const int rc = vib_run(&a);
        CHECK(rc == 2, "运行时 count 失败 → 返回 2（健康未知，实际 %d）", rc);
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_count_fail.bin", dir);
        CHECK(!has_end_block(p), "count 失败后文件无结束块（writer_abort 截断）");
    }

    printf("== 6) 采过样本后 read_fifo 失败 → 中止（不写干净结束块）==\n");
    {
        g_reset();
        /* FIFO_DATA 读序：自检 1 + 第一次排空 1 = 2 次成功，第 3 次（第二次排空）失败。 */
        g.fail_fifo_read_at = 3;
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        a.name[0] = '\0';
        snprintf(a.name, sizeof(a.name), "startup_readfifo_fail");
        a.seconds = 0.20;          /* 足够第一次排空成功（采集到样本）后再失败 */
        const int rc = vib_run(&a);
        CHECK(rc == 2, "采过样本后 read_fifo 失败 → 返回 2（实际 %d）", rc);
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_readfifo_fail.bin", dir);
        CHECK(file_size(p) > (long)(VIB_HEADER_SIZE + VIB_BLOCK_HEADER_SIZE),
              "已采到部分数据（文件 > 头+块头，实际 %ld B）", file_size(p));
        CHECK(!has_end_block(p), "read_fifo 失败后文件无结束块（不写干净结束块）");
    }

    printf("== 7) 尾部队列丢块不能留下健康结束块 ==\n");
    {
        g_reset();
        memset(&g_spi, 0, sizeof(g_spi));
        g_spi.xfer_fn = mock_xfer;
        vib_run_args_t a;
        mk_args(&a, &g_spi, dir);
        snprintf(a.name, sizeof(a.name), "startup_host_tail_drop");
        a.seconds = 0.10;
        host_tail_drops = 0;
        inject_host_tail_drop = 1;
        const int rc = vib_run(&a);
        inject_host_tail_drop = 0;
        CHECK(rc == 2 && host_tail_drops > 0, "确实注入了尾部丢块，返回 2");
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_host_tail_drop.bin", dir);
        CHECK(file_size(p) > (long)(VIB_HEADER_SIZE + VIB_BLOCK_HEADER_SIZE),
              "丢块前有真实写入的数据");
        CHECK(!has_end_block(p), "尾部丢块后不写健康结束块");
        unlink(p);
        snprintf(p, sizeof(p), "%s/startup_host_tail_drop.run.json", dir);
        unlink(p);
    }

    /* 清理测试产物 */
    {
        char p[1024];
        snprintf(p, sizeof(p), "%s/startup_test.bin", dir);       unlink(p);
        snprintf(p, sizeof(p), "%s/startup_test.run.json", dir);  unlink(p);
        snprintf(p, sizeof(p), "%s/startup_selftest_fail.bin", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_flush_fail.bin", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_baseline_fail.bin", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_count_fail.bin", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_count_fail.run.json", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_readfifo_fail.bin", dir); unlink(p);
        snprintf(p, sizeof(p), "%s/startup_readfifo_fail.run.json", dir); unlink(p);
        rmdir(dir);
    }

    if (failures) {
        fprintf(stderr, "startup_test: %d 项失败 ❌\n", failures);
        return 1;
    }
    printf("startup_test: 全部通过 ✅\n");
    return 0;
}
