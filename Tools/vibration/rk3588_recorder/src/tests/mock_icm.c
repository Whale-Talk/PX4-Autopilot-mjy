/* ============================================================================
 * tests/mock_icm.c — 无硬件 mock SPI 器件模型 + 自检/启动逻辑测试
 *
 * 通过 spi_dev_t.xfer_fn 注入一个内存 ICM-42688-P 模型，真实跑 icm42688p.c 的
 * icm_apply_config / icm_verify_config / icm_selftest / icm_fifo_flush /
 * icm_fifo_count_* / icm_read_lost / icm_read_status。
 *
 * 覆盖（fix 3 的 mock 需求）：
 *   1) 自检的 sub-full 窗口（4 ms）不引发 FIFO 溢出（lost Δ == 0）
 *   2) INTF_CONFIG0 回读：COUNT_REC=1（按记录）、COUNT_ENDIAN=1（大端）、DATA_ENDIAN=1（大端）
 *   3) raw Δ = 117/118（不是 125），自检通过
 *   4) 错误分支：flush 失败 / status/lost 读取失败 → 返回 -1（不得当作 0 健康）
 *   5) 芯片级：SIGNAL_PATH_RESET 写 0x04（TMST_STROBE）不清 FIFO，0x02（FIFO_FLUSH）才清
 *   6) 独立 raw-byte 夹具：大端解析 + 无效哨兵 -32768 / -32766 合法满量程
 *
 * 器件模型时间语义：FIFO 在 PWR_MGMT0 使能后按 8 kHz 实时填充（mono_ns 计时，
 * 每次 SPI 访问推进一次），与真实芯片一致——因此 apply_config 的 50 ms 等待会
 * 灌满 FIFO，而 4 ms 自检窗口远小于 16 ms 满窗。
 *
 * 用法: mock_icm   （退出码 0=通过，1=失败）
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "spi_dev.h"
#include "icm42688p.h"
#include "util.h"

#define MOCK_FIFO_CAP 128

typedef struct {
    uint8_t  reg[3][256];
    uint8_t  cur_bank;
    uint8_t  enabled;             /* PWR_MGMT0 使能后 1 */
    uint8_t  fifo[MOCK_FIFO_CAP][16];
    uint32_t fifo_count;          /* 记录数 */
    uint32_t fifo_read_off;       /* FIFO_DATA 突发读游标 */
    uint32_t lost;                /* FIFO_LOST_PKT 累计 */
    uint64_t raw_cum;             /* 时间戳累计 raw ticks 分子（单位 1/32） */
    uint64_t last_ns;
} mock_dev_t;

static mock_dev_t g;

static uint16_t mock_count_value(void)
{
    uint8_t intf0 = g.reg[0][ICM_REG_INTF_CONFIG0];
    if (intf0 & ICM_INTF0_COUNT_REC)
        return (uint16_t)g.fifo_count;                 /* 记录数 */
    return (uint16_t)(g.fifo_count * 16u);             /* 字节数 */
}

static uint16_t mock_lost_value(void)
{
    return (uint16_t)g.lost;
}

/* FIFO_CONFIG 位 [7:6] = FIFO_MODE：00=Bypass（不填 FIFO）01=Stream（连续填）。 */
static int mock_fifo_streaming(void)
{
    return (((g.reg[0][ICM_REG_FIFO_CONFIG] >> 6) & 0x03u) == 1u);
}

static void mock_push_record(void)
{
    /* 时间戳：raw_cum += 125*30；tmst_raw = raw_cum/32（117/118 交替） */
    g.raw_cum += 125u * 30u;
    const uint16_t tmst = (uint16_t)((g.raw_cum / 32u) & 0xFFFFu);

    if (g.fifo_count >= MOCK_FIFO_CAP) {
        g.lost++;                                     /* Stream-to-FIFO 溢出 */
        return;
    }
    uint8_t *r = g.fifo[g.fifo_count++];
    r[0] = ICM_FIFO_HDR_ACCEL | ICM_FIFO_HDR_GYRO | ICM_FIFO_HDR_TS_ODR;  /* 0x68 */
    /* 传感器数据按 SENSOR_DATA_ENDIAN=1（**大端**）出线缆：高字节在前。 */
    const int16_t ax = 1, ay = 2, az = 3, gx = 4, gy = 5, gz = 6;
    r[1] = (uint8_t)((uint16_t)ax >> 8); r[2]  = (uint8_t)ax;
    r[3] = (uint8_t)((uint16_t)ay >> 8); r[4]  = (uint8_t)ay;
    r[5] = (uint8_t)((uint16_t)az >> 8); r[6]  = (uint8_t)az;
    r[7] = (uint8_t)((uint16_t)gx >> 8); r[8]  = (uint8_t)gx;
    r[9] = (uint8_t)((uint16_t)gy >> 8); r[10] = (uint8_t)gy;
    r[11] = (uint8_t)((uint16_t)gz >> 8); r[12] = (uint8_t)gz;
    r[13] = 21;
    r[14] = (uint8_t)(tmst >> 8);                  /* 时间戳也大端 */
    r[15] = (uint8_t)(tmst & 0xFF);
}

static void mock_advance(void)
{
    const uint64_t now = mono_ns();
    const uint64_t el_ns = now - g.last_ns;
    g.last_ns = now;
    if (!g.enabled || el_ns < 1000000ull)             /* < 1 ms 忽略，避免噪声 */
        return;
    if (!mock_fifo_streaming())
        return;                                       /* Bypass：不填 FIFO */
    const uint64_t samples = el_ns / 125000ull;        /* 8 kHz：125 µs/样本 */
    for (uint64_t i = 0; i < samples; i++)
        mock_push_record();
}

static void mock_soft_reset(void)
{
    memset(g.reg, 0, sizeof(g.reg));
    g.reg[0][ICM_REG_INTF_CONFIG0] = 0x30;            /* 复位值 */
    g.reg[0][ICM_REG_INT_CONFIG1]  = 0x10;
    g.reg[0][ICM_REG_INT_SOURCE0]  = 0x10;
    g.reg[0][ICM_REG_GYRO_CONFIG1] = 0x16;
    g.reg[0][ICM_REG_ACCEL_CONFIG1] = 0x0D;
    g.reg[0][ICM_REG_GYRO_ACCEL_CONFIG0] = 0x11;
    g.cur_bank = 0;
    g.enabled = 0;
    g.fifo_count = 0;
    g.fifo_read_off = 0;
    g.lost = 0;
    g.raw_cum = 0;
}

static uint8_t mock_read_byte(uint8_t addr)
{
    switch (addr) {
    case ICM_REG_WHO_AM_I:        return ICM_WHO_AM_I_VALUE;
    case ICM_REG_REG_BANK_SEL:    return g.cur_bank;
    case ICM_REG_FIFO_COUNTH: {
        const uint16_t v = mock_count_value();
        return (g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_ENDIAN)
               ? (uint8_t)(v >> 8) : (uint8_t)(v & 0xFF);
    }
    case ICM_REG_FIFO_COUNTL: {
        const uint16_t v = mock_count_value();
        return (g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_ENDIAN)
               ? (uint8_t)(v & 0xFF) : (uint8_t)(v >> 8);
    }
    case ICM_REG_FIFO_LOST_PKT0:  return (uint8_t)(mock_lost_value() & 0xFF);
    case ICM_REG_FIFO_LOST_PKT1:  return (uint8_t)(mock_lost_value() >> 8);
    case ICM_REG_INT_STATUS:      return 0x00;
    case ICM_REG_FIFO_DATA: {
        /* 逐字节读：按记录顺序返回，fifo_count 之后回绕到 0 */
        if (g.fifo_count == 0)
            return 0;
        const uint32_t idx = g.fifo_read_off++ % (g.fifo_count * 16u);
        const uint32_t rec = idx / 16u, byte = idx % 16u;
        return g.fifo[rec][byte];
    }
    default:
        return g.reg[g.cur_bank][addr];
    }
}

static void mock_write_byte(uint8_t addr, uint8_t val)
{
    if (addr == ICM_REG_REG_BANK_SEL) {
        g.cur_bank = (uint8_t)(val & 0x07);
        return;
    }
    if (addr == ICM_REG_DEVICE_CONFIG && (val & 0x01)) {
        mock_soft_reset();
        return;
    }
    if (addr == ICM_REG_SIGNAL_PATH_RESET) {
        /* 手册/PX4 一致：bit1=0x02 才是 FIFO_FLUSH；0x04 是 TMST_STROBE，不清 FIFO。
         * 写 1 是自清 strobe，不落寄存器。 */
        if (val & ICM_SPR_FIFO_FLUSH)
            g.fifo_count = 0;
        return;
    }
    if (addr == ICM_REG_PWR_MGMT0 && (val & 0x0F) != 0) {
        g.enabled = 1;
    }
    g.reg[g.cur_bank][addr] = val;
}

/* 注入到 spi_dev_t.xfer_fn 的 mock 传输实现 */
static int mock_xfer(spi_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)d;
    mock_advance();
    if (len == 0)
        return 0;
    const uint8_t first = tx[0];
    const uint8_t addr = (uint8_t)(first & 0x7Fu);
    if (first & 0x80u) {                               /* 读（单字节或突发） */
        if (addr == ICM_REG_FIFO_DATA) {
            /* FIFO_DATA 是突发读寄存器：地址不递增，连续字节都来自 FIFO */
            for (size_t i = 0; i + 1 < len && rx; i++)
                rx[i + 1] = mock_read_byte(ICM_REG_FIFO_DATA);
        } else {
            for (size_t i = 0; i + 1 < len && rx; i++)
                rx[i + 1] = mock_read_byte((uint8_t)(addr + i));
        }
    } else {                                           /* 写（单字节） */
        if (len >= 2)
            mock_write_byte(addr, tx[1]);
    }
    return 0;
}

/* 让后续读取失败的注入开关 */
static int g_fail_reads = 0;
static int mock_xfer_failing(spi_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)d; (void)tx; (void)rx; (void)len;
    if (g_fail_reads)
        return -1;
    return mock_xfer(d, tx, rx, len);
}

static int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; fprintf(stderr, "  ✗ " __VA_ARGS__); fprintf(stderr, "\n"); } \
    else { printf("  ✓ " __VA_ARGS__); printf("\n"); } \
} while (0)

static int run_selftest(char *report, size_t n)
{
    spi_dev_t spi;
    memset(&spi, 0, sizeof(spi));
    spi.xfer_fn = mock_xfer;

    icm_t icm;
    memset(&icm, 0, sizeof(icm));
    icm.spi = &spi;
    icm_default_cfg(&icm.cfg);
    icm.count_div = 1;
    icm.count_endian = 1;
    return icm_selftest(&icm, report, n);
}

int main(void)
{
    printf("== 自检 sub-full 窗口不溢出 + INTF_CONFIG0 回读 + raw Δ 117/118 ==\n");
    char report[4096];
    mock_soft_reset();
    g.last_ns = mono_ns();
    const int rc = run_selftest(report, sizeof(report));
    if (rc != 0)
        fprintf(stderr, "%s", report);
    CHECK(rc == 0, "自检通过（%s", report[0] ? "见上" : "报告为空");

    /* 检查 INTF_CONFIG0 的计数单位/端序判定结果（写进 reg[0] 的是 0x70） */
    CHECK(g.reg[0][ICM_REG_INTF_CONFIG0] == 0x70, "INTF_CONFIG0=0x70（COUNT_REC/COUNT_ENDIAN/DATA_ENDIAN 全置）");
    CHECK((g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_COUNT_REC) != 0,
          "FIFO_COUNT 按记录计数（COUNT_REC=1）");
    CHECK((g.reg[0][ICM_REG_INTF_CONFIG0] & ICM_INTF0_DATA_ENDIAN) != 0,
          "传感器数据大端（DATA_ENDIAN=1，消歧义）");

    printf("== 错误分支：flush 失败 ==\n");
    {
        spi_dev_t spi; memset(&spi, 0, sizeof(spi)); spi.xfer_fn = mock_xfer_failing;
        icm_t icm; memset(&icm, 0, sizeof(icm)); icm.spi = &spi;
        icm_default_cfg(&icm.cfg); icm.count_div = 1;
        g_fail_reads = 1;
        CHECK(icm_fifo_flush(&icm) != 0, "flush 在 SPI 失败时返回 -1（不当 0 健康）");
        uint32_t lost = 0;
        CHECK(icm_read_lost(&icm, &lost) != 0, "read_lost 在 SPI 失败时返回 -1");
        uint8_t st = 0;
        CHECK(icm_read_status(&icm, &st) != 0, "read_status 在 SPI 失败时返回 -1");
        g_fail_reads = 0;
    }

    printf("== 芯片级：写 0x04 不清 FIFO，0x02 才清（FIFO_FLUSH=bit1）==\n");
    {
        mock_soft_reset();
        g.enabled = 1;
        g.reg[0][ICM_REG_FIFO_CONFIG] = 0x40;   /* Stream */
        mock_push_record();
        mock_push_record();
        CHECK(g.fifo_count == 2, "预填 2 条记录");
        mock_write_byte(ICM_REG_SIGNAL_PATH_RESET, ICM_SPR_TMST_STROBE);  /* 0x04 */
        CHECK(g.fifo_count == 2, "写 0x04（TMST_STROBE）不清 FIFO（仍 %u 条）", g.fifo_count);
        mock_write_byte(ICM_REG_SIGNAL_PATH_RESET, ICM_SPR_FIFO_FLUSH);   /* 0x02 */
        CHECK(g.fifo_count == 0, "写 0x02（FIFO_FLUSH）清空 FIFO（0 条）");
    }

    printf("== 独立 raw-byte 夹具：大端解析 + 无效哨兵 -32768 / -32766 合法 ==\n");
    {
        /* 手工按「手册独立字节常量」构造 16 B Packet 3（线缆大端）。
         * 不经过 mock 生产逻辑，独立验证 icm_parse_fifo 的字节序与哨兵。 */
        uint8_t raw[16];
        memset(raw, 0, sizeof(raw));
        raw[0]  = ICM_FIFO_HDR_ACCEL | ICM_FIFO_HDR_GYRO | ICM_FIFO_HDR_TS_ODR;  /* 0x68 */
        raw[1]  = 0x80; raw[2]  = 0x00;   /* accel[0] = 0x8000 = -32768（无效哨兵） */
        raw[3]  = 0x80; raw[4]  = 0x02;   /* accel[1] = 0x8002 = -32766（合法满量程替换值） */
        raw[5]  = 0x00; raw[6]  = 0x01;   /* accel[2] = 1 */
        raw[7]  = 0x00; raw[8]  = 0x04;   /* gyro[0] = 4 */
        raw[9]  = 0x00; raw[10] = 0x05;   /* gyro[1] = 5 */
        raw[11] = 0x00; raw[12] = 0x06;   /* gyro[2] = 6 */
        raw[13] = 21;
        raw[14] = 0x00; raw[15] = 0x75;   /* tmst = 0x0075 = 117 */

        vib_record_t out[1];
        uint32_t bad = 0;
        size_t got = icm_parse_fifo(raw, 16, out, 1, &bad);
        CHECK(got == 1 && bad == 0, "解析 1 条记录、0 包头异常");
        CHECK(out[0].accel[0] == -32768, "accel[0] 0x8000 → -32768（无效哨兵）");
        CHECK(out[0].accel[1] == -32766, "accel[1] 0x8002 → -32766（合法满量程，不误判）");
        CHECK(out[0].accel[2] == 1, "accel[2] → 1");
        CHECK(out[0].gyro[0] == 4 && out[0].gyro[2] == 6, "gyro 大端解析正确");
        CHECK(out[0].tmst_raw == 117, "raw 00 75 → tmst_raw=117");
        CHECK((out[0].rec_flags & VIB_RF_INVALID) != 0, "含 -32768 → 标记 VIB_RF_INVALID");

        /* 第二条：只有 -32766（无 -32768）→ 不应标无效 */
        uint8_t raw2[16];
        memset(raw2, 0, sizeof(raw2));
        raw2[0] = raw[0];
        raw2[1] = 0x80; raw2[2] = 0x02;   /* accel[0] = -32766 */
        raw2[3] = 0x00; raw2[4] = 0x01;   /* accel[1] = 1 */
        raw2[5] = 0x00; raw2[6] = 0x02;   /* accel[2] = 2 */
        raw2[7] = 0x00; raw2[8] = 0x04;
        raw2[9] = 0x00; raw2[10] = 0x05;
        raw2[11] = 0x00; raw2[12] = 0x06;
        raw2[13] = 21;
        raw2[14] = 0x00; raw2[15] = 0xEC;  /* 117+119=236 → 下一条时间戳，仅用于独立性 */
        vib_record_t out2[1];
        bad = 0;
        got = icm_parse_fifo(raw2, 16, out2, 1, &bad);
        CHECK(got == 1 && bad == 0, "第二条解析 1 条、0 包头异常");
        CHECK((out2[0].rec_flags & VIB_RF_INVALID) == 0,
              "仅 -32766（合法满量程）→ 不标无效（哨兵已修正为 -32768）");
    }

    if (failures) {
        fprintf(stderr, "mock_icm: %d 项失败 ❌\n", failures);
        return 1;
    }
    printf("mock_icm: 全部通过 ✅\n");
    return 0;
}
