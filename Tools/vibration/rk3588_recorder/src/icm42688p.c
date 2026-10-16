/* ============================================================================
 * icm42688p.c — ICM-42688-P 配置与 FIFO 读取实现
 *
 * 页码全部指 TDK DS-000347 Rev 1.3（归档见 ../../01_硬件/datasheet/）。
 * 口径详见 ../寄存器配置.md。
 *
 * 本版固化的档位：ODR 8 kHz、±16 g、±2000 dps（== 寄存器配置.md §2）。
 * 其它档位需先补手册 p.76/p.77 的编码表——**不在此处猜**。
 * ==========================================================================*/
#define _GNU_SOURCE
#include "icm42688p.h"
#include "vib_format.h"
#include "util.h"
#include "tmst.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * 写入序列：每个操作 = 在某个 bank 的某个寄存器上做一次「掩码写」
 *   val = (readback & ~clr) | set
 * clr=0xFF 即普通整字节写。
 * 用同一张表驱动 apply / hash / verify，保证三者永远一致。
 * -------------------------------------------------------------------------*/
typedef struct {
    uint8_t bank, reg, clr, set;
} icm_op_t;

static size_t build_ops(const icm_cfg_t *c, icm_op_t *ops, size_t max)
{
    size_t k = 0;
    const uint8_t delt = c->aaf_delt;
    const uint16_t dsq = c->aaf_delt_sq;
    const uint8_t sq_lo = (uint8_t)(dsq & 0xFF);
    const uint8_t sq_hi_bs = (uint8_t)(((c->aaf_bit_shift & 0x0F) << 4) | ((dsq >> 8) & 0x0F));
    const uint8_t aaf_dis_g = c->aaf_enable ? 0x00 : 0x02;   /* bank1 0x0B bit1 GYRO_AAF_DIS (p.28) */
    const uint8_t aaf_dis_a = c->aaf_enable ? 0x00 : 0x01;   /* bank2 0x03 bit0 ACCEL_AAF_DIS (p.29) */
    const uint8_t bw = (uint8_t)(c->ui_filt_bw_nibble & 0x0F);

#define OP(b, r, cl, se) do { if (k < max) ops[k++] = (icm_op_t){ b, r, cl, se }; } while (0)

    /* ---- BANK 1：陀螺静态抗混叠滤波（p.27–28） ---- */
    OP(ICM_BANK1, ICM_B1_GYRO_CONFIG_STATIC2, 0x02, aaf_dis_g);
    OP(ICM_BANK1, ICM_B1_GYRO_AAF_DELT,      0xFF, (uint8_t)(delt & 0x3F));
    OP(ICM_BANK1, ICM_B1_GYRO_AAF_DELTSQR,   0xFF, sq_lo);
    OP(ICM_BANK1, ICM_B1_GYRO_AAF_BITSHIFT,  0xFF, sq_hi_bs);

    /* ---- BANK 2：加速度静态抗混叠滤波（p.27、p.29） ---- */
    OP(ICM_BANK2, ICM_B2_ACCEL_CONFIG_STATIC2, 0xFF,
       (uint8_t)(((delt & 0x3F) << 1) | aaf_dis_a));
    OP(ICM_BANK2, ICM_B2_ACCEL_AAF_DELTSQR,  0xFF, sq_lo);
    OP(ICM_BANK2, ICM_B2_ACCEL_AAF_BITSHIFT, 0xFF, sq_hi_bs);

    /* ---- BANK 0 ---- */
    /* FIFO_MODE = 01 Stream-to-FIFO（p.64）：连续模式，溢出时覆盖最旧记录，
     * 丢失量由 FIFO_LOST_PKT 计数。 */
    OP(ICM_BANK0, ICM_REG_FIFO_CONFIG, 0xFF, 0x40);
    /* INTF_CONFIG0（p.73，复位 0x30）：
     *   bit7 HOLD_LAST_DATA_EN 清 0（无效样本不静默复制上一条）
     *   bit6 FIFO_COUNT_REC    置 1（计数按记录，不再除 16 猜单位）
     *   bit5 FIFO_COUNT_ENDIAN 置 1（大端，与 FIFO_COUNTH/L 命名一致）
     *   bit4 SENSOR_DATA_ENDIAN 置 1 → **大端**（轴/时间戳统一大端读，消歧义）
     * 整个字节显式写入并回读校验（c->intf_config0 = 0x70）。 */
    OP(ICM_BANK0, ICM_REG_INTF_CONFIG0, 0xFF, c->intf_config0);
    /* 记录时间戳：TMST_CONFIG=0x33 → RES=0(名义 1µs) + 内部时钟（无 CLKIN），
     * Rev 1.6 §12.7：物理 µs = raw_ticks * 32/30，8 kHz 相邻 raw 差 117/118。 */
    OP(ICM_BANK0, ICM_REG_TMST_CONFIG, 0xFF, c->tmst_config);
    /* 以下三个为复位值，显式写出以便回读校验（p.78/p.79/p.80） */
    OP(ICM_BANK0, ICM_REG_GYRO_CONFIG1,        0xFF, 0x16);
    OP(ICM_BANK0, ICM_REG_ACCEL_CONFIG1,       0xFF, 0x0D);
    OP(ICM_BANK0, ICM_REG_GYRO_ACCEL_CONFIG0,  0xFF, (uint8_t)((bw << 4) | bw));
    /* FIFO：16-bit 包 + 时间戳 + 温度 + 陀螺 + 加速度（p.82） */
    OP(ICM_BANK0, ICM_REG_FIFO_CONFIG1, 0xFF, 0x0F);
    OP(ICM_BANK0, ICM_REG_FIFO_CONFIG2, 0xFF, (uint8_t)(c->wm_bytes & 0xFF));
    OP(ICM_BANK0, ICM_REG_FIFO_CONFIG3, 0xFF, (uint8_t)((c->wm_bytes >> 8) & 0x0F));
    /* 中断清除方式：10=读 FIFO 数据时清除（p.83）；
     * FIFO_THS[3:2]=10 → 0x08，FIFO_FULL[1:0]=10 → 0x02 ⇒ 0x0A */
    OP(ICM_BANK0, ICM_REG_INT_CONFIG0,  0xFF, c->int_config0);
    /* ODR ≥ 4 kHz 时手册强制要求（p.84）：脉冲 8 µs + 关闭 de-assert，且 ASYNC_RESET 置 0 */
    OP(ICM_BANK0, ICM_REG_INT_CONFIG1,  0xFF, c->int_config1);
    /* FIFO_THS → INT1（p.84），复位值 0x10 已含 RESET_DONE_INT1_EN */
    OP(ICM_BANK0, ICM_REG_INT_SOURCE0,  0xFF, c->int_source0);
    /* 量程与 ODR：±2000 dps / ±16 g + 8 kHz（p.76/p.77） */
    OP(ICM_BANK0, ICM_REG_GYRO_CONFIG0,  0xFF, 0x03);
    OP(ICM_BANK0, ICM_REG_ACCEL_CONFIG0, 0xFF, 0x03);
    /* PWR_MGMT0 必须最后（p.75：写后 200 µs 内不得再写寄存器）：
     * TEMP 使能 + GYRO_MODE=11(LN) + ACCEL_MODE=11(LN) */
    OP(ICM_BANK0, ICM_REG_PWR_MGMT0, 0xFF, 0x0F);
#undef OP
    return k;
}

/* ---------------------------------------------------------------------------
 * 默认配置
 * -------------------------------------------------------------------------*/
void icm_default_cfg(icm_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->odr_hz           = 8000;
    c->wm_bytes         = 512;      /* ≈32 条 16 B 记录 → 4 ms @8 kHz */
    c->fs_accel_g       = 16;
    c->fs_gyro_dps      = 2000;
    c->aaf_enable       = 1;
    c->aaf_delt         = 39;       /* 3 dB BW ≈ 2097 Hz（p.28 表） */
    c->aaf_delt_sq      = 1536;
    c->aaf_bit_shift    = 4;
    c->ui_filt_bw_nibble = 1;       /* 档 1 → max(400,ODR)/4 = 2000 Hz @8k（p.79） */
    c->tmst_config      = 0x33;     /* TO_REGS=1, RES=0(1µs), FSYNC=1, EN=1（p.80） */
    c->int_config1      = 0x60;     /* ODR≥4kHz 要求（p.84） */
    c->int_config0      = 0x0A;     /* THS/FULL 在读 FIFO 数据时清除（p.83） */
    c->int_source0      = 0x14;     /* FIFO_THS → INT1（p.84） */
    c->intf_config0     = 0x70;     /* COUNT_REC=1(按记录)+COUNT_ENDIAN=1(大端)+DATA_ENDIAN=1(大端) */
}

uint32_t icm_reg_config_hash(const icm_cfg_t *c)
{
    icm_op_t ops[32];
    size_t n = build_ops(c, ops, sizeof(ops) / sizeof(ops[0]));
    uint32_t h = 2166136261u;              /* FNV-1a 32 */
    for (size_t i = 0; i < n; i++) {
        const uint8_t *p = (const uint8_t *)&ops[i];
        for (size_t j = 0; j < sizeof(ops[i]); j++) {
            h ^= p[j];
            h *= 16777619u;
        }
    }
    return h;
}

/* ---------------------------------------------------------------------------
 * 基础操作
 * -------------------------------------------------------------------------*/
static int bank_select(icm_t *d, uint8_t bank)
{
    return spi_reg_write(d->spi, ICM_REG_REG_BANK_SEL, (uint8_t)(bank & 0x07));
}

static int reg_read_any(icm_t *d, uint8_t bank, uint8_t reg, uint8_t *val)
{
    if (bank_select(d, bank) != 0)
        return -1;
    return spi_reg_read(d->spi, reg, val);
}

static int reg_write_any(icm_t *d, uint8_t bank, uint8_t reg, uint8_t val)
{
    if (bank_select(d, bank) != 0)
        return -1;
    return spi_reg_write(d->spi, reg, val);
}

int icm_read_whoami(icm_t *d, uint8_t *id)
{
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;
    if (spi_reg_read(d->spi, ICM_REG_WHO_AM_I, id) != 0)
        return -1;
    d->who_am_i = *id;
    return 0;
}

int icm_soft_reset(icm_t *d)
{
    /* DEVICE_CONFIG bit0 = 1 触发软复位，之后必须等 1 ms（p.63） */
    if (reg_write_any(d, ICM_BANK0, ICM_REG_DEVICE_CONFIG, 0x01) != 0)
        return -1;
    sleep_us(1200);
    /* 复位后应回到 BANK0 */
    return bank_select(d, ICM_BANK0);
}

/* ---------------------------------------------------------------------------
 * 配置写入 / 回读校验
 * -------------------------------------------------------------------------*/
int icm_apply_config(icm_t *d)
{
    icm_op_t ops[32];
    size_t n = build_ops(&d->cfg, ops, sizeof(ops) / sizeof(ops[0]));
    uint8_t cur_bank = 0xFF;

    for (size_t i = 0; i < n; i++) {
        if (ops[i].bank != cur_bank) {
            if (bank_select(d, ops[i].bank) != 0) {
                fprintf(stderr, "[icm] 切换到 bank %u 失败\n", ops[i].bank);
                return -1;
            }
            cur_bank = ops[i].bank;
        }

        if (ops[i].clr == 0xFF) {
            if (spi_reg_write(d->spi, ops[i].reg, ops[i].set) != 0) {
                fprintf(stderr, "[icm] 写 bank%u reg 0x%02X=0x%02X 失败\n",
                        ops[i].bank, ops[i].reg, ops[i].set);
                return -1;
            }
        } else {
            uint8_t old = 0, nv = 0;
            if (spi_reg_rmw(d->spi, ops[i].reg, ops[i].clr, ops[i].set, &old, &nv) != 0) {
                fprintf(stderr, "[icm] 读-改-写 bank%u reg 0x%02X 失败\n",
                        ops[i].bank, ops[i].reg);
                return -1;
            }
        }
    }

    /* p.75：写 PWR_MGMT0 后 200 µs 内不得再写寄存器 */
    sleep_us(300);
    /* p.75：陀螺从 OFF 进入任意模式后必须保持 ON ≥ 45 ms 才能使用 */
    sleep_us(50000);
    d->reg_hash = icm_reg_config_hash(&d->cfg);
    return 0;
}

int icm_verify_config(icm_t *d, char *report, size_t n)
{
    icm_op_t ops[32];
    size_t cnt = build_ops(&d->cfg, ops, sizeof(ops) / sizeof(ops[0]));
    size_t off = 0;
    int bad = 0;

    off += (size_t)snprintf(report + off, n > off ? n - off : 0, "[回读校验] ");
    for (size_t i = 0; i < cnt; i++) {
        uint8_t rb = 0;
        if (reg_read_any(d, ops[i].bank, ops[i].reg, &rb) != 0) {
            off += (size_t)snprintf(report + off, n > off ? n - off : 0,
                                    "b%u:0x%02X=读失败 ", ops[i].bank, ops[i].reg);
            bad++;
            continue;
        }
        int ok;
        if (ops[i].clr == 0xFF)
            ok = (rb == ops[i].set);
        else
            ok = ((rb & ops[i].set) == ops[i].set) && ((rb & ops[i].clr) == 0);

        if (!ok) {
            off += (size_t)snprintf(report + off, n > off ? n - off : 0,
                                    "b%u:0x%02X(读0x%02X≠期望%s0x%02X) ",
                                    ops[i].bank, ops[i].reg, rb,
                                    ops[i].clr == 0xFF ? "" : "掩码", ops[i].set);
            bad++;
        }
    }
    if (bad)
        off += (size_t)snprintf(report + off, n > off ? n - off : 0,
                                "不一致 %d 项", bad);
    else
        off += (size_t)snprintf(report + off, n > off ? n - off : 0,
                                "全部一致（%u 项）", (unsigned)cnt);
    return bad == 0 ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * FIFO
 * -------------------------------------------------------------------------*/
int icm_fifo_flush(icm_t *d)
{
    /* FIFO_FLUSH 是 SIGNAL_PATH_RESET 的 bit1（0x02）；0x04 是 TMST_STROBE，**不清 FIFO**
     * （官方 PX4 InvenSense_ICM42688P_registers.hpp SIGNAL_PATH_RESET_BIT 与 TDK 手册
     * §14.33 一致）。
     *
     * 核验策略：active FIFO 会持续采样，不能「1 ms 后要求 count==0」。正确做法是
     *   1) 把 FIFO 切到 Bypass（停掉输入）→ 2) 写 FIFO_FLUSH=0x02 → 3) 读回 COUNT==0
     *   → 4) 恢复 Stream-to-FIFO 并读回确认。这样 flush 是否真正生效可被独立验证。 */
    uint8_t v = 0;
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;

    if (spi_reg_write(d->spi, ICM_REG_FIFO_CONFIG, 0x00) != 0) {  /* FIFO_MODE=00 Bypass */
        fprintf(stderr, "[icm] FIFO 切 Bypass 失败\n");
        return -1;
    }
    if (spi_reg_write(d->spi, ICM_REG_SIGNAL_PATH_RESET, ICM_SPR_FIFO_FLUSH) != 0) {
        fprintf(stderr, "[icm] 写 FIFO_FLUSH(0x02) 失败\n");
        return -1;
    }
    sleep_us(1000);
    uint16_t raw = 0;
    if (icm_fifo_count_raw(d, &raw) != 0) {
        fprintf(stderr, "[icm] 读 FIFO_COUNT 校验 flush 失败\n");
        return -1;
    }
    if (raw != 0) {
        fprintf(stderr, "[icm] flush 后 FIFO_COUNT=%u ≠ 0（Bypass 下应清空）——flush 未生效\n", raw);
        return -1;
    }
    if (spi_reg_write(d->spi, ICM_REG_FIFO_CONFIG, 0x40) != 0) {  /* Stream-to-FIFO */
        fprintf(stderr, "[icm] FIFO 恢复 Stream 失败\n");
        return -1;
    }
    if (spi_reg_read(d->spi, ICM_REG_FIFO_CONFIG, &v) != 0 || v != 0x40) {
        fprintf(stderr, "[icm] FIFO 恢复 Stream 回读不一致（读 0x%02X ≠ 0x40）\n", v);
        return -1;
    }
    return 0;
}

int icm_fifo_count_raw(icm_t *d, uint16_t *raw)
{
    uint8_t b[2] = { 0, 0 };
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;
    /* 一次 CS 内突发读两个字节，天然一致（p.52 支持突发读）。
     * FIFO_COUNT_ENDIAN 保持复位值 1（大端）→ 先 COUNTH 后 COUNTL。 */
    if (spi_burst_read(d->spi, ICM_REG_FIFO_COUNTH, b, 2) != 0)
        return -1;
    *raw = (uint16_t)((b[0] << 8) | b[1]);
    return 0;
}

int icm_fifo_count_records(icm_t *d, int *recs)
{
    uint16_t raw = 0;
    if (icm_fifo_count_raw(d, &raw) != 0)
        return -1;
    *recs = (int)(raw / (uint16_t)d->count_div);
    return 0;
}

int icm_read_lost(icm_t *d, uint32_t *total)
{
    uint8_t b[2] = { 0, 0 };
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;
    if (spi_burst_read(d->spi, ICM_REG_FIFO_LOST_PKT0, b, 2) != 0)
        return -1;
    *total = (uint32_t)b[0] | ((uint32_t)b[1] << 8);
    d->lost_total = *total;
    return 0;
}

int icm_read_status(icm_t *d, uint8_t *st)
{
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;
    /* 注意：INT_STATUS 是 R/C（读清），本实现不用中断线，故无副作用 */
    return spi_reg_read(d->spi, ICM_REG_INT_STATUS, st);
}

int icm_read_fifo(icm_t *d, uint8_t *buf, size_t nbytes)
{
    if (bank_select(d, ICM_BANK0) != 0)
        return -1;
    return spi_burst_read(d->spi, ICM_REG_FIFO_DATA, buf, nbytes);
}

/* ---------------------------------------------------------------------------
 * FIFO 原始字节 → 记录数组
 * Packet 3 布局（p.37–38）：Header(1)+Accel(6)+Gyro(6)+Temp(1)+TMST(2)
 * Header 位：bit7 MSG、bit6 ACCEL、bit5 GYRO、bit3:2 TIMESTAMP_FSYNC(=10 含 ODR 时间戳)、
 *            bit1 ODR_ACCEL 变化、bit0 ODR_GYRO 变化
 * -------------------------------------------------------------------------*/
size_t icm_parse_fifo(const uint8_t *raw, size_t nbytes, vib_record_t *out,
                      size_t out_cap, uint32_t *bad_headers)
{
    size_t nrec = nbytes / VIB_RECORD_SIZE;
    size_t ok = 0;
    uint32_t bad = 0;

    for (size_t i = 0; i < nrec && ok < out_cap; i++) {
        const uint8_t *f = raw + i * VIB_RECORD_SIZE;
        uint8_t h = f[0];

        if (h & ICM_FIFO_HDR_MSG) {           /* bit7：FIFO 空标记，不是数据 */
            bad++;
            continue;
        }
        if (!(h & ICM_FIFO_HDR_ACCEL) || !(h & ICM_FIFO_HDR_GYRO)) {
            bad++;                            /* 必须同时含 Accel 与 Gyro，否则不是 Packet 3 */
            continue;
        }

        vib_record_t r;
        memset(&r, 0, sizeof(r));
        /* 大端：高字节在前（INTF_CONFIG0 SENSOR_DATA_ENDIAN=1，0x70）。存盘仍小端，
         * 因为写入的是本机 int16_t 的 vib_record_t。 */
        r.accel[0] = (int16_t)(((uint16_t)f[1] << 8) | (uint16_t)f[2]);
        r.accel[1] = (int16_t)(((uint16_t)f[3] << 8) | (uint16_t)f[4]);
        r.accel[2] = (int16_t)(((uint16_t)f[5] << 8) | (uint16_t)f[6]);
        r.gyro[0]  = (int16_t)(((uint16_t)f[7] << 8) | (uint16_t)f[8]);
        r.gyro[1]  = (int16_t)(((uint16_t)f[9] << 8) | (uint16_t)f[10]);
        r.gyro[2]  = (int16_t)(((uint16_t)f[11] << 8) | (uint16_t)f[12]);
        r.temp_raw = f[13];
        r.tmst_raw = (uint16_t)(((uint16_t)f[14] << 8) | (uint16_t)f[15]);

        if (h & ICM_FIFO_HDR_ODR_ACCEL)
            r.rec_flags |= VIB_RF_ODR_ACCEL_CHG;
        if (h & ICM_FIFO_HDR_ODR_GYRO)
            r.rec_flags |= VIB_RF_ODR_GYRO_CHG;

        /* 无效样本识别：哨兵值 + 后续由削顶规则覆盖（格式规范 §5） */
        if (r.accel[0] == VIB_INVALID_SENTINEL || r.accel[1] == VIB_INVALID_SENTINEL ||
            r.accel[2] == VIB_INVALID_SENTINEL || r.gyro[0] == VIB_INVALID_SENTINEL ||
            r.gyro[1] == VIB_INVALID_SENTINEL || r.gyro[2] == VIB_INVALID_SENTINEL)
            r.rec_flags |= VIB_RF_INVALID;

        out[ok++] = r;
    }

    if (bad_headers)
        *bad_headers += bad;
    return ok;
}

/* ---------------------------------------------------------------------------
 * 自检（G1 闸门的一部分，跑在真正采集之前）
 * -------------------------------------------------------------------------*/
int icm_selftest(icm_t *d, char *report, size_t n)
{
    size_t off = 0;
    int    fail = 0;
    uint8_t id = 0;

#define APP(...) do { if (off < n) off += (size_t)snprintf(report + off, n - off, __VA_ARGS__); } while (0)

    APP("[自检] 软复位… ");
    if (icm_soft_reset(d) != 0) { APP("失败(SPI 无响应)\n"); return -1; }

    if (icm_read_whoami(d, &id) != 0) { APP("WHO_AM_I 读取失败\n"); return -1; }
    if (id != ICM_WHO_AM_I_VALUE) {
        APP("WHO_AM_I=0x%02X ≠ 0x%02X（接线/模式/时钟有问题）\n", id, ICM_WHO_AM_I_VALUE);
        return -1;
    }
    APP("WHO_AM_I=0x%02X ✅\n", id);

    APP("[自检] 写配置… ");
    if (icm_apply_config(d) != 0) { APP("失败\n"); return -1; }
    APP("完成，reg_hash=0x%08X\n", d->reg_hash);

    char vrep[1024];
    vrep[0] = '\0';
    if (icm_verify_config(d, vrep, sizeof(vrep)) != 0) {
        APP("[自检] %s  ❌\n", vrep);
        fail = 1;
    } else {
        APP("[自检] %s ✅\n", vrep);
    }

    if (icm_fifo_flush(d) != 0) { APP("[自检] FIFO 清空失败\n"); return -1; }

    /* ---- 读回 INTF_CONFIG0：显式确认 FIFO_COUNT 单位与端序（不靠 20 ms 猜） ---- */
    uint8_t intf0 = 0;
    if (reg_read_any(d, ICM_BANK0, ICM_REG_INTF_CONFIG0, &intf0) != 0) {
        APP("[自检] 读回 INTF_CONFIG0 失败\n");
        return -1;
    }
    d->count_endian = (intf0 & ICM_INTF0_COUNT_ENDIAN) ? 1 : 0;
    if (intf0 & ICM_INTF0_COUNT_REC) {
        d->count_div = 1;
        APP("[自检] INTF_CONFIG0=0x%02X：FIFO_COUNT 按记录计数、%s端 ✅\n",
            intf0, d->count_endian ? "大" : "小");
    } else {
        d->count_div = 16;
        APP("[自检] INTF_CONFIG0=0x%02X：FIFO_COUNT 按字节计数（除 16）、%s端\n",
            intf0, d->count_endian ? "大" : "小");
    }
    if (intf0 & ICM_INTF0_DATA_ENDIAN) {
        APP("[自检] SENSOR_DATA_ENDIAN=1（大端，轴/时间戳统一大端读）✅\n");
    } else {
        APP("[自检] SENSOR_DATA_ENDIAN=0（小端），与期望的大端（INTF_CONFIG0=0x70）不符 ❌\n");
        fail = 1;
    }

    /* ---- 清 FIFO + 丢包基线 ---- */
    uint32_t lost0 = 0, lost1 = 0;
    if (icm_read_lost(d, &lost0) != 0) { APP("[自检] 读 FIFO_LOST_PKT 基线失败\n"); return -1; }

    /* ---- sub-full 窗口采样：4 ms@8kHz ≈ 32 条，远小于 16 ms 满窗，不会主动溢出 ---- */
    sleep_us(ICM_SELFTEST_WINDOW_US);
    uint16_t raw = 0;
    if (icm_fifo_count_raw(d, &raw) != 0) { APP("[自检] 读 FIFO_COUNT 失败\n"); return -1; }
    if (icm_read_lost(d, &lost1) != 0) { APP("[自检] 读 FIFO_LOST_PKT 失败\n"); return -1; }
    if (lost1 != lost0) {
        APP("[自检] 4 ms 窗口内 FIFO_LOST_PKT 增长 %u→%u（自检不应主动溢出）❌\n",
            lost0, lost1);
        fail = 1;
    }

    int nrec = (int)(raw / (uint16_t)d->count_div);
    const int expect_min = (int)(d->cfg.odr_hz / 500);   /* 2 ms 下限，容忍调度抖动 */
    const int expect_max = (int)(d->cfg.odr_hz / 160);   /* 6.25 ms 上限 */
    if (nrec < expect_min || nrec > expect_max) {
        APP("[自检] 4 ms 窗口 FIFO_COUNT=%u → %d 条，偏离 8 kHz 预期（%d..%d）⚠️\n",
            raw, nrec, expect_min, expect_max);
        fail = 1;
    }

    /* ---- 采样时间戳校验：raw Δ 必须落在正常窗口（名义 117/118，含 ±1% PLL 容差
     *      ⇒ 116..119），绝不是 125（那是「把 raw ticks 当 µs」的旧错误口径） ---- */
    if (nrec < 8) {
        APP("[自检] 记录数不足（%d），无法校验时间戳 ❌\n", nrec);
        return -1;
    }
    int nread = nrec;
    if (nread > VIB_SAMPLES_PER_BLOCK)
        nread = VIB_SAMPLES_PER_BLOCK;

    uint8_t  rawbuf[VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE];
    vib_record_t recs[VIB_SAMPLES_PER_BLOCK];
    if (icm_read_fifo(d, rawbuf, (size_t)nread * VIB_RECORD_SIZE) != 0) {
        APP("[自检] 读 FIFO 数据失败\n");
        return -1;
    }
    uint32_t bad = 0;
    size_t got = icm_parse_fifo(rawbuf, (size_t)nread * VIB_RECORD_SIZE, recs,
                                VIB_SAMPLES_PER_BLOCK, &bad);

    tmst_accum_t ta;
    tmst_accum_init(&ta);
    for (size_t i = 0; i < got; i++)
        tmst_accum_add(&ta, recs[i].tmst_raw);
    const int median_raw = tmst_accum_median_raw(&ta);
    APP("[自检] 解析 %zu 条记录，包头异常 %u 条；raw Δ: 中位=%d（预期窗口 [%u,%u]）范围=[%d,%d]\n",
        got, bad, median_raw, VIB_TMST_RAW_LO, VIB_TMST_RAW_HI,
        ta.n_deltas ? ta.dmin : 0, ta.n_deltas ? ta.dmax : 0);
    if (median_raw < 0) {
        APP("[自检] 无法得到 raw Δ ❌\n");
        fail = 1;
    } else if (median_raw < (int)VIB_TMST_RAW_LO || median_raw > (int)VIB_TMST_RAW_HI) {
        APP("[自检] raw Δ 中位数 %d 超出正常窗口 [%u,%u]（≈125 说明把 raw ticks 当 µs 了）❌\n",
            median_raw, VIB_TMST_RAW_LO, VIB_TMST_RAW_HI);
        fail = 1;
    } else {
        APP("[自检] raw Δ 符合 8 kHz 预期（≈%.1f µs → 实测 ODR ≈ %.1f Hz）✅\n",
            (double)median_raw * (double)VIB_TMST_SCALE_NUM / (double)VIB_TMST_SCALE_DEN,
            tmst_accum_rate_hz(&ta));
    }
    if (bad)
        fail = 1;

    APP("[自检] 结论：%s\n", fail ? "未通过 ❌" : "通过 ✅");
#undef APP
    return fail ? -1 : 0;
}
