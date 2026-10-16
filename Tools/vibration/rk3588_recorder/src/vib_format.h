/* ============================================================================
 * vib_format.h — 采集数据格式 v2 的机器可读定义
 *
 * 唯一真源是同目录文档：../数据格式规范.md
 * 改字段/布局的顺序永远是：① 改文档 ② 升 VIB_FORMAT_VERSION ③ 改本文件
 *
 * 依据：TDK DS-000347 Rev 1.6（ICM-42688-P 数据手册，归档于
 *       01_硬件/datasheet/TDK_ICM-42688-P_DS-000347.pdf）
 *
 * v3 关键变更（相对 v2）：记录内 16-bit 时间戳字段语义从「µs」改为「raw ticks」，
 * 物理 µs = raw_ticks * 32/30（Rev 1.6 §12.7，未用 CLKIN、TMST_RES=0）。文件头新增
 * 时间戳换算元数据（scale、时钟源、分辨率、FIFO_COUNT 单位/端序）。v2 文件把 raw
 * ticks 直接当 µs 是一种已声明为「假设」的旧口径，解析器须显式标注而非当作可信物理时间。
 * ==========================================================================*/
#ifndef VIB_FORMAT_H
#define VIB_FORMAT_H

#include <stdint.h>
#include <stddef.h>

/* ---- 版本与尺寸（文档 §1、§2、§3） ---- */
#define VIB_MAGIC              "VIB2"
#define VIB_FORMAT_VERSION     3
#define VIB_HEADER_SIZE        256
#define VIB_BLOCK_HEADER_SIZE  32
#define VIB_RECORD_SIZE        16
#define VIB_SAMPLES_PER_BLOCK  32      /* 512 B payload == FIFO 水位，8 kHz 下约 4 ms */
#define VIB_DRIVER_VERSION     0x000201 /* major.minor.patch = 0.2.1 */

#define VIB_BLOCK_TYPE_DATA    0x0001
#define VIB_BLOCK_TYPE_END     0xFFFF

/* ---- 判定阈值（文档 §5） ---- */
#define VIB_CLIP_THRESHOLD     32760   /* |raw| >= 32760 即判削顶（满量程 -8 LSB） */
/* 器件无效样本哨兵 = -32768（0x8000，INT16_MIN）。
 * 手册「Invalid Data Generation」表（归档 text ~2927–2949）与 §14.34：无效数据输出
 * -32768。**-32766（0x8002）是合法的满量程替换值**，不是无效哨兵——之前把它当哨兵
 * 正好反了。官方 PX4 ICM42688P.cpp 也以 INT16_MIN（-32768）判无效。 */
#define VIB_INVALID_SENTINEL   (-32768)

/* ---- ODR 时间戳（手册 p.80 TMST_CONFIG：RES=0 → 名义 1 µs；p.38 记录内 2 B） ---- */
/* 关键：记录内 16-bit 是「raw ticks」，不是 µs。
 * Rev 1.6 §12.7：未用 CLKIN、TMST_RES=0 时物理 µs = raw_ticks * 32/30。
 * 8 kHz 名义物理间隔 125 µs → raw ≈ 117.1875（相邻差 117/118），绝不是 125。 */
#define VIB_TMST_SCALE_NUM     32u     /* ticks→µs 分子 */
#define VIB_TMST_SCALE_DEN     30u     /* ticks→µs 分母 */
#define VIB_TMST_WRAP          65536u  /* 16 位 raw ticks 回绕周期 */
#define VIB_TMST_HALF          32768   /* 回绕展开判定用半周期 */
#define VIB_TMST_US_NOMINAL    125u    /* 8 kHz 下相邻记录名义物理 Δ（µs，校正后） */
/* 8 kHz 相邻记录 raw Δ 的「正常窗口」——**现选用的保守诊断门限**，不是时钟精度指标。
 * 口径（Rev 1.6 §12.7 换算）：
 *   名义物理 Δ = 125 µs  →  raw = 125 × 30/32 = 117.1875（整数差 117/118 交替）
 *   实机相邻 raw Δ 目前只见 117/118；[115,120] 只是本实现**当前选定**的保守诊断门限，
 *   用于区分「名义附近的量化/抖动」与「漏样本」，integer fixture 以名义附近的整数偏移
 *   （Δ=116、Δ=119 这类名义附近整数偏移）覆盖测试（见 src/tests/tmst_tolerance_test.c）。
 *     · ⚠️ **不得由本窗口反推时钟容差**：绝对钟差（例如 TDK Rev1.6 表 4 给出
 *       CLK_SEL=01 且 gyro active 的初始容差 ±1.5%，及 p.24 §4.10 另述的 ±1%——
 *       两者条件不同）**不能**直接等同**同一芯片内相邻 raw Δ 的变化**；不得把
 *       「绝对钟差」硬套进本 tick 窗口，也不得据此声明绝对物理 ODR 已校准。
 *       **绝对时钟误差只能靠外部参考**（run.c 的 host_rate_hz 等独立速率）判定。
 *     · 窗口**不代表实测时钟精度**，也**不构成「无丢包」保证**（丢包须由
 *       FIFO_LOST_PKT / block_index / FIFO 满标志判定）。
 * ⚠️ 上界必须远小于 2× 名义（≈232）：漏 1 个样本的 Δ ≈ 232–237 仍须被判缺口。
 * ⚠️ 曾经把窗口写成零容差的 {117,118}——那样只要名义附近出现 Δ=116/119 这类整数偏移
 *    （fixture 即可复现），约一半 Δ 会被误判为 gap → 整批「时间轴不可信」、自检失败。
 *    反例见 src/tests/tmst_tolerance_test.c 与 tools/tests/assert_tmst_tolerance.py。
 * ⚠️ 本窗口按 8 kHz 标定；改 ODR 必须同步重算。 */
#define VIB_TMST_RAW_LO        115u    /* 8 kHz 正常 raw Δ 下界（现选用的保守诊断门限） */
#define VIB_TMST_RAW_HI        120u    /* 8 kHz 正常 raw Δ 上界（现选用的保守诊断门限） */
#define VIB_TMST_TOL_US        1u      /* 换算后物理窗口 = 名义值 ±1 µs */

/* ---- 时间戳时钟源 / 分辨率（写入文件头，见 vib_file_header_t） ---- */
#define VIB_TMST_SOURCE_INTERNAL 0u    /* 内部时钟（无 CLKIN） */
#define VIB_TMST_SOURCE_CLKIN    1u    /* 外部 CLKIN —— 本实现不支持，应拒绝 */
#define VIB_TMST_RES_1US         0u    /* TMST_CONFIG RES=0：名义 1 µs */
#define VIB_TMST_RES_16US        1u    /* TMST_CONFIG RES=1：名义 16 µs —— 本实现不支持 */

/* ---- 块头 flags（文档 §3） ---- */
#define VIB_BF_FIFO_FULL       0x0001  /* 本次读之前 FIFO 满过（INT_STATUS.FIFO_FULL） */
#define VIB_BF_SHORT_READ      0x0002  /* 实际读到记录数少于水位 */
#define VIB_BF_WRITE_RETRY     0x0004  /* 写盘重试过 */
#define VIB_BF_SPI_ERR         0x0008  /* 本块 status/lost 读取失败，FIFO 健康未知（不得当 0 健康） */

/* ---- 记录 rec_flags（文档 §3） ---- */
#define VIB_RF_INVALID         0x01
#define VIB_RF_ODR_ACCEL_CHG   0x02
#define VIB_RF_ODR_GYRO_CHG    0x04
#define VIB_RF_HEADER_MSG      0x08

/* ---------------------------------------------------------------------------
 * 文件头 256 B（文档 §2）
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    char     magic[4];                 /* 0   "VIB2" */
    uint16_t format_version;           /* 4   3 */
    uint16_t header_size;              /* 6   256 */
    uint32_t flags;                    /* 8   bit0=含削顶；bit1=20bit（v2 必须 0） */
    uint32_t driver_version;           /* 12  major<<16|minor<<8|patch */
    uint32_t reg_config_hash;          /* 16  寄存器写入序列 FNV-1a */
    uint32_t odr_hz;                   /* 20  标称 ODR，如 8000 */
    uint16_t fs_accel_g;               /* 24  16 */
    uint16_t fs_gyro_dps;              /* 26  2000 */
    uint16_t record_bytes;             /* 28  16 */
    uint16_t reserved0;                /* 30 */
    uint64_t t_start_monotonic_ns;     /* 32  CLOCK_MONOTONIC（主机读时刻，非采样时刻） */
    uint64_t t_start_utc_ns;           /* 40  UTC（仅跨设备粗对齐） */
    uint8_t  who_am_i;                 /* 48  期望 0x47 */
    uint8_t  frame;                    /* 49  0=SENSOR, 1=FRD */
    uint8_t  reserved1[6];             /* 50 */
    uint64_t sample_count_expected;    /* 56  估计值，0=未知 */
    uint16_t tmst_scale_num;           /* 64  ticks→µs 分子（32） */
    uint16_t tmst_scale_den;           /* 66  ticks→µs 分母（30） */
    uint8_t  tmst_source;              /* 68  VIB_TMST_SOURCE_*（0=内部时钟） */
    uint8_t  tmst_res;                 /* 69  VIB_TMST_RES_*（0=1µs 名义） */
    uint8_t  count_unit;               /* 70  FIFO_COUNT 单位：0=未知 1=记录 16=字节 */
    uint8_t  count_endian;             /* 71  FIFO_COUNT 端序：0=小端 1=大端 */
    uint32_t flags2;                   /* 72  保留标志（全 0） */
    uint8_t  reserved2[176];           /* 76  全 0（到 251） */
    uint32_t crc32;                    /* 252 覆盖 [0,252) */
} vib_file_header_t;

/* ---------------------------------------------------------------------------
 * 块头 32 B（文档 §3）
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint16_t block_type;               /* 0  0x0001 数据 / 0xFFFF 结束 */
    uint16_t n_samples;                /* 2  1..255 */
    uint32_t block_index;              /* 4  从 0 单调递增（缺号=主机丢块） */
    uint64_t t_read_monotonic_ns;      /* 8  本次 SPI 批量读完成时刻 */
    uint32_t lost_pkt_total;           /* 16 器件 FIFO_LOST_PKT 累计 */
    uint16_t flags;                    /* 20 VIB_BF_* */
    uint16_t reserved;                 /* 22 */
    uint32_t clip_count;               /* 24 本块削顶记录数 */
    uint32_t crc32;                    /* 28 覆盖 [0,28) + payload */
} vib_block_header_t;

/* ---------------------------------------------------------------------------
 * 单条记录 16 B（文档 §3）
 * 对应器件 FIFO Packet 3：Header(1)+Accel(6)+Gyro(6)+Temp(1)+TimeStamp(2) = 16 B
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    int16_t  accel[3];                 /* 0  ax, ay, az（**存盘小端**；线缆上是大端，见 icm_parse_fifo） */
    int16_t  gyro[3];                  /* 6  gx, gy, gz（**存盘小端**；线缆上是大端） */
    uint8_t  temp_raw;                 /* 12 **存盘是原始 uint8 字节**（v3 布局/CRC 不变）。
                                        * 物理温度须按**有符号 8 位**二补码解释：
                                        * TEMP_DATA = (temp_raw >= 128) ? temp_raw-256 : temp_raw，
                                        * T[℃] = TEMP_DATA/2.07 + 25（手册 p.25/p.66，signed/2.07+25）。
                                        * 例：0x00→25 ℃、0xFF→24.5169 ℃、0xF9→21.6184 ℃。
                                        * 本实现不据温度值置任何 invalid 标志：手册 p.25/p.66
                                        * 未给出可依赖的 8 位温度「无效」编码，不引入未证实判定。 */
    uint16_t tmst_raw;                 /* 13 ODR 时间戳 raw ticks，16 位回绕；µs = *32/30 */
    uint8_t  rec_flags;                /* 15 VIB_RF_* */
} vib_record_t;

_Static_assert(sizeof(vib_file_header_t)  == VIB_HEADER_SIZE,        "file header 必须 256 B");
_Static_assert(sizeof(vib_block_header_t) == VIB_BLOCK_HEADER_SIZE,  "block header 必须 32 B");
_Static_assert(sizeof(vib_record_t)       == VIB_RECORD_SIZE,        "record 必须 16 B");
_Static_assert(sizeof(vib_file_header_t) % 4 == 0, "packed 结构不得有隐式填充");

/* 器件一帧 FIFO 数据（Packet 3）的原始字节布局，供解析用（手册 p.38） */
typedef struct __attribute__((packed)) {
    uint8_t header;                    /* bit7 HEADER_MSG, bit6 ACCEL, bit5 GYRO,
                                        * bit3:2 TIMESTAMP_FSYNC(10=含 ODR 时间戳),
                                        * bit1/bit0 ODR_ACCEL/GYRO 变化 */
    uint8_t accel[6];
    uint8_t gyro[6];
    uint8_t temp;
    uint8_t tmst[2];
} icm_fifo_record_raw_t;

_Static_assert(sizeof(icm_fifo_record_raw_t) == VIB_RECORD_SIZE, "FIFO Packet 3 必须 16 B");

#endif /* VIB_FORMAT_H */
