/* ============================================================================
 * vib_format.h — 采集数据格式 v2 的机器可读定义
 *
 * 唯一真源是同目录文档：../数据格式规范.md
 * 改字段/布局的顺序永远是：① 改文档 ② 升 VIB_FORMAT_VERSION ③ 改本文件
 *
 * 依据：TDK DS-000347 Rev 1.3（ICM-42688-P 数据手册，归档于
 *       01_硬件/datasheet/TDK_ICM-42688-P_DS-000347.pdf）
 * ==========================================================================*/
#ifndef VIB_FORMAT_H
#define VIB_FORMAT_H

#include <stdint.h>
#include <stddef.h>

/* ---- 版本与尺寸（文档 §1、§2、§3） ---- */
#define VIB_MAGIC              "VIB2"
#define VIB_FORMAT_VERSION     2
#define VIB_HEADER_SIZE        256
#define VIB_BLOCK_HEADER_SIZE  32
#define VIB_RECORD_SIZE        16
#define VIB_SAMPLES_PER_BLOCK  32      /* 512 B payload == FIFO 水位，8 kHz 下约 4 ms */
#define VIB_DRIVER_VERSION     0x000100 /* major.minor.patch = 0.1.0 */

#define VIB_BLOCK_TYPE_DATA    0x0001
#define VIB_BLOCK_TYPE_END     0xFFFF

/* ---- 判定阈值（文档 §5） ---- */
#define VIB_CLIP_THRESHOLD     32760   /* |raw| >= 32760 即判削顶（满量程 -8 LSB） */
#define VIB_INVALID_SENTINEL   (-32766) /* 器件哨兵：-32768/-32767 被替换值，见手册 p.73 */

/* ---- ODR 时间戳（手册 p.80 TMST_CONFIG：RES=0 → 1 µs；p.38 记录内 2 B） ---- */
#define VIB_TMST_WRAP          65536u  /* 16 位回绕周期，µs */
#define VIB_TMST_HALF          32768   /* 回绕展开判定用半周期 */
#define VIB_TMST_NOMINAL_US    125u    /* 8 kHz 下相邻记录名义 Δtmst（校正前） */
#define VIB_TMST_TOL_US        1u      /* 判定窗口 = 名义值 ±1 µs */

/* ---- 块头 flags（文档 §3） ---- */
#define VIB_BF_FIFO_FULL       0x0001  /* 本次读之前 FIFO 满过（INT_STATUS.FIFO_FULL） */
#define VIB_BF_SHORT_READ      0x0002  /* 实际读到记录数少于水位 */
#define VIB_BF_WRITE_RETRY     0x0004  /* 写盘重试过 */

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
    uint16_t format_version;           /* 4   2 */
    uint16_t header_size;              /* 6   256 */
    uint32_t flags;                    /* 8   bit0=含削顶；bit1=20bit（v2 必须 0） */
    uint32_t driver_version;           /* 12  major<<16|minor<<8|patch */
    uint32_t reg_config_hash;          /* 16  寄存器写入序列 FNV-1a */
    uint32_t odr_hz;                   /* 20  标称 ODR，如 8000 */
    uint16_t fs_accel_g;               /* 24  16 */
    uint16_t fs_gyro_dps;              /* 26  2000 */
    uint16_t record_bytes;             /* 28  16 */
    uint16_t reserved0;                /* 30 */
    uint64_t t_start_monotonic_ns;     /* 32  CLOCK_MONOTONIC */
    uint64_t t_start_utc_ns;           /* 40  UTC */
    uint8_t  who_am_i;                 /* 48  期望 0x47 */
    uint8_t  frame;                    /* 49  0=SENSOR, 1=FRD */
    uint8_t  reserved1[6];             /* 50 */
    uint64_t sample_count_expected;    /* 56  估计值，0=未知 */
    uint8_t  reserved2[188];           /* 64  全 0（到 251） */
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
    int16_t  accel[3];                 /* 0  ax, ay, az（小端） */
    int16_t  gyro[3];                  /* 6  gx, gy, gz（小端） */
    uint8_t  temp_raw;                 /* 12 T[℃] = temp_raw/2.07 + 25（p.24） */
    uint16_t tmst_us;                  /* 13 ODR 时间戳，µs，16 位回绕 */
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
