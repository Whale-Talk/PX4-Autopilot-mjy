/* ============================================================================
 * icm42688p.h — ICM-42688-P 寄存器与配置 API
 *
 * 事实来源：TDK DS-000347 Rev 1.3
 *   归档：../../01_硬件/datasheet/TDK_ICM-42688-P_DS-000347.pdf
 *   文本：../../01_硬件/datasheet/text/ICM-42688-P_DS-000347.txt
 *   配置口径：../寄存器配置.md（每个数值都带页码）
 *
 * 纪律：本文件里出现的每个寄存器地址/位域都必须能在手册里找到页码；找不到的
 *       一律不写死，改用「读-改-写 + 自检回读上报」的方式处理。
 * ==========================================================================*/
#ifndef VIB_ICM42688P_H
#define VIB_ICM42688P_H

#include <stdint.h>
#include <stddef.h>
#include "spi_dev.h"
#include "vib_format.h"   /* vib_record_t：本头文件对外暴露解析接口 */

/* ---- BANK 0 寄存器（地址见手册 p.60–61 总表，细节页码逐条标注） ---- */
#define ICM_REG_DEVICE_CONFIG      0x11  /* p.63 复位 0x00：bit4 SPI_MODE(0=Mode0/3) bit0 SOFT_RESET */
#define ICM_REG_DRIVE_CONFIG       0x13  /* p.63 复位 0x05：SPI_SLEW_RATE[2:0] */
#define ICM_REG_INT_CONFIG         0x14  /* p.63 INT1/INT2 极性驱动 */
#define ICM_REG_FIFO_CONFIG        0x16  /* p.64 复位 0x00：FIFO_MODE[7:6] 00=Bypass 01=Stream 10/11=STOP-on-FULL */
#define ICM_REG_INT_STATUS         0x2D  /* p.60 R/C：bit3 DATA_RDY bit2 FIFO_THS bit1 FIFO_FULL */
#define ICM_REG_FIFO_COUNTH        0x2E  /* p.60 */
#define ICM_REG_FIFO_COUNTL        0x2F  /* p.60 */
#define ICM_REG_FIFO_DATA          0x30  /* p.60 */
#define ICM_REG_SIGNAL_PATH_RESET  0x4B  /* p.61：bit3 ABORT_AND_RESET, bit2 TMST_STROBE, bit1 FIFO_FLUSH */
#define ICM_REG_INTF_CONFIG0       0x4C  /* p.73 复位 0x30：bit7 HOLD_LAST bit6 COUNT_REC bit5 COUNT_ENDIAN bit4 DATA_ENDIAN */
#define ICM_INTF0_HOLD_LAST        0x80  /* bit7 无效样本静默复制上一条（本实现清 0） */
#define ICM_INTF0_COUNT_REC        0x40  /* bit6 1=FIFO_COUNT 按记录计数 */
#define ICM_INTF0_COUNT_ENDIAN     0x20  /* bit5 1=FIFO_COUNT 大端（先 COUNTH 后 COUNTL） */
#define ICM_INTF0_DATA_ENDIAN      0x10  /* bit4 1=传感器数据大端（本实现用 0x70 → 置 1，消歧义） */

/* SIGNAL_PATH_RESET（0x4B）位域：与官方 PX4 InvenSense_ICM42688P_registers.hpp
 * SIGNAL_PATH_RESET_BIT 一致（ABORT_AND_RESET=Bit3, FIFO_FLUSH=Bit1）。
 * 0x04 是 TMST_STROBE（Bit2），**不是** FIFO_FLUSH。 */
#define ICM_SPR_FIFO_FLUSH         0x02  /* bit1：清 FIFO */
#define ICM_SPR_TMST_STROBE        0x04  /* bit2：时间戳 strobe（不清 FIFO） */
#define ICM_SPR_ABORT_AND_RESET    0x08  /* bit3：中止并复位 */
#define ICM_REG_INTF_CONFIG1       0x4D  /* p.74 复位 0x91：bit2 RTC_MODE bit1:0 CLKSEL */
#define ICM_REG_PWR_MGMT0          0x4E  /* p.75 复位 0x00：bit5 TEMP_DIS bit4 IDLE bit3:2 GYRO_MODE bit1:0 ACCEL_MODE */
#define ICM_REG_GYRO_CONFIG0       0x4F  /* p.76 复位 0x06：[7:5] FS [3:0] ODR */
#define ICM_REG_ACCEL_CONFIG0      0x50  /* p.77 复位 0x06：[7:5] FS [3:0] ODR */
#define ICM_REG_GYRO_CONFIG1       0x51  /* p.78 复位 0x16：[7:5] TEMP_FILT_BW [3:2] UI_FILT_ORD [1:0] DEC2_M2_ORD */
#define ICM_REG_GYRO_ACCEL_CONFIG0 0x52  /* p.79 复位 0x11：[7:4] ACCEL_UI_FILT_BW [3:0] GYRO_UI_FILT_BW */
#define ICM_REG_ACCEL_CONFIG1      0x53  /* p.80 复位 0x0D：[4:3] UI_FILT_ORD [2:1] DEC2_M2_ORD */
#define ICM_REG_TMST_CONFIG        0x54  /* p.80 复位 0x23：bit4 TO_REGS bit3 RES(0=1µs) bit2 DELTA bit1 FSYNC bit0 EN */
#define ICM_REG_FIFO_CONFIG1       0x5F  /* p.82 复位 0x00：bit5 WM_GT_TH bit4 HIRES bit3 TMST_FSYNC bit2 TEMP bit1 GYRO bit0 ACCEL */
#define ICM_REG_FIFO_CONFIG2       0x60  /* p.82 FIFO_WM[7:0] */
#define ICM_REG_FIFO_CONFIG3       0x61  /* p.82 FIFO_WM[11:8] */
#define ICM_REG_INT_CONFIG0        0x63  /* p.83 复位 0x00：bit5:4 UI_DRDY_CLR bit3:2 FIFO_THS_CLR bit1:0 FIFO_FULL_CLR */
#define ICM_REG_INT_CONFIG1        0x64  /* p.84 复位 0x10：bit6 TPULSE bit5 TDEASSERT bit4 ASYNC_RESET */
#define ICM_REG_INT_SOURCE0        0x65  /* p.84 复位 0x10：bit2 FIFO_THS_INT1_EN bit1 FIFO_FULL_INT1_EN 等 */
#define ICM_REG_FIFO_LOST_PKT0     0x6C  /* p.61 丢失记录计数低字节 */
#define ICM_REG_FIFO_LOST_PKT1     0x6D  /* p.61 高字节 */
#define ICM_REG_WHO_AM_I           0x75  /* p.87 期望 0x47 */
#define ICM_REG_REG_BANK_SEL       0x76  /* p.61 BANK_SEL[2:0]（对全部 bank 可见） */

/* ---- BANK 1：陀螺静态滤波（手册 p.27 表） ---- */
#define ICM_B1_GYRO_CONFIG_STATIC2  0x0B /* GYRO_AAF_DIS / GYRO_NF_DIS */
#define ICM_B1_GYRO_AAF_DELT        0x0C /* GYRO_AAF_DELT[5:0] */
#define ICM_B1_GYRO_AAF_DELTSQR     0x0D /* GYRO_AAF_DELTSQR[7:0] */
#define ICM_B1_GYRO_AAF_BITSHIFT    0x0E /* [3:0] DELTSQR[11:8] [7:4] BITSHIFT */
#define ICM_B1_INTF_CONFIG4         0x7A /* p.92 复位 0x83：bit1 SPI_AP_4WIRE=1（4 线默认，勿动） */

/* ---- BANK 2：加速度静态滤波（手册 p.29 表） ---- */
#define ICM_B2_ACCEL_CONFIG_STATIC2 0x03 /* [6:1] ACCEL_AAF_DELT  bit0 ACCEL_AAF_DIS */
#define ICM_B2_ACCEL_AAF_DELTSQR    0x04 /* [7:0] DELTSQR */
#define ICM_B2_ACCEL_AAF_BITSHIFT   0x05 /* [3:0] DELTSQR[11:8] [7:4] BITSHIFT */

#define ICM_BANK0 0x00
#define ICM_BANK1 0x01
#define ICM_BANK2 0x02

#define ICM_WHO_AM_I_VALUE 0x47

/* ---- FIFO 包头位（手册 p.38，Packet 3 的 header 字节） ---- */
#define ICM_FIFO_HDR_MSG        0x80  /* 1 = FIFO 空标记（不是数据） */
#define ICM_FIFO_HDR_ACCEL      0x40
#define ICM_FIFO_HDR_GYRO       0x20
#define ICM_FIFO_HDR_20BIT      0x10
#define ICM_FIFO_HDR_TS_MASK    0x0C  /* bit3:2 HEADER_TIMESTAMP_FSYNC */
#define ICM_FIFO_HDR_TS_ODR     0x08  /* 10 = 含 ODR 时间戳 */
#define ICM_FIFO_HDR_ODR_ACCEL  0x02  /* 加速度 ODR 相对上一包变化 */
#define ICM_FIFO_HDR_ODR_GYRO   0x01

/* ---- INT_STATUS（0x2D，p.68，R/C 读清，复位 0x10） ---- */
#define ICM_STATUS_UI_FSYNC     0x40
#define ICM_STATUS_PLL_RDY      0x20
#define ICM_STATUS_RESET_DONE   0x10
#define ICM_STATUS_DATA_RDY     0x08
#define ICM_STATUS_FIFO_THS     0x04
#define ICM_STATUS_FIFO_FULL    0x02
#define ICM_STATUS_AGC_RDY      0x01

/* ---- 目标配置（见 ../寄存器配置.md §2） ---- */
typedef struct {
    uint32_t odr_hz;            /* 8000 */
    uint16_t wm_bytes;          /* FIFO 水位，512 B ≈ 32 条 16 B 记录 */
    uint8_t  fs_accel_g;        /* 16 */
    uint16_t fs_gyro_dps;       /* 2000 */
    /* AAF（2 阶，陀螺/加速度独立）：手册 p.27/p.29 表格
     * 3 dB BW ≈ 2097 Hz → DELT=39, DELTSQR=1536, BITSHIFT=4 */
    uint8_t  aaf_enable;        /* 1=启用 AAF（清 AAF_DIS），0=旁路 */
    uint8_t  aaf_delt;          /* 39 */
    uint16_t aaf_delt_sq;       /* 1536 */
    uint8_t  aaf_bit_shift;     /* 4 */
    /* UI 滤波带宽档：GYRO_ACCEL_CONFIG0 复位 0x11 → 陀螺[3:0]=1, 加速度[7:4]=1
     * 手册 p.79：档 1 → max(400, ODR)/4 = 2000 Hz @8kHz（名义值） */
    uint8_t  ui_filt_bw_nibble; /* 1 */
    /* FIFO 记录时间戳：TMST_CONFIG 期望值
     * 0x33 = 保留位5=1 | TO_REGS=1 | RES=0(1µs) | DELTA=0 | FSYNC=1 | EN=1（p.80）
     * RES=0 + 内部时钟（无 CLKIN）：Rev 1.6 §12.7 规定物理 µs = raw_ticks * 32/30。 */
    uint8_t  tmst_config;
    /* INT_CONFIG1：ODR ≥ 4kHz 时手册 p.84 明确要求
     * bit6 TPULSE=1(8µs) 且 bit5 TDEASSERT_DISABLE=1，并把 bit4 ASYNC_RESET 由默认 1 改为 0
     * → 0x60 */
    uint8_t  int_config1;
    /* INT_CONFIG0：FIFO_THS/FIFO_FULL 的清除方式，10=读 FIFO 数据时清除（p.83） */
    uint8_t  int_config0;
    /* INT_SOURCE0：bit2 使能 FIFO_THS→INT1（p.84） */
    uint8_t  int_source0;
    /* INTF_CONFIG0（p.73）：0x70 = COUNT_REC=1(按记录) + COUNT_ENDIAN=1(大端) +
     * DATA_ENDIAN=1(传感器数据大端)，HOLD_LAST=0。轴上/时间戳统一按大端读，消除
     * 「SENSOR_DATA_ENDIAN=0 时时间戳字节序」的歧义；存盘仍保持小端布局。
     * 写入后回读校验，不靠猜。 */
    uint8_t  intf_config0;
} icm_cfg_t;

/* 自检里「等一小段再看计数」的窗口：4 ms@8kHz ≈ 32 条，远小于 16 ms 满窗，
 * 不会像旧版 20 ms 等待那样主动把 FIFO 灌满。 */
#define ICM_SELFTEST_WINDOW_US 4000u

typedef struct {
    spi_dev_t *spi;
    icm_cfg_t  cfg;
    uint8_t    who_am_i;
    uint32_t   reg_hash;       /* 写入序列的 FNV-1a，落进文件头 */
    int        count_div;      /* FIFO_COUNT 换算成记录数的除数：1（记录）或 16（字节），回读判定 */
    uint8_t    count_endian;   /* FIFO_COUNT 端序（回读 INTF_CONFIG0 bit5）：1=大端 */
    uint32_t   lost_total;     /* 最近一次读到的 FIFO_LOST_PKT 累计 */
    uint64_t   records_total;
    uint32_t   fifo_full_events;
    uint32_t   bad_header_records; /* FIFO 包头与标称 Packet 3 不符的记录数（应恒为 0） */
} icm_t;

void icm_default_cfg(icm_cfg_t *c);

/* 寄存器写入序列的哈希（同一 cfg 必得同一值；用于数据文件头的可复现标记） */
uint32_t icm_reg_config_hash(const icm_cfg_t *c);

int icm_soft_reset(icm_t *d);
int icm_read_whoami(icm_t *d, uint8_t *id);
int icm_apply_config(icm_t *d);
int icm_verify_config(icm_t *d, char *report, size_t n);

/* 自检：软复位 → ID → 配置 → 回读比对（含 INTF_CONFIG0 的 COUNT_REC/COUNT_ENDIAN）
 * → 清 FIFO → 4 ms sub-full 窗口采样 → 校验 FIFO_COUNT 单位 + 丢包 Δ==0 + Δtmst(raw 117/118)。
 * 返回 0 全部通过；report 里写人类可读结论（会被写进运行日志）。失败应中止采集。 */
int icm_selftest(icm_t *d, char *report, size_t n);

int icm_fifo_flush(icm_t *d);
int icm_fifo_count_raw(icm_t *d, uint16_t *raw);
int icm_fifo_count_records(icm_t *d, int *recs);
int icm_read_lost(icm_t *d, uint32_t *total);
int icm_read_status(icm_t *d, uint8_t *st);
int icm_read_fifo(icm_t *d, uint8_t *buf, size_t nbytes);

/* 解析 FIFO 原始字节（Packet 3，16 B/条，手册 p.37–38）为记录数组。
 * 返回成功解析的记录数；*bad_headers 输出「包头与标称 Packet 3 不符」的条数（应恒为 0）。
 * 字节序：线缆上数据按 INTF_CONFIG0 的 SENSOR_DATA_ENDIAN=1（**大端**）解释，
 * 解析后写入主机本机（小端）的 vib_record_t —— 存盘仍保持小端布局不变。 */
size_t icm_parse_fifo(const uint8_t *raw, size_t nbytes, vib_record_t *out,
                      size_t out_cap, uint32_t *bad_headers);

#endif /* VIB_ICM42688P_H */
