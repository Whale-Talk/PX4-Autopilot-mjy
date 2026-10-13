/* ============================================================================
 * spi_dev.h — Linux spidev 封装（香橙派 5 Pro，SPI4 → /dev/spidev4.0）
 *
 * 依据：DS-000347 p.52「SPI INTERFACE」
 *   - MSB first、SCLK 上升沿锁存、下降沿翻转 → SPI Mode 0 或 3（p.52；DEVICE_CONFIG
 *     bit4 SPI_MODE=0 即 Mode0/3，为复位默认，见 p.63）
 *   - SCLK 上限 24 MHz
 *   - 读：首字节 = 地址 | 0x80，其后为数据；支持单读与**突发读**
 *   - 写：**仅支持单字节写**（2 字节事务），没有突发写
 *   - 4 线 SPI 为复位默认（INTF_CONFIG4 复位 0x83，bit1 SPI_AP_4WIRE=1，p.92）→ 无需改动
 * ==========================================================================*/
#ifndef VIB_SPI_DEV_H
#define VIB_SPI_DEV_H

#include <stdint.h>
#include <stddef.h>

/* 一次突发读的上限：FIFO 2 kB + 1 字节地址 */
#define SPI_SCRATCH_MAX 2049

typedef struct spi_dev {
    int      fd;
    uint32_t speed_hz;
    uint8_t  mode;      /* 0 或 3 */
    uint8_t  bits;      /* 8 */
    /* 统计（用于运行日志与自检报告） */
    uint64_t xfers;
    uint64_t tx_bytes;
    uint64_t errs;
    /* 突发读内部暂存，避免热路径 malloc */
    uint8_t  scratch[SPI_SCRATCH_MAX];
    char     path[64];
    /* 传输注入点：生产代码为 NULL → 走真实 spidev ioctl；单元测试注入 mock。
     * 协议语义与真实 spidev 一致：tx[0] bit7=1 读、否则写，突发读 tx=[addr|0x80, 0...]。
     * 这让 icm42688p 的自检/启动逻辑能在无硬件（无 spidev）下被测试。 */
    int    (*xfer_fn)(struct spi_dev *d, const uint8_t *tx, uint8_t *rx, size_t len);
} spi_dev_t;

/* 打开并配置 spidev。返回 0 成功，-1 失败（errno 已设置，同时打印原因）。 */
int  spi_open(spi_dev_t *d, const char *path, uint32_t hz, uint8_t mode, uint8_t bits);
void spi_close(spi_dev_t *d);

/* 裸收发（len 字节，tx/rx 均可为 NULL 表示单向） */
int spi_xfer(spi_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len);

/* 单寄存器：读 1 字节 / 写 1 字节（写=2 字节事务，符合 p.52） */
int spi_reg_read(spi_dev_t *d, uint8_t reg, uint8_t *val);
int spi_reg_write(spi_dev_t *d, uint8_t reg, uint8_t val);

/* 从 reg 起突发读 len 字节（一次 CS 有效期内完成，天然一致） */
int spi_burst_read(spi_dev_t *d, uint8_t reg, uint8_t *buf, size_t len);

/* 读-改-写单个寄存器（用于只改指定位、不动其余位） */
int spi_reg_rmw(spi_dev_t *d, uint8_t reg, uint8_t clr_mask, uint8_t set_mask,
                uint8_t *old_val, uint8_t *new_val);

const char *spi_dev_path(const spi_dev_t *d);

#endif /* VIB_SPI_DEV_H */
