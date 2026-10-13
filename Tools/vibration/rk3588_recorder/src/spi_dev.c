/* ============================================================================
 * spi_dev.c — Linux spidev 封装实现
 * ==========================================================================*/
#define _GNU_SOURCE
#include "spi_dev.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_READ_FLAG 0x80u   /* p.52：首字节 bit7=1 表示读 */

const char *spi_dev_path(const spi_dev_t *d)
{
    return d->path;
}

int spi_open(spi_dev_t *d, const char *path, uint32_t hz, uint8_t mode, uint8_t bits)
{
    memset(d, 0, sizeof(*d));
    d->speed_hz = hz;
    d->mode     = mode;
    d->bits     = bits;
    snprintf(d->path, sizeof(d->path), "%s", path);

    d->fd = open(path, O_RDWR);
    if (d->fd < 0) {
        fprintf(stderr, "[spi] 打开 %s 失败: %s\n", path, strerror(errno));
        fprintf(stderr, "[spi] 提示：SPI 默认关闭，需先使能 overlay\n"
                        "      官方 Debian/Ubuntu: sudo orangepi-config → System → Hardware → spi4-m2-cs0-spidev\n"
                        "      或 /boot/extlinux/extlinux.conf 的 FDTOVERLAYS 加 rk3588-spi4-m2-cs0-spidev.dtbo\n");
        return -1;
    }

    uint8_t  m = mode, b = bits;
    uint32_t s = hz;
    if (ioctl(d->fd, SPI_IOC_WR_MODE, &m) < 0)          { perror("[spi] SPI_IOC_WR_MODE"); goto fail; }
    if (ioctl(d->fd, SPI_IOC_WR_BITS_PER_WORD, &b) < 0) { perror("[spi] SPI_IOC_WR_BITS_PER_WORD"); goto fail; }
    if (ioctl(d->fd, SPI_IOC_WR_MAX_SPEED_HZ, &s) < 0)  { perror("[spi] SPI_IOC_WR_MAX_SPEED_HZ"); goto fail; }

    /* 回读确认（内核可能钳制到控制器上限） */
    uint32_t got_s = 0; uint8_t got_m = 0, got_b = 0;
    (void)ioctl(d->fd, SPI_IOC_RD_MAX_SPEED_HZ, &got_s);
    (void)ioctl(d->fd, SPI_IOC_RD_MODE, &got_m);
    (void)ioctl(d->fd, SPI_IOC_RD_BITS_PER_WORD, &got_b);
    d->speed_hz = got_s; d->mode = got_m; d->bits = got_b;
    if (got_s < hz) {
        fprintf(stderr, "[spi] 警告：请求 %u Hz，内核实际 %u Hz\n", hz, got_s);
    }
    fprintf(stderr, "[spi] %s opened: speed=%u Hz mode=%u bits=%u\n",
            path, d->speed_hz, d->mode, d->bits);
    return 0;

fail:
    close(d->fd);
    d->fd = -1;
    return -1;
}

void spi_close(spi_dev_t *d)
{
    if (d->fd >= 0) {
        close(d->fd);
        d->fd = -1;
    }
}

int spi_xfer(spi_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0)
        return 0;

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = (uint32_t)len;
    tr.speed_hz      = d->speed_hz;
    tr.bits_per_word = d->bits;
    tr.cs_change     = 0;

    int r = ioctl(d->fd, SPI_IOC_MESSAGE(1), &tr);
    d->xfers++;
    d->tx_bytes += len;
    if (r < 1) {
        d->errs++;
        fprintf(stderr, "[spi] transfer(len=%zu) 失败: %s\n", len, strerror(errno));
        return -1;
    }
    return 0;
}

int spi_reg_read(spi_dev_t *d, uint8_t reg, uint8_t *val)
{
    uint8_t rx[2] = { 0, 0 };
    uint8_t tx[2] = { (uint8_t)(reg | SPI_READ_FLAG), 0x00 };
    if (spi_xfer(d, tx, rx, 2) != 0)
        return -1;
    *val = rx[1];
    return 0;
}

int spi_reg_write(spi_dev_t *d, uint8_t reg, uint8_t val)
{
    /* p.52：写只支持单字节（2 字节事务），首字节 bit7=0 */
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };
    return spi_xfer(d, tx, NULL, 2);
}

int spi_burst_read(spi_dev_t *d, uint8_t reg, uint8_t *buf, size_t len)
{
    if (len == 0 || len > SPI_SCRATCH_MAX - 1)
        return -1;

    d->scratch[0] = (uint8_t)(reg | SPI_READ_FLAG);
    memset(d->scratch + 1, 0, len);
    if (spi_xfer(d, d->scratch, d->scratch, len + 1) != 0)
        return -1;
    memcpy(buf, d->scratch + 1, len);
    return 0;
}

int spi_reg_rmw(spi_dev_t *d, uint8_t reg, uint8_t clr_mask, uint8_t set_mask,
                uint8_t *old_val, uint8_t *new_val)
{
    uint8_t v = 0;
    if (spi_reg_read(d, reg, &v) != 0)
        return -1;
    if (old_val)
        *old_val = v;
    uint8_t nv = (uint8_t)((v & (uint8_t)~clr_mask) | set_mask);
    if (new_val)
        *new_val = nv;
    if (nv == v)
        return 0;
    return spi_reg_write(d, reg, nv);
}
