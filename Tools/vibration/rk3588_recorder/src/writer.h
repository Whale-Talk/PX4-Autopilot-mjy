/* ============================================================================
 * writer.h — 落盘线程与无锁环形缓冲（生产者=采集线程，消费者=写盘线程）
 *
 * 目的：采集线程绝不能因写盘（尤其是 fsync）而错过 8 kHz / 4 ms 的 FIFO 排空窗口。
 * 因此采集线程只做「序列化一个块 → 投递」，落盘与 fsync 全在另一线程。
 *
 * 环满时的策略：**不阻塞、不覆盖**，直接返回 -1。调用方的 block_index 仍会前进，
 * 于是解析器会看到 block_index 缺号 —— 这正是格式规范 §6 定义的「主机侧丢块」证据。
 * ==========================================================================*/
#ifndef VIB_WRITER_H
#define VIB_WRITER_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <stdatomic.h>

#include "vib_format.h"

/* 2048 槽 × 544 B ≈ 1.11 MB ≈ 8 s@8kHz 的缓冲，足以吸收 SSD/NVMe 抖动 */
#define WRITER_RING_SLOTS  2048
#define WRITER_SLOT_BYTES  (VIB_BLOCK_HEADER_SIZE + VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE)

typedef struct {
    uint32_t len;
    uint8_t  bytes[WRITER_SLOT_BYTES];
} writer_slot_t;

typedef struct {
    char              path[512];
    uint32_t          fsync_ms;

    vib_file_header_t fh;

    writer_slot_t    *ring;
    _Atomic uint64_t  head;        /* 生产者发布位置 */
    _Atomic uint64_t  tail;        /* 消费者处理位置 */

    uint64_t          submitted;
    uint64_t          dropped;     /* 环满未提交的块数（⇒ 解析器会看到缺号） */
    uint64_t          blocks_written;
    uint64_t          bytes_written;
    uint64_t          samples_written;
    uint64_t          clip_total;
    uint32_t          last_lost;
    uint64_t          fsync_count;

    int               fd;
    pthread_t         th;
    _Atomic int       stop;
    _Atomic int       error;       /* 首次写盘失败的 errno；非 0 时不再消费 */
} writer_t;

/* 打开文件、写入文件头（含 CRC）、预分配（prealloc_bytes=0 表示不预分配）、起写盘线程 */
int  writer_start(writer_t *w, const char *path, const vib_file_header_t *fh,
                  uint64_t prealloc_bytes, uint32_t fsync_ms);

/* 投递一个数据块。bh->crc32 由本函数填充（调用方无需计算）。返回 0 成功，-1 环满。 */
int  writer_submit(writer_t *w, const vib_block_header_t *bh, const void *payload, size_t len);

/* 停止线程、写结束块（block_type=0xFFFF）、fsync、关闭。 */
void writer_finish(writer_t *w, uint32_t final_lost_total);

const char *writer_path(const writer_t *w);
int         writer_error(const writer_t *w);

#endif /* VIB_WRITER_H */
