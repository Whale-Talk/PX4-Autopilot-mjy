/* ============================================================================
 * writer.c — 落盘线程实现
 *
 * 设计要点：
 *   1) 采集线程只做序列化 + 投递，绝不 fsync（避免毫秒级停顿切断 FIFO 排空）；
 *   2) 环满是「丢块」而非覆盖：block_index 会留下缺号，成为解析器可检出的
 *      主机侧丢块证据（数据格式规范.md §6）；
 *   3) 所有多字节字段一律用 memcpy 读写，不对齐访问不依赖编译器行为。
 * ==========================================================================*/
#define _GNU_SOURCE
#include "writer.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

/* CRC 覆盖 header[0:28) + payload （文档 §3）；跳过 header 里的 crc32 字段本身 */
static uint32_t slot_crc(const writer_slot_t *s)
{
    uint32_t c = vib_crc32_begin();
    c = vib_crc32_update(c, s->bytes, offsetof(vib_block_header_t, crc32));
    if (s->len > VIB_BLOCK_HEADER_SIZE)
        c = vib_crc32_update(c, s->bytes + VIB_BLOCK_HEADER_SIZE,
                             s->len - VIB_BLOCK_HEADER_SIZE);
    return vib_crc32_end(c);
}

static void slot_pack(writer_slot_t *s, const vib_block_header_t *bh,
                      const void *payload, size_t len)
{
    s->len = (uint32_t)(VIB_BLOCK_HEADER_SIZE + len);
    memcpy(s->bytes, bh, VIB_BLOCK_HEADER_SIZE);
    if (len && payload)
        memcpy(s->bytes + VIB_BLOCK_HEADER_SIZE, payload, len);

    uint32_t crc = slot_crc(s);
    memcpy(s->bytes + offsetof(vib_block_header_t, crc32), &crc, sizeof(crc));
}

static void *writer_thread(void *arg)
{
    writer_t *w = (writer_t *)arg;
    uint64_t last_fsync = mono_ns();

    for (;;) {
        uint64_t head = atomic_load_explicit(&w->head, memory_order_acquire);
        uint64_t tail = atomic_load_explicit(&w->tail, memory_order_relaxed);

        if (tail == head) {
            if (atomic_load_explicit(&w->stop, memory_order_acquire))
                break;
            sleep_us(500);
            continue;
        }

        if (atomic_load_explicit(&w->error, memory_order_relaxed) != 0) {
            sleep_us(2000);   /* 已出错：停止消费，让上层看到 dropped 增长 */
            continue;
        }

        const writer_slot_t *s = &w->ring[tail % WRITER_RING_SLOTS];
        ssize_t n = write(w->fd, s->bytes, s->len);
        if (n != (ssize_t)s->len) {
            int e = errno;
            atomic_store_explicit(&w->error, e, memory_order_relaxed);
            vib_block_header_t bh;
            memcpy(&bh, s->bytes, VIB_BLOCK_HEADER_SIZE);
            fprintf(stderr, "[writer] 写盘失败: %s（%zd/%u B，block_index=%u）\n",
                    strerror(e), n, s->len, bh.block_index);
            continue;
        }

        vib_block_header_t bh;
        memcpy(&bh, s->bytes, VIB_BLOCK_HEADER_SIZE);
        w->blocks_written++;
        w->bytes_written += s->len;
        w->samples_written += bh.n_samples;
        w->clip_total      += bh.clip_count;
        w->last_lost        = bh.lost_pkt_total;

        atomic_store_explicit(&w->tail, tail + 1, memory_order_release);

        if (w->fsync_ms &&
            (mono_ns() - last_fsync) >= (uint64_t)w->fsync_ms * 1000000ull) {
            if (fdatasync(w->fd) != 0 && errno != EINVAL)
                fprintf(stderr, "[writer] fdatasync 失败: %s\n", strerror(errno));
            last_fsync = mono_ns();
            w->fsync_count++;
        }
    }
    return NULL;
}

int writer_start(writer_t *w, const char *path, const vib_file_header_t *fh,
                 uint64_t prealloc_bytes, uint32_t fsync_ms)
{
    memset(w, 0, sizeof(*w));
    snprintf(w->path, sizeof(w->path), "%s", path);
    w->fsync_ms = fsync_ms;
    w->fd = -1;

    w->ring = (writer_slot_t *)calloc(WRITER_RING_SLOTS, sizeof(writer_slot_t));
    if (!w->ring) {
        fprintf(stderr, "[writer] 环形缓冲分配失败（%zu B）\n",
                (size_t)WRITER_RING_SLOTS * sizeof(writer_slot_t));
        return -1;
    }

    w->fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (w->fd < 0) {
        fprintf(stderr, "[writer] 创建 %s 失败: %s\n", path, strerror(errno));
        free(w->ring);
        w->ring = NULL;
        return -1;
    }

    if (prealloc_bytes > VIB_HEADER_SIZE) {
        int rc = posix_fallocate(w->fd, 0, (off_t)prealloc_bytes);
        if (rc != 0)
            fprintf(stderr, "[writer] 预分配 %llu B 失败: %s（继续，但可能中途 ENOSPC）\n",
                    (unsigned long long)prealloc_bytes, strerror(rc));
    }

    w->fh = *fh;
    memcpy(w->fh.magic, VIB_MAGIC, 4);
    w->fh.format_version = VIB_FORMAT_VERSION;
    w->fh.header_size    = VIB_HEADER_SIZE;
    w->fh.record_bytes   = VIB_RECORD_SIZE;
    w->fh.driver_version = VIB_DRIVER_VERSION;
    w->fh.crc32          = 0;
    w->fh.crc32          = vib_crc32(&w->fh, offsetof(vib_file_header_t, crc32));

    /* 注意：不能直接 write(&w->fh, 256)。GCC 看到 packed 结构的首成员 magic[4]
     * 会误判为「读 256 字节越界」（-Wstringop-overread 假阳性），
     * 因此先 memcpy 到显式字节缓冲再写。 */
    uint8_t hdrbuf[VIB_HEADER_SIZE];
    memcpy(hdrbuf, &w->fh, VIB_HEADER_SIZE);
    ssize_t n = write(w->fd, hdrbuf, VIB_HEADER_SIZE);
    if (n != VIB_HEADER_SIZE) {
        fprintf(stderr, "[writer] 写文件头失败: %s\n", strerror(errno));
        close(w->fd);
        w->fd = -1;
        free(w->ring);
        w->ring = NULL;
        return -1;
    }
    w->bytes_written = VIB_HEADER_SIZE;

    if (pthread_create(&w->th, NULL, writer_thread, w) != 0) {
        fprintf(stderr, "[writer] 创建写盘线程失败: %s\n", strerror(errno));
        close(w->fd);
        w->fd = -1;
        free(w->ring);
        w->ring = NULL;
        return -1;
    }
    return 0;
}

int writer_submit(writer_t *w, const vib_block_header_t *bh, const void *payload, size_t len)
{
    if (!w->ring)
        return -1;
    if (len > VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE) {
        fprintf(stderr, "[writer] payload 过大: %zu B\n", len);
        return -1;
    }

    uint64_t head = atomic_load_explicit(&w->head, memory_order_relaxed);
    uint64_t tail = atomic_load_explicit(&w->tail, memory_order_acquire);
    if (head - tail >= WRITER_RING_SLOTS) {
        w->dropped++;   /* ⇒ 解析器会看到 block_index 缺号（格式规范 §6） */
        return -1;
    }

    writer_slot_t *s = &w->ring[head % WRITER_RING_SLOTS];
    slot_pack(s, bh, payload, len);

    atomic_store_explicit(&w->head, head + 1, memory_order_release);
    w->submitted++;
    return 0;
}

void writer_finish(writer_t *w, uint32_t final_lost_total)
{
    if (!w->ring)
        return;

    atomic_store_explicit(&w->stop, 1, memory_order_release);
    pthread_join(w->th, NULL);

    if (atomic_load_explicit(&w->error, memory_order_relaxed) == 0) {
        vib_block_header_t bh;
        memset(&bh, 0, sizeof(bh));
        bh.block_type          = VIB_BLOCK_TYPE_END;
        bh.n_samples           = 0;
        bh.block_index         = (uint32_t)(w->submitted & 0xFFFFFFFFu);
        bh.t_read_monotonic_ns = mono_ns();
        bh.lost_pkt_total      = final_lost_total;
        bh.clip_count          = (uint32_t)(w->clip_total & 0xFFFFFFFFu);
        bh.crc32               = vib_crc32(&bh, offsetof(vib_block_header_t, crc32));

        uint8_t endbuf[VIB_BLOCK_HEADER_SIZE];
        memcpy(endbuf, &bh, VIB_BLOCK_HEADER_SIZE);
        ssize_t n = write(w->fd, endbuf, VIB_BLOCK_HEADER_SIZE);
        if (n != VIB_BLOCK_HEADER_SIZE)
            fprintf(stderr, "[writer] 写结束块失败: %s\n", strerror(errno));
        else
            w->bytes_written += VIB_BLOCK_HEADER_SIZE;
    } else {
        fprintf(stderr, "[writer] 存在写盘错误（errno=%d），未写结束块——"
                        "该文件将被解析器标记为「异常掉电/写入失败」\n",
                atomic_load_explicit(&w->error, memory_order_relaxed));
    }

    if (w->fd >= 0) {
        if (fdatasync(w->fd) != 0 && errno != EINVAL)
            fprintf(stderr, "[writer] 收尾 fdatasync 失败: %s\n", strerror(errno));
        if (close(w->fd) != 0)
            fprintf(stderr, "[writer] close 失败: %s\n", strerror(errno));
        w->fd = -1;
    }

    free(w->ring);
    w->ring = NULL;
}

const char *writer_path(const writer_t *w)
{
    return w->path;
}

int writer_error(const writer_t *w)
{
    return atomic_load_explicit(&w->error, memory_order_relaxed);
}
