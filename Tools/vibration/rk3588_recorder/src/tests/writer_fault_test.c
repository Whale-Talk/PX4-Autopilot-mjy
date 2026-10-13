/* ============================================================================
 * tests/writer_fault_test.c — 生产 writer.c 的 I/O 故障注入回归
 *
 * 目的（对应独立复核的反例）：
 *   1) writer_finish 的「结束块写失败/短写」、收尾 fdatasync、close 失败，
 *      以前只打印、writer_error 仍为 0 ⇒ vib_run 会把该批次当健康返回；
 *      现在**所有真实 I/O 失败**都必须留下非 0 的首个 error。
 *   2) 数据块短写时 errno 可能为 0 ⇒ 必须以 EIO 表示「未完整落盘」。
 *   3) 关联死锁：写盘线程在 error != 0 且 tail < head（队列还有块）时旧代码
 *      只 sleep/continue、不检查 stop ⇒ writer_finish/writer_abort 的
 *      pthread_join 永不返回。本测试**先成功投递队列再制造失败**，
 *      覆盖「有队列 + error 非 0 时 finish/abort 仍能结束」。
 *   4) EINVAL 不是「已持久化」的证据：周期/收尾 fdatasync 返回 EINVAL 同样记错误。
 *
 * 做法：链接期故障注入（-Wl,--wrap=write/fdatasync/close），只包裹生产
 * writer.c 的调用点；被测代码就是生产源码本身，不是模型重写。
 * 产物写到独立临时目录（不碰真实设备、不碰仓库）：
 *   <base>/must_reject/  结构必坏的 .bin 产物 → 解析器必须判为不 OK（由 run_roundtrip
 *                        的 Python 步骤复核，本程序不做清理）
 *   <base>/struct_ok/    结构仍完整的 .bin 产物（失败靠 writer_errno 暴露）
 *
 * 用法: writer_fault_test [BASE_DIR]   （默认 /tmp；退出码 0=通过）
 *       编译时必须带 -Wl,--wrap=write -Wl,--wrap=fdatasync -Wl,--wrap=close
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <stdatomic.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "writer.h"
#include "vib_format.h"

/* 链接期 --wrap 提供的真实符号 */
extern ssize_t __real_write(int fd, const void *buf, size_t n);
extern int     __real_fdatasync(int fd);
extern int     __real_close(int fd);

/* ---- 故障注入开关（由测试逐个场景设置） ---- */
static int g_fail_first_data_write;   /* 第一个数据块 write 返回 -1/EIO */
static int g_short_first_data_write;  /* 第一个数据块 write 短写 1 B，且 errno=0 */
static int g_fail_end_write;          /* 结束块 write 返回 -1/EIO */
static int g_short_end_write;         /* 结束块 write 短写 1 B，且 errno=0 */
static int g_fail_sync_mode;          /* 0=正常 1=一次 EIO 2=一次 EINVAL 3=每次 EIO */
static int g_fail_close;              /* close 返回 -1/EIO */

static void fault_reset(void)
{
    g_fail_first_data_write = 0;
    g_short_first_data_write = 0;
    g_fail_end_write = 0;
    g_short_end_write = 0;
    g_fail_sync_mode = 0;
    g_fail_close = 0;
}

static uint16_t block_type_of(const void *buf, size_t n)
{
    if (n < 2)
        return 0;
    const uint8_t *b = (const uint8_t *)buf;
    return (uint16_t)((uint16_t)b[0] | (uint16_t)((uint16_t)b[1] << 8));
}

/* 只拦截生产 writer.c 的三类落盘调用；文件头（256 B，魔数 "VIB2"）不拦。 */
ssize_t __wrap_write(int fd, const void *buf, size_t n)
{
    const uint16_t bt = block_type_of(buf, n);

    if (n == (size_t)VIB_BLOCK_HEADER_SIZE && bt == (uint16_t)VIB_BLOCK_TYPE_END) {
        if (g_fail_end_write) {
            g_fail_end_write = 0;
            errno = EIO;
            return -1;
        }
        if (g_short_end_write) {
            g_short_end_write = 0;
            errno = 0;                        /* 短写不设 errno：必须由 EIO 兜底 */
            (void)__real_write(fd, buf, n - 1);
            return (ssize_t)(n - 1);
        }
    } else if (bt == (uint16_t)VIB_BLOCK_TYPE_DATA) {
        if (g_fail_first_data_write) {
            g_fail_first_data_write = 0;
            errno = EIO;
            return -1;
        }
        if (g_short_first_data_write) {
            g_short_first_data_write = 0;
            errno = 0;
            (void)__real_write(fd, buf, n - 1);
            return (ssize_t)(n - 1);
        }
    }
    return __real_write(fd, buf, n);
}

int __wrap_fdatasync(int fd)
{
    if (g_fail_sync_mode == 1) { g_fail_sync_mode = 0; errno = EIO;    return -1; }
    if (g_fail_sync_mode == 2) { g_fail_sync_mode = 0; errno = EINVAL; return -1; }
    if (g_fail_sync_mode == 3) { errno = EIO; return -1; }
    return __real_fdatasync(fd);
}

int __wrap_close(int fd)
{
    if (g_fail_close) {
        g_fail_close = 0;
        errno = EIO;
        return -1;
    }
    return __real_close(fd);
}

/* ------------------------------------------------------------------ 小工具 */
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

/* 是否存在**完整**的结束块（32 B 块头都要在）。短写的 31 B 残块不算。 */
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
        if (btype == (uint16_t)VIB_BLOCK_TYPE_END) {
            uint8_t rest[VIB_BLOCK_HEADER_SIZE - 4];
            found = (fread(rest, 1, sizeof(rest), f) == sizeof(rest));
            break;
        }
        if (fseek(f, (long)(VIB_BLOCK_HEADER_SIZE - 4) + (long)((size_t)ns * VIB_RECORD_SIZE),
                  SEEK_CUR) != 0)
            break;
    }
    fclose(f);
    return found;
}

static void assert_dir(const char *p)
{
    if (mkdir(p, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "mkdir %s 失败: %s\n", p, strerror(errno));
        exit(2);
    }
}

static void make_header(vib_file_header_t *fh)
{
    memset(fh, 0, sizeof(*fh));
    fh->odr_hz          = 8000;
    fh->fs_accel_g      = 16;
    fh->fs_gyro_dps     = 2000;
    fh->who_am_i        = 0x47;
    fh->tmst_scale_num  = (uint16_t)VIB_TMST_SCALE_NUM;
    fh->tmst_scale_den  = (uint16_t)VIB_TMST_SCALE_DEN;
    fh->tmst_source     = (uint8_t)VIB_TMST_SOURCE_INTERNAL;
    fh->tmst_res        = (uint8_t)VIB_TMST_RES_1US;
    fh->count_unit      = 1;
    fh->count_endian    = 1;
}

static int submit_blocks(writer_t *w, int nblocks, uint32_t *idx)
{
    uint8_t payload[VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE];
    memset(payload, 0, sizeof(payload));
    for (int i = 0; i < nblocks; i++) {
        vib_block_header_t bh;
        memset(&bh, 0, sizeof(bh));
        bh.block_type  = (uint16_t)VIB_BLOCK_TYPE_DATA;
        bh.n_samples   = (uint16_t)VIB_SAMPLES_PER_BLOCK;
        bh.block_index = (*idx)++;
        if (writer_submit(w, &bh, payload, sizeof(payload)) != 0)
            return -1;
    }
    return 0;
}

/* 等待写盘线程消化到 want 个块，或等到出现 error（最多 timeout_ms 毫秒）。 */
static void wait_settled(writer_t *w, uint64_t want, int timeout_ms)
{
    for (int i = 0; i < timeout_ms; i++) {
        if (atomic_load_explicit(&w->error, memory_order_relaxed) != 0)
            return;
        if (atomic_load_explicit(&w->tail, memory_order_relaxed) >= want)
            return;
        usleep(1000);
    }
    fprintf(stderr, "  ! 等待写盘线程超时（tail=%llu want=%llu）\n",
            (unsigned long long)atomic_load_explicit(&w->tail, memory_order_relaxed),
            (unsigned long long)want);
}

#define BLOCK_BYTES ((long)(VIB_BLOCK_HEADER_SIZE + VIB_SAMPLES_PER_BLOCK * VIB_RECORD_SIZE))

int main(int argc, char **argv)
{
    const char *base = (argc > 1) ? argv[1] : "/tmp";
    char reject[1024], okdir[1024];
    snprintf(reject, sizeof(reject), "%s/must_reject", base);
    snprintf(okdir,  sizeof(okdir),  "%s/struct_ok",   base);
    assert_dir(base);
    assert_dir(reject);
    assert_dir(okdir);

    /* 死锁保护：任一场景 pthread_join 卡死时由 SIGALRM 终止，让回归显式失败 */
    alarm(90);

    printf("== 0) 无故障时不得误报错误（正常线程/CRC/落盘逻辑不被破坏）==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/normal.bin", okdir);
        fault_reset();
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 3, &idx) == 0, "投递 3 个数据块");
        wait_settled(&w, 3, 3000);
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == 0, "无故障 → writer_error()==0（实际 %d）", writer_error(&w));
        CHECK(has_end_block(path), "正常路径写出完整结束块");
        CHECK(file_size(path) == (long)VIB_HEADER_SIZE + 3 * BLOCK_BYTES
                                + (long)VIB_BLOCK_HEADER_SIZE,
              "文件长度 == 256 + 3×544 + 32（实际 %ld）", file_size(path));
    }

    printf("== 1) 数据块写失败 + 队列仍有块 → error=EIO，finish 能结束（不死锁）==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/data_fail.bin", reject);
        fault_reset();
        g_fail_first_data_write = 1;
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 4, &idx) == 0, "投递 4 个块（队尾未消费）");
        wait_settled(&w, 4, 3000);
        CHECK(writer_error(&w) == EIO, "首个 error == EIO（实际 %d）", writer_error(&w));
        CHECK(atomic_load_explicit(&w.head, memory_order_relaxed)
                  > atomic_load_explicit(&w.tail, memory_order_relaxed),
              "队列仍有未消费块（head=%llu > tail=%llu）",
              (unsigned long long)atomic_load_explicit(&w.head, memory_order_relaxed),
              (unsigned long long)atomic_load_explicit(&w.tail, memory_order_relaxed));
        printf("  … 调用 writer_finish（若 join 死锁会被 alarm 杀掉）\n");
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == EIO, "finish 后仍保留首个 error（EIO，实际 %d）", writer_error(&w));
        CHECK(!has_end_block(path), "error 非 0 时不写干净结束块");
        CHECK(file_size(path) == (long)VIB_HEADER_SIZE,
              "只有 256 B 文件头（实际 %ld）", file_size(path));
    }

    printf("== 2) 数据块短写且 errno=0 → error=EIO（不是 0/健康）==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/data_short.bin", reject);
        fault_reset();
        g_short_first_data_write = 1;
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 3, &idx) == 0, "投递 3 个块");
        wait_settled(&w, 3, 3000);
        CHECK(writer_error(&w) == EIO, "短写 errno=0 → EIO（实际 %d）", writer_error(&w));
        writer_finish(&w, 0);
        CHECK(!has_end_block(path), "短写后不写干净结束块");
        CHECK(file_size(path) == (long)VIB_HEADER_SIZE + BLOCK_BYTES - 1,
              "短写只落 543 B（文件应为 %ld，实际 %ld）",
              (long)VIB_HEADER_SIZE + BLOCK_BYTES - 1, file_size(path));
    }

    printf("== 3) 先成功写数据，再让结束块 write 失败 → error 非 0 ==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/end_fail.bin", reject);
        fault_reset();
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 2, &idx) == 0, "投递 2 个数据块");
        wait_settled(&w, 2, 3000);
        CHECK(writer_error(&w) == 0, "数据块全部写成功（先成功 write data）");
        CHECK(w.blocks_written == 2, "blocks_written==2（实际 %llu）",
              (unsigned long long)w.blocks_written);
        g_fail_end_write = 1;
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == EIO, "结束块写失败 → error=EIO（实际 %d）", writer_error(&w));
        CHECK(!has_end_block(path), "无完整结束块");
        CHECK(file_size(path) == (long)VIB_HEADER_SIZE + 2 * BLOCK_BYTES,
              "数据块仍在（文件 %ld，实际 %ld）",
              (long)VIB_HEADER_SIZE + 2 * BLOCK_BYTES, file_size(path));
    }

    printf("== 4) 先成功写数据，再让结束块短写(errno=0) → error=EIO + 残块 ==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/end_short.bin", reject);
        fault_reset();
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 2, &idx) == 0, "投递 2 个数据块");
        wait_settled(&w, 2, 3000);
        CHECK(writer_error(&w) == 0, "数据块全部写成功");
        g_short_end_write = 1;
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == EIO, "结束块短写 → error=EIO（实际 %d）", writer_error(&w));
        CHECK(!has_end_block(path), "31 B 残块不算完整结束块");
        CHECK(file_size(path) == (long)VIB_HEADER_SIZE + 2 * BLOCK_BYTES
                                + (long)VIB_BLOCK_HEADER_SIZE - 1,
              "文件尾部只落了 31 B（实际 %ld）", file_size(path));
    }

    printf("== 5) 周期 fdatasync 失败(EIO) → error 非 0（不再只打印）==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/sync_periodic.bin", okdir);
        fault_reset();
        g_fail_sync_mode = 1;
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 1) == 0, "writer_start(fsync_ms=1) 成功");
        usleep(5000);                       /* 让线程的 last_fsync 过期，下一块触发周期 sync */
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 1, &idx) == 0, "投递 1 个数据块");
        wait_settled(&w, 1, 3000);
        CHECK(writer_error(&w) == EIO, "周期 fdatasync 失败 → EIO（实际 %d）", writer_error(&w));
        CHECK(w.blocks_written == 1, "数据块本身已写成功（错误来自 fdatasync）");
        CHECK(w.fsync_count >= 1, "fsync_count 已计数（实际 %llu）",
              (unsigned long long)w.fsync_count);
        printf("  … 调用 writer_abort（error 非 0 时也必须能结束）\n");
        writer_abort(&w);
    }

    printf("== 6) 先成功写数据，再让收尾 fdatasync 返回 EINVAL → error=EINVAL ==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/sync_final_einval.bin", okdir);
        fault_reset();
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 2, &idx) == 0, "投递 2 个数据块");
        wait_settled(&w, 2, 3000);
        CHECK(writer_error(&w) == 0, "数据块全部写成功");
        g_fail_sync_mode = 2;               /* 收尾 fdatasync → EINVAL */
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == EINVAL,
              "EINVAL 也不等于成功持久化 → error=EINVAL（实际 %d）", writer_error(&w));
        CHECK(has_end_block(path), "结构仍完整（失败靠 .run.json 的 writer_errno 暴露）");
    }

    printf("== 7) 先成功写数据，再让 close 失败 → error 非 0 ==\n");
    {
        char path[1200];
        snprintf(path, sizeof(path), "%s/close_fail.bin", okdir);
        fault_reset();
        writer_t w;
        vib_file_header_t fh;
        make_header(&fh);
        CHECK(writer_start(&w, path, &fh, 0, 0) == 0, "writer_start 成功");
        uint32_t idx = 0;
        CHECK(submit_blocks(&w, 2, &idx) == 0, "投递 2 个数据块");
        wait_settled(&w, 2, 3000);
        CHECK(writer_error(&w) == 0, "数据块全部写成功");
        g_fail_close = 1;
        writer_finish(&w, 0);
        CHECK(writer_error(&w) == EIO, "close 失败 → error=EIO（实际 %d）", writer_error(&w));
        CHECK(has_end_block(path), "结构仍完整（失败靠 .run.json 的 writer_errno 暴露）");
    }

    printf("\n产物目录：%s（must_reject/ 交给解析器复核）\n", base);
    if (failures) {
        fprintf(stderr, "writer_fault_test: %d 项失败 ❌\n", failures);
        return 1;
    }
    printf("writer_fault_test: 全部通过 ✅\n");
    return 0;
}
