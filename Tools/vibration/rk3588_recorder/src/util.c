/* ============================================================================
 * util.c — 通用工具实现
 * ==========================================================================*/
#define _GNU_SOURCE
#include "util.h"

#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

uint64_t utc_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

void sleep_us(unsigned int us)
{
    if (us < 50) {
        uint64_t t0 = mono_ns();
        while (mono_ns() - t0 < (uint64_t)us * 1000ull) {
            /* 忙等：短延时用 nanosleep 反而更差（调度粒度 ms 级） */
        }
        return;
    }
    struct timespec req = { .tv_sec = us / 1000000u,
                            .tv_nsec = (long)(us % 1000000u) * 1000L };
    while (nanosleep(&req, &req) == -1 && errno == EINTR) {
        /* 被信号打断继续睡 */
    }
}

/* ---------------- CRC-32/ISO-HDLC ---------------- */
static uint32_t crc_table[256];
static int      crc_ready = 0;

static void crc_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc_table[i] = c;
    }
    crc_ready = 1;
}

uint32_t vib_crc32_begin(void)
{
    if (!crc_ready)
        crc_init();
    return 0xFFFFFFFFu;
}

uint32_t vib_crc32_update(uint32_t crc, const void *buf, size_t len)
{
    if (!crc_ready)
        crc_init();
    const uint8_t *p = (const uint8_t *)buf;
    for (size_t i = 0; i < len; i++)
        crc = crc_table[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
    return crc;
}

uint32_t vib_crc32_end(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

uint32_t vib_crc32(const void *buf, size_t len)
{
    return vib_crc32_end(vib_crc32_update(vib_crc32_begin(), buf, len));
}

/* ---------------- 实时化 ---------------- */
int rt_setup(int cpu, int prio, int verbose)
{
    int rc = 0;

    /* 1) 锁内存，禁止换页（换页会造成 ms 级停顿） */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        if (verbose)
            fprintf(stderr, "[rt] mlockall 失败: %s（需 sudo 或 CAP_IPC_LOCK）\n", strerror(errno));
        rc = -1;
    }

    /* 2) 绑核 */
    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET((size_t)cpu, &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
            if (verbose)
                fprintf(stderr, "[rt] 绑核到 CPU%d 失败\n", cpu);
            rc = -1;
        }
    }

    /* 3) SCHED_FIFO */
    if (prio > 0) {
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            if (verbose)
                fprintf(stderr, "[rt] SCHED_FIFO(%d) 失败: %s（需 root 或 CAP_SYS_NICE）\n",
                        prio, strerror(errno));
            rc = -1;
        }
    }
    return rc;
}
