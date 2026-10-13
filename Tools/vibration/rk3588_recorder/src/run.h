/* ============================================================================
 * run.h — 采集流程（自检 → 准备 → 清空+基线 → 排空循环 → 收尾）
 *
 * 把 main.c 里「生产启动/采集顺序」抽成可复用函数 vib_run()，让 main() 与
 * 无硬件单元测试（src/tests/startup_test.c）走**同一条**生产代码路径：
 * 自检失败默认中止、准备阶段（预分配/写线程/实时化）完成后才「已验证清空 +
 * 丢包基线」、flush/baseline 失败用 writer_abort 不留干净结束块。测试通过
 * 注入 spi->xfer_fn 的确定性 mock 实际执行该顺序，而不是另写一份模型。
 * ==========================================================================*/
#ifndef VIB_RUN_H
#define VIB_RUN_H

#include <stdint.h>
#include <stddef.h>
#include "spi_dev.h"

typedef struct {
    const char *dev;
    uint32_t    spi_hz;
    uint8_t     spi_mode;
    const char *outdir;
    char        name[128];
    double      seconds;
    int         cpu;
    int         prio;
    uint32_t    fsync_ms;
    int         frame;
    int         do_selftest;
    int         selftest_only;

    /* ---- 测试注入（生产调用方置 NULL / 0） ---- */
    spi_dev_t  *spi;              /* 非 NULL：用已打开的 spi（含 mock xfer_fn），不 open/close */
    uint32_t    max_drains;       /* >0：主循环排空达该次数后退出（确定性测试用） */
} vib_run_args_t;

/* 执行完整采集。返回：0=成功；1=启动/自检/配置失败；2=采集后检出健康问题。 */
int vib_run(const vib_run_args_t *a);

#endif /* VIB_RUN_H */
