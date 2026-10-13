/* ============================================================================
 * tmst.h — 原始 ODR 时间戳（raw ticks）的展开 / 换算 / 速率估计
 *
 * 事实来源：TDK DS-000347 Rev 1.6 §12.7。记录内 16-bit 时间戳是 raw ticks：
 * 物理 µs = raw_ticks * 32/30（未用 CLKIN、TMST_RES=0）。因此 8 kHz 的相邻
 * raw 差是 117/118（不是 125），物理间隔仍是 125 µs。
 *
 * 设计纪律：
 *   1) 累加器只存整数「raw ticks 总和」，物理 µs 一律由它派生，绝不做逐样本
 *      ×32/30 的浮点累加——避免舍入漂移。
 *   2) 速率用「总 raw ticks / 总物理时间」估计（总 ticks / 总 time），不用
 *      「1e6 / median(Δ)」——后者对 117/118 抖动更敏感且丢失全程信息。
 *
 * 本模块是纯函数集合，无硬件依赖，可被单元测试直接链接。
 * ==========================================================================*/
#ifndef VIB_TMST_H
#define VIB_TMST_H

#include <stdint.h>
#include <stddef.h>

#define TMST_HIST_N 4096u            /* raw Δ 直方图桶数（>=4096 记入 big） */

typedef struct {
    uint64_t n_samples;              /* 送入的记录数 */
    uint64_t n_deltas;               /* 有效 Δ 数（= n_samples-1，无回绕误判） */
    uint16_t prev_raw;               /* 上一条 raw ticks */
    int      have_prev;

    uint64_t sum_raw_ticks;          /* Σ 展开后的 raw Δ（整数，规范真源） */

    uint64_t hist[TMST_HIST_N];      /* raw Δ 分布（诊断用） */
    uint64_t big;                    /* Δ >= TMST_HIST_N 的次数 */
    uint64_t out_of_window;          /* raw Δ 落在 [117,118] 之外的次数 */
    int      dmin;                   /* raw Δ 最小（展开后） */
    int      dmax;                   /* raw Δ 最大（展开后） */
} tmst_accum_t;

void tmst_accum_init(tmst_accum_t *t);

/* 送入一条 raw ticks（16-bit）。内部做 16-bit 回绕展开 + 整数累加。 */
void tmst_accum_add(tmst_accum_t *t, uint16_t raw);

/* 物理时间（µs）= sum_raw_ticks * num / den，按整数四舍五入，不丢分数。 */
uint64_t tmst_accum_total_us(const tmst_accum_t *t);

/* 端点速率 = n_deltas / (total_us * 1e-6)。无 delta 时返回 0。 */
double tmst_accum_rate_hz(const tmst_accum_t *t);

/* raw Δ 中位数（诊断显示用）。无 delta 时返回 -1。 */
int tmst_accum_median_raw(const tmst_accum_t *t);

/* 把 raw ticks 差换算成物理 µs（四舍五入到整数），供「换算后物理窗口」判定。 */
uint64_t tmst_raw_delta_us(uint32_t raw_delta);

#endif /* VIB_TMST_H */
