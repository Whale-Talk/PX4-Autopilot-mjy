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
    uint64_t big;                    /* Δ >= TMST_HIST_N 的次数（大间隔，时间轴在此断裂） */
    uint64_t dup;                    /* 重复时间戳次数（d==0，异常，不得静默吞掉） */
    uint64_t gap;                    /* Δ 落在 [1,4096) 但**超出正常窗口**
                                      * [VIB_TMST_RAW_LO, VIB_TMST_RAW_HI] 的次数：
                                      * 单样本缺口（234/236）或时钟异常，时间轴必须断裂/
                                      * 不可信（不能只当 out_of_window）。窗口是现选用的
                                      * 保守诊断门限（名义附近抖动 vs 漏样本），故名义附近
                                      * 的整数抖动不会进这里；它**不是时钟精度指标**，
                                      * 绝对钟差不得由它反推（须另用外部参考）。 */
    uint64_t breaks;                 /* 时间轴断裂次数（dup 或 big 或 gap，每处计 1） */
    uint64_t out_of_window;          /* raw Δ 落在正常窗口 [VIB_TMST_RAW_LO, VIB_TMST_RAW_HI]
                                      * 之外的次数（诊断，含 gap） */
    int      dmin;                   /* raw Δ 最小（展开后） */
    int      dmax;                   /* raw Δ 最大（展开后） */
} tmst_accum_t;

void tmst_accum_init(tmst_accum_t *t);

/* 送入一条 raw ticks（16-bit）。内部做 16-bit 回绕展开 + 整数累加。 */
void tmst_accum_add(tmst_accum_t *t, uint16_t raw);

/* 物理时间（µs）= sum_raw_ticks * num / den，按整数四舍五入，不丢分数。 */
uint64_t tmst_accum_total_us(const tmst_accum_t *t);

/* 端点速率 = n_deltas / (total_us * 1e-6)。
 * ⚠️ 口径：这是**芯片内部标称时基的换算速率**（raw*32/30，未接 CLKIN），不是独立
 * 绝对实测 ODR——本模块不做、也不声称绝对时钟校准；独立参考量见 run.c 的
 * host_rate_hz（数据块 t_read 窗口）。无 delta 时返回 0。 */
double tmst_accum_rate_hz(const tmst_accum_t *t);

/* raw Δ 中位数（诊断显示用）。无 delta 时返回 -1。 */
int tmst_accum_median_raw(const tmst_accum_t *t);

/* 把 raw ticks 差换算成物理 µs（四舍五入到整数），供「换算后物理窗口」判定。 */
uint64_t tmst_raw_delta_us(uint32_t raw_delta);

/* 把 raw ticks 总量换算成物理 µs（整数四舍五入），uint64 安全，**不截断到 uint32**。
 * 用「先除后乘」避免 raw*num 溢出；sum 为 Σ 展开后的 raw Δ（可超 2^32）。 */
uint64_t tmst_scale_round_u64(uint64_t ticks);

/* 时间轴是否可信：出现重复时间戳、大间隔或单样本缺口时返回非 0。此处「单样本缺口」
 * 指 raw Δ ∈ [1,4096) 但超出正常窗口 [VIB_TMST_RAW_LO, VIB_TMST_RAW_HI]（现为
 * [115,120]，随 vib_format.h 的宏走；旧注释写的「非 117/118」是已废弃的零容差口径），
 * 如漏 1 个样本的 234/236。
 * 该窗口只是**现选用的保守诊断门限**（名义附近抖动 vs 漏样本）：**不代表实测时钟
 * 精度**，绝对钟差不得由它反推（须另用外部参考），也**不构成「无丢包」保证**
 * （丢包另由 FIFO_LOST_PKT / block_index / FIFO 满标志判定）。
 * 调用方据此把该批次标记为「时间轴不可信」，不得静默当作全程健康。 */
int tmst_accum_untrusted(const tmst_accum_t *t);

#endif /* VIB_TMST_H */
