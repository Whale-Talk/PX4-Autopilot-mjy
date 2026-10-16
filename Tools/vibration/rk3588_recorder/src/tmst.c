/* ============================================================================
 * tmst.c — 原始 ODR 时间戳（raw ticks）的展开 / 换算 / 速率估计
 *
 * 详见 tmst.h。纯函数，无硬件依赖。
 * ==========================================================================*/
#include "tmst.h"
#include "vib_format.h"

#include <string.h>

void tmst_accum_init(tmst_accum_t *t)
{
    memset(t, 0, sizeof(*t));
    t->dmin = 1 << 20;
    t->dmax = -(1 << 20);
}

void tmst_accum_add(tmst_accum_t *t, uint16_t raw)
{
    if (!t->have_prev) {
        t->prev_raw  = raw;
        t->have_prev = 1;
        t->n_samples = 1;
        return;
    }

    /* 16-bit 回绕展开：负差 = 跨了一次回绕 */
    int d = (int)raw - (int)t->prev_raw;
    if (d < 0)
        d += (int)VIB_TMST_WRAP;
    t->prev_raw = raw;
    t->n_samples++;

    /* d ∈ [0, 65535]（回绕展开后）。d==0 = 重复时间戳（异常）；d>=4096 = 大间隔。
     * 两者都会让时间轴断裂，必须显式计数、不得静默吞掉（否则丢包/回绕会被
     * 报告成全程健康）。 */
    if (d == 0) {
        t->dup++;
        t->breaks++;
        return;
    }
    if (d >= (int)TMST_HIST_N) {
        t->big++;
        t->breaks++;
        return;
    }

    t->n_deltas++;
    t->sum_raw_ticks += (uint64_t)d;
    t->hist[d]++;
    if (d < t->dmin) t->dmin = d;
    if (d > t->dmax) t->dmax = d;

    /* raw Δ 正常窗口：名义 117.1875（物理 125 µs），实机只见 117/118。
     * 窗口 [VIB_TMST_RAW_LO, VIB_TMST_RAW_HI] = [115,120] 是**现选用的保守诊断门限**，
     * 只用来区分「名义附近的量化/抖动」与「漏样本」；integer fixture 以名义附近的整数
     * 偏移（Δ=116、Δ=119 这类名义附近整数偏移）覆盖测试（见 tests/tmst_tolerance_test.c）。
     * ⚠️ 不得由本窗口反推时钟容差：绝对钟差（TDK Rev1.6 表 4 的 ±1.5%、p.24 §4.10 的
     * ±1%，条件不同）**不能**直接等同**同一芯片内相邻 raw Δ 的变化**；绝对时钟误差须
     * 靠外部参考判定（run.c 的 host_rate_hz），本窗口不构成任何时钟精度宣称。
     * 任何落在 [1,4096) 但**超出窗口**的 Δ（如 234/236 = 漏掉一个样本）都必须计作
     * 「缺口」并让时间轴断裂/不可信——不能只当作 out_of_window 诊断继续报告健康。
     * 窗口上界远小于 2× 名义，故漏样本检测不受该门限影响。 */
    if ((uint32_t)d < VIB_TMST_RAW_LO || (uint32_t)d > VIB_TMST_RAW_HI) {
        t->out_of_window++;
        t->gap++;
        t->breaks++;
    }
}

uint64_t tmst_scale_round_u64(uint64_t ticks)
{
    const uint64_t num = VIB_TMST_SCALE_NUM;
    const uint64_t den = VIB_TMST_SCALE_DEN;
    /* round(ticks * num / den)，先除后乘，避免 ticks*num 溢出；uint64 全程，不截断 */
    const uint64_t q = ticks / den;
    const uint64_t r = ticks % den;
    return q * num + (r * num + den / 2u) / den;
}

uint64_t tmst_raw_delta_us(uint32_t raw_delta)
{
    return tmst_scale_round_u64((uint64_t)raw_delta);
}

uint64_t tmst_accum_total_us(const tmst_accum_t *t)
{
    return tmst_scale_round_u64(t->sum_raw_ticks);
}

double tmst_accum_rate_hz(const tmst_accum_t *t)
{
    if (t->n_deltas == 0 || t->sum_raw_ticks == 0)
        return 0.0;
    /* rate = n_deltas / (sum_raw_ticks * 32/30 * 1e-6) */
    const double total_us = (double)t->sum_raw_ticks
                            * (double)VIB_TMST_SCALE_NUM / (double)VIB_TMST_SCALE_DEN;
    return (double)t->n_deltas * 1e6 / total_us;
}

int tmst_accum_median_raw(const tmst_accum_t *t)
{
    if (t->n_deltas == 0)
        return -1;
    uint64_t acc = 0;
    for (uint64_t i = 0; i < TMST_HIST_N; i++) {
        acc += t->hist[i];
        if (acc * 2 >= t->n_deltas)
            return (int)i;
    }
    return -1;                        /* 全部落在 big 桶 */
}

int tmst_accum_untrusted(const tmst_accum_t *t)
{
    return (t->dup != 0) || (t->big != 0) || (t->gap != 0);
}
