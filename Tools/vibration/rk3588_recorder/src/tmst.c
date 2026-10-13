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

    if (d <= 0)
        return;                       /* 重复/倒退样本：不进统计，避免污染速率 */
    if (d >= (int)TMST_HIST_N) {
        t->big++;
        return;
    }

    t->n_deltas++;
    t->sum_raw_ticks += (uint64_t)d;
    t->hist[d]++;
    if (d < t->dmin) t->dmin = d;
    if (d > t->dmax) t->dmax = d;

    /* raw Δ 窗口：8 kHz 应为 117/118（物理 125 µs），不是 125 */
    if ((uint32_t)d < VIB_TMST_RAW_LO || (uint32_t)d > VIB_TMST_RAW_HI)
        t->out_of_window++;
}

uint64_t tmst_raw_delta_us(uint32_t raw_delta)
{
    /* 四舍五入的整数换算：round(raw * num / den) */
    return ((uint64_t)raw_delta * VIB_TMST_SCALE_NUM + VIB_TMST_SCALE_DEN / 2u)
           / VIB_TMST_SCALE_DEN;
}

uint64_t tmst_accum_total_us(const tmst_accum_t *t)
{
    return tmst_raw_delta_us((uint32_t)t->sum_raw_ticks);
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
