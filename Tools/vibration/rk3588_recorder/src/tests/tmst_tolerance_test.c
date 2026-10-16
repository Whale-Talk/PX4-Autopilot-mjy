/* ============================================================================
 * tests/tmst_tolerance_test.c — 反例：名义附近的整数时基偏移不得被判「缺口/不可信」
 *
 * 背景（独立反例，非文案）：
 *   本项目口径（数据格式规范 §4、tmst.h、vib_format.h）明确：未接 CLKIN 时 ODR 走
 *   芯片内部标称时基换算（物理 µs = raw ticks × 30/32，8 kHz 名义 raw ≈ 117.1875，
 *   整数差 117/118 交替）。判定窗口 [VIB_TMST_RAW_LO, VIB_TMST_RAW_HI] = [115,120]
 *   是**现选用的保守诊断门限**，不是时钟公差指标；本测试用**名义附近的整数偏移**
 *   （±1.0%，即 8080/7920 Hz fixture 输入）覆盖它。
 *
 *   ⚠️ 时基公差另说，且**不得**由本窗口反推：TDK DS-000347 Rev1.6 表 4（CLK_SEL=01 且
 *   gyro active 的初始容差 ±1.5%）与 p.24 §4.10（±1%）条件不同，**不能**把绝对钟差
 *   直接等同**同一芯片内相邻 raw Δ 的变化**、也不得硬套进本 tick 窗口；绝对时钟误差
 *   只能靠外部参考（run.c 的 host_rate_hz）判定。下方 ±1% 输入只是**整数 fixture**，
 *   不构成任何时钟精度或绝对校准宣称。
 *
 *   若把「正常窗口」写成零容差的 {117,118}，则本测试的 ±1.0% 整数偏移 fixture 输入
 *   （8080 Hz → Δ=116/117，7920 Hz → Δ=118/119）下，约一半 Δ 被计成 gap → breaks →
 *   tmst_accum_untrusted() → 整批被判「时间轴不可信」。这属把**门限本应覆盖的名义附近
 *   抖动**误判为故障：它**只**证明本门限对这些 fixture 输入的容忍度，**不**证明实机
 *   绝对钟差，也**不**证明实机相邻 raw Δ 会取到这些整数（实机目前只见 117/118）。
 *
 *   同时必须保住真正的缺口检测：漏掉一个样本 ⇒ Δ ≈ 2× 名义 ≈ 232–237，
 *   远在正常窗口之外。
 *
 * 本测试直接链接生产 tmst.c，不复制其逻辑。
 * 用法: tmst_tolerance_test   （退出码 0=通过，1=失败）
 * ==========================================================================*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "tmst.h"
#include "vib_format.h"

static int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; fprintf(stderr, "  ✗ " __VA_ARGS__); fprintf(stderr, "\n"); } \
    else { printf("  ✓ " __VA_ARGS__); printf("\n"); } \
} while (0)

/* 恒定物理间隔 1e6/rate µs 的累计 raw ticks（整数运算，无浮点漂移）：
 * cum(i) = i * 1e6 * 30 / (rate * 32) = i * 30000000 / (rate * 32)          */
static uint16_t cum_raw(uint64_t i, uint32_t rate_hz)
{
    return (uint16_t)((i * 30000000ull / ((uint64_t)rate_hz * 32ull)) & 0xFFFFull);
}

static void check_rate(uint32_t rate_hz, const char *label)
{
    printf("== ODR %u Hz（%s）==\n", rate_hz, label);
    tmst_accum_t t;
    tmst_accum_init(&t);
    const uint64_t n = 16000;                    /* 2 s @8k */
    for (uint64_t i = 0; i < n; i++)
        tmst_accum_add(&t, cum_raw(i, rate_hz));

    const double rate = tmst_accum_rate_hz(&t);
    const int median = tmst_accum_median_raw(&t);
    const double err_pct = fabs(rate - (double)rate_hz) / (double)rate_hz * 100.0;

    CHECK(t.gap == 0, "无 gap（名义附近偏移不得判缺口；实际 %llu）",
          (unsigned long long)t.gap);
    CHECK(t.dup == 0, "无重复时间戳");
    CHECK(t.big == 0, "无大间隔");
    CHECK(t.out_of_window == 0, "无越窗");
    CHECK(tmst_accum_untrusted(&t) == 0,
          "时间轴可信（不得因名义附近时基偏移被判不可信）");
    CHECK(err_pct < 0.3, "速率估计 ≈ %u Hz（实际 %.2f Hz，偏差 %.3f%%）",
          rate_hz, rate, err_pct);
    CHECK(median >= (int)VIB_TMST_RAW_LO && median <= (int)VIB_TMST_RAW_HI,
          "raw Δ 中位数 %d 落在正常窗口 [%u,%u]", median,
          VIB_TMST_RAW_LO, VIB_TMST_RAW_HI);
    CHECK(t.n_deltas == n - 1, "Δ 数 == n-1");
}

/* 真实缺口（漏一个样本）在同样的名义附近偏移下仍必须被检出 */
static void check_dropout_still_detected(uint32_t rate_hz, const char *label)
{
    printf("== ODR %u Hz（%s）+ 漏 1 样本 ⇒ 仍须判缺口 ==\n", rate_hz, label);
    tmst_accum_t t;
    tmst_accum_init(&t);
    const uint64_t gap_at = 500;
    for (uint64_t i = 0; i < 1000; i++) {
        uint16_t raw = cum_raw(i, rate_hz);
        if (i == gap_at) {
            /* 额外再加一个名义间隔 ⇒ 该处 Δ ≈ 2× 名义（漏一个样本） */
            const uint16_t step = (uint16_t)(cum_raw(gap_at, rate_hz) -
                                             cum_raw(gap_at - 1, rate_hz));
            raw = (uint16_t)(cum_raw(i, rate_hz) + step);
        }
        tmst_accum_add(&t, raw);
    }
    CHECK(t.gap == 1, "检出 1 处缺口（实际 %llu）", (unsigned long long)t.gap);
    CHECK(tmst_accum_untrusted(&t) != 0, "缺口 ⇒ 时间轴不可信（仍须断段）");
}

int main(void)
{
    printf("== 窗口必须覆盖名义附近的整数 raw Δ（现选用的保守诊断门限）==\n");
    /* 门限口径：名义 raw = 117.1875。用名义附近的整数偏移（±1.0% fixture 输入）
     * 覆盖 ⇒ 下界 ≤116、上界 ≥119 即可容纳。这与时钟公差无关：**不得**由本窗口
     * 反推时钟容差，也不构成任何时钟精度/绝对校准宣称（绝对钟差须靠外部参考）。 */
    CHECK(VIB_TMST_RAW_LO <= 116u,
          "窗口下界 %u ≤ 116（容纳名义附近整数偏移 fixture 的 Δ=116）", VIB_TMST_RAW_LO);
    CHECK(VIB_TMST_RAW_HI >= 119u,
          "窗口上界 %u ≥ 119（容纳名义附近整数偏移 fixture 的 Δ=119）", VIB_TMST_RAW_HI);
    /* 同时不能宽到把「漏 1 样本」吞掉：2× 名义 ≈ 232，必须仍在窗口外 */
    CHECK(VIB_TMST_RAW_HI < 232u,
          "窗口上界 %u < 232（漏样本的 2× 名义间隔仍被判缺口）", VIB_TMST_RAW_HI);

    check_rate(8000, "名义");
    check_rate(8080, "fixture：名义附近 +1.0% 整数偏移");
    check_rate(7920, "fixture：名义附近 -1.0% 整数偏移");
    check_dropout_still_detected(8000, "名义");
    check_dropout_still_detected(8080, "+1.0% fixture");
    check_dropout_still_detected(7920, "-1.0% fixture");

    if (failures) {
        fprintf(stderr, "tmst_tolerance_test: %d 项失败 ❌\n", failures);
        return 1;
    }
    printf("tmst_tolerance_test: 全部通过 ✅\n");
    return 0;
}
