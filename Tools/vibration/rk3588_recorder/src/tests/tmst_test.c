/* ============================================================================
 * tests/tmst_test.c — 时间戳换算纯函数单元测试（无硬件）
 *
 * 验证 tmst.c：
 *   1) 117/118 raw Δ 序列 → 速率 ≈ 8000 Hz（总 ticks / 总时间）
 *   2) 16-bit 回绕展开
 *   3) raw Δ 中位数 == 117
 *   4) raw→µs 换算 round(raw*32/30)
 *   5) 把 raw ticks 当 µs 的旧口径（Δ=125）会被越窗标记
 *
 * 用法: tmst_test   （退出码 0=通过，1=失败）
 * ==========================================================================*/
#include <stdio.h>
#include <math.h>
#include "tmst.h"
#include "vib_format.h"

static int failures = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; fprintf(stderr, "  ✗ " __VA_ARGS__); fprintf(stderr, "\n"); } \
    else { printf("  ✓ " __VA_ARGS__); printf("\n"); } \
} while (0)

/* 生成 raw ticks：cum = floor(i * 125*30/32) mod 65536（严格 8 kHz） */
static uint16_t synth_raw(uint64_t i)
{
    return (uint16_t)(((i * 125ull * 30ull) / 32ull) & 0xFFFFull);
}

static void test_nominal_rate(void)
{
    printf("== 117/118 序列速率 ==\n");
    tmst_accum_t t;
    tmst_accum_init(&t);
    const int n = 16000;
    for (int i = 0; i < n; i++)
        tmst_accum_add(&t, synth_raw((uint64_t)i));

    const double rate = tmst_accum_rate_hz(&t);
    const int median = tmst_accum_median_raw(&t);
    CHECK(median == 117, "raw Δ 中位数 == 117（实际 %d）", median);
    CHECK(fabs(rate - 8000.0) < 1.0, "速率 ≈ 8000 Hz（实际 %.3f）", rate);
    CHECK(t.out_of_window == 0, "无越窗（实际 %llu）", (unsigned long long)t.out_of_window);
    CHECK(t.n_deltas == (uint64_t)(n - 1), "Δ 数 == n-1");
    CHECK(tmst_raw_delta_us(117) == 125, "round(117*32/30) == 125 µs");
    CHECK(tmst_raw_delta_us(118) == 126, "round(118*32/30) == 126 µs");
    /* 总时间 ≈ (n-1)*125 µs */
    const uint64_t total_us = tmst_accum_total_us(&t);
    CHECK(total_us > (uint64_t)(n - 1) * 124 && total_us < (uint64_t)(n - 1) * 126,
          "总时间 ≈ (n-1)*125 µs（实际 %llu）", (unsigned long long)total_us);
}

static void test_rollover(void)
{
    printf("== 16-bit 回绕展开 ==\n");
    tmst_accum_t t;
    tmst_accum_init(&t);
    /* 从接近回绕点开始：0xFFF0 → 0x0007 → ... 跨越 0xFFFF→0x0000 */
    uint16_t seq[] = { 0xFFF0u, 0x0007u, 0x001E, 0x0035 };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
        tmst_accum_add(&t, seq[i]);
    /* Δ = 0x0007-0xFFF0 = -65513 → +65536 = 23；接着 23, 23 */
    CHECK(t.n_deltas == 3, "跨回绕得到 3 个 Δ");
    CHECK(t.dmin == 23 && t.dmax == 23, "回绕 Δ 都展开为 23");
}

static void test_wrong_us_assumption(void)
{
    printf("== 把 raw ticks 当 µs（Δ=125）会被标记 ==\n");
    tmst_accum_t t;
    tmst_accum_init(&t);
    for (uint64_t i = 0; i < 200; i++)
        tmst_accum_add(&t, (uint16_t)((i * 125u) & 0xFFFFu));
    CHECK(t.out_of_window == t.n_deltas, "Δ=125 全部越窗（117/118 窗口之外）");
}

static void test_long_total_no_truncation(void)
{
    printf("== long total：>2^32 raw ticks 不截断到 uint32 ==\n");
    /* 直接复现反例：sum_raw_ticks=4,500,000,000 → 应为 4,800,000,000 µs，
     * 旧实现截断到 uint32 得到 218,701,551。 */
    tmst_accum_t t;
    tmst_accum_init(&t);
    t.sum_raw_ticks = 4500000000ull;
    CHECK(tmst_accum_total_us(&t) == 4800000000ull,
          "4.5e9 ticks → 4.8e9 µs（实际 %llu）",
          (unsigned long long)tmst_accum_total_us(&t));

    /* 走真实累加路径跨过 2^32：每步 Δ=117（117 与 65536 互质），N=40M 样本。
     * sum_raw_ticks = 117*(N-1) = 4,679,999,883 > 2^32。 */
    tmst_accum_t a;
    tmst_accum_init(&a);
    const uint64_t n = 40000000ull;
    for (uint64_t i = 0; i < n; i++)
        tmst_accum_add(&a, (uint16_t)((i * 117ull) & 0xFFFFull));
    const uint64_t expect_sum = 117ull * (n - 1);
    CHECK(a.sum_raw_ticks == expect_sum, "整数累加 sum == 117*(N-1)（实际 %llu）",
          (unsigned long long)a.sum_raw_ticks);
    CHECK(expect_sum > (1ull << 32), "sum > 2^32（实际 %llu）",
          (unsigned long long)expect_sum);
    CHECK(a.big == 0 && a.dup == 0, "全程无重复/大间隔");
    const uint64_t expect_us = (expect_sum / 30ull) * 32ull
                             + ((expect_sum % 30ull) * 32ull + 15ull) / 30ull;
    CHECK(tmst_accum_total_us(&a) == expect_us, "总时间 uint64 换算正确（实际 %llu）",
          (unsigned long long)tmst_accum_total_us(&a));
}

static void test_dup_big_untrusted(void)
{
    printf("== 重复时间戳 / 大间隔 → 时间轴不可信 ==\n");
    tmst_accum_t t;
    tmst_accum_init(&t);
    tmst_accum_add(&t, 100);
    tmst_accum_add(&t, 100);                       /* 重复 */
    CHECK(t.dup == 1, "重复时间戳被计数（dup==1）");
    CHECK(tmst_accum_untrusted(&t) != 0, "重复 → 不可信");

    tmst_accum_t u;
    tmst_accum_init(&u);
    tmst_accum_add(&u, 100);
    tmst_accum_add(&u, 5000);                      /* 大间隔 */
    CHECK(u.big == 1, "大间隔被计数（big==1）");
    CHECK(tmst_accum_untrusted(&u) != 0, "大间隔 → 不可信");

    tmst_accum_t ok;
    tmst_accum_init(&ok);
    for (uint64_t i = 0; i < 100; i++)
        tmst_accum_add(&ok, (uint16_t)((i * 117u) & 0xFFFFu));
    CHECK(tmst_accum_untrusted(&ok) == 0, "连续 117 序列 → 可信");
}

static void test_gap_untrusted(void)
{
    printf("== 单样本缺口（Δ=234/236）→ 时间轴断裂/不可信 ==\n");
    /* 反例：234/236 rawticks 是漏掉一个样本的缺口，旧实现只 out_of_window、
     * 不 untrusted。现在必须断续/不可信。 */
    tmst_accum_t t;
    tmst_accum_init(&t);
    tmst_accum_add(&t, 0);       /* 首样本锚点 */
    tmst_accum_add(&t, 117);     /* 正常 */
    tmst_accum_add(&t, 351);     /* 117+234：中间漏一个样本（Δ=234） */
    CHECK(t.gap == 1, "Δ=234 计作 gap（实际 %llu）", (unsigned long long)t.gap);
    CHECK(t.out_of_window == 1, "Δ=234 计作 out_of_window（诊断保留）");
    CHECK(tmst_accum_untrusted(&t) != 0, "单样本缺口 → 不可信");

    /* 236 同理 */
    tmst_accum_t v;
    tmst_accum_init(&v);
    tmst_accum_add(&v, 100);
    tmst_accum_add(&v, 336);     /* Δ=236 */
    CHECK(v.gap == 1, "Δ=236 计作 gap");
    CHECK(tmst_accum_untrusted(&v) != 0, "Δ=236 → 不可信");

    /* 单纯连续 117/118 不应有 gap */
    tmst_accum_t ok;
    tmst_accum_init(&ok);
    for (uint64_t i = 0; i < 200; i++)
        tmst_accum_add(&ok, synth_raw(i));
    CHECK(ok.gap == 0, "连续 117/118 序列无 gap（实际 %llu）", (unsigned long long)ok.gap);
}

int main(void)
{
    test_nominal_rate();
    test_rollover();
    test_wrong_us_assumption();
    test_long_total_no_truncation();
    test_dup_big_untrusted();
    test_gap_untrusted();
    if (failures) {
        fprintf(stderr, "tmst_test: %d 项失败 ❌\n", failures);
        return 1;
    }
    printf("tmst_test: 全部通过 ✅\n");
    return 0;
}
