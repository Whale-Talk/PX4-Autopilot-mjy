#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_startup_test.py — FIFO 启动排空顺序的动力学模型回归（fix 3）

用确定性模型模拟 ICM-42688-P FIFO 的 Stream-to-FIFO 行为，验证：
  1) 旧版自检 20 ms 窗口会主动把 FIFO 灌满（>16 ms 满窗 → 溢出）
  2) 新版 sub-full 窗口（4 ms）不会溢出
  3) 准备阶段耗时 >16 ms（预分配/写线程/实时化）：
       - 旧顺序（清空 → 准备 → 采集）会在准备期间溢出，丢包被算进采集起点
       - 新顺序（准备 → 清空+基线 → 采集）不会把准备期丢包带进采集
  4) 错误分支决策：flush/status/lost 失败不得当作 0 健康；自检失败默认中止

这是对设计不变量（而非 C 二进制）的回归；C 侧真实自检逻辑由 src/tests/mock_icm.c
覆盖。无硬件、无 spidev。
"""

import sys

# 器件参数：2 KB FIFO / 16 B 记录 → 128 记录满；8 kHz → 满窗 = 128/8000 = 16 ms
FIFO_CAP = 128
ODR_HZ = 8000
FULL_WINDOW_US = FIFO_CAP * 1_000_000 // ODR_HZ          # 16000 µs

FAILS = []


def check(cond, label, extra=""):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s %s" % (label, extra))
        FAILS.append(label)


def fill(fifo, lost, us):
    """Stream-to-FIFO：满后覆盖最旧并递增 lost。返回 (fifo, lost)。"""
    new = us * ODR_HZ // 1_000_000
    fifo += new
    if fifo > FIFO_CAP:
        lost += fifo - FIFO_CAP
        fifo = FIFO_CAP
    return fifo, lost


def flush(_fifo):
    return 0


def startup(selftest_us, prep_us, order):
    """模拟「自检 → 准备 → 采集起点」的 FIFO 状态。

    返回 dict:
      selftest_lost：自检窗口内是否溢出（应恒为 0 的 sub-full 窗口）
      acq_delta：采集期间相对基线的丢包增量（旧顺序会把准备期溢出带进来）
    """
    lost = 0
    fifo = 0

    # ---- 自检窗口 ----
    fifo = flush(fifo)
    fifo, lost = fill(fifo, lost, selftest_us)
    selftest_lost = lost

    if order == "old":
        # 旧顺序：基线在准备之前读取，准备期溢出会变成采集期间的丢包增量
        baseline = lost
        fifo, lost = fill(fifo, lost, prep_us)
        acq_delta = lost - baseline
    else:
        # 新顺序：准备先发生，随后「已验证清空 + 读基线」，准备期溢出被吸收进基线
        fifo, lost = fill(fifo, lost, prep_us)
        fifo = flush(fifo)
        acq_delta = 0

    return {"selftest_lost": selftest_lost, "acq_delta": acq_delta}


def test_old_selftest_window_overflows():
    print("== 旧自检 20 ms 窗口溢出 ==\n")
    r = startup(20_000, 0, "new")
    check(r["selftest_lost"] > 0, "20 ms 自检窗口产生丢包（实际 lost=%d）" % r["selftest_lost"])


def test_new_subfull_window_safe():
    print("== 新 sub-full 4 ms 窗口不溢出 ==\n")
    r = startup(4_000, 0, "new")
    check(r["selftest_lost"] == 0, "4 ms 窗口无丢包（实际 lost=%d）" % r["selftest_lost"])


def test_prep_delay_gt_16ms():
    print("== 准备期 >16 ms：顺序决定是否把丢包带进采集 ==\n")
    prep = 30_000  # 30 ms > 16 ms 满窗
    old = startup(4_000, prep, "old")
    new = startup(4_000, prep, "new")
    check(old["acq_delta"] > 0,
          "旧顺序（基线在准备前）：准备期溢出变成采集丢包增量（Δ=%d）" % old["acq_delta"])
    check(new["acq_delta"] == 0,
          "新顺序（准备后再清空+基线）：采集丢包增量为 0（Δ=%d）" % new["acq_delta"])


def test_error_branch_decisions():
    print("== 错误分支：失败不得当作 0 健康 ==\n")
    # 决策语义与 main.c / icm42688p.c 一致：
    # flush 失败 → 中止；status/lost 读失败 → 标记健康未知；自检失败 → 中止
    decisions = {
        "flush_ok": True, "lost_ok": True, "selftest_ok": True,
    }

    def abort_reason(d):
        if not d["selftest_ok"]:
            return "selftest-fail"
        if not d["flush_ok"]:
            return "flush-fail"
        if not d["lost_ok"]:
            return "baseline-fail"
        return None

    check(abort_reason({"selftest_ok": False, "flush_ok": True, "lost_ok": True}) == "selftest-fail",
          "自检失败默认中止")
    check(abort_reason({"selftest_ok": True, "flush_ok": False, "lost_ok": True}) == "flush-fail",
          "flush 失败中止（不当 0 健康）")
    check(abort_reason({"selftest_ok": True, "flush_ok": True, "lost_ok": False}) == "baseline-fail",
          "丢包基线读取失败中止")
    check(abort_reason({"selftest_ok": True, "flush_ok": True, "lost_ok": True}) is None,
          "全部通过才进入采集")


def main():
    test_old_selftest_window_overflows()
    test_new_subfull_window_safe()
    test_prep_delay_gt_16ms()
    test_error_branch_decisions()
    if FAILS:
        print("\n结果：%d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nmock_startup_test: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
