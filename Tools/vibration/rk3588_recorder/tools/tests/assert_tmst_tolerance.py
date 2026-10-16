#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
assert_tmst_tolerance.py — 反例（Python 侧）：规格内 ODR 偏移不得判「缺口/不可信」

与 src/tests/tmst_tolerance_test.c 对应，验证**解析器侧**（bin_reader）与生产
时间基准同一口径：未接 CLKIN 的 PLL ODR 精度 ±1%（见 数据格式规范 §4），
故正常 raw Δ 可以是 116..119；只有真正的漏样本（≈2× 名义 ≈232–237）才算缺口。

fixture 复用 assert_tmst_gap.write_vib3 的**真实字节**写入路径（逐块 CRC 实算），
只把名义间隔改成目标 ODR —— 不复制解析逻辑。

用法: python3 assert_tmst_tolerance.py   （退出码 0=通过）
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))       # tools/
sys.path.insert(0, HERE)                        # tools/tests/
import bin_reader as br                          # noqa: E402
import assert_tmst_gap as F                      # noqa: E402

FAILS = []


def check(cond, label):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s" % label)
        FAILS.append(label)


def build(path, nrec, rate_hz, gap_at=None, gap_ticks=0):
    """以 rate_hz 的恒定物理间隔生成 raw ticks（整数运算）；可注入一次缺口。"""
    denom = rate_hz * 32

    def nominal_ticks(i):
        return i * 30000000 // denom

    F.nominal_ticks = nominal_ticks          # 让复用 builder 采用目标 ODR
    return F.write_vib3(path, nrec, gap_at=gap_at, gap_ticks=gap_ticks)


def main():
    tmp = tempfile.mkdtemp(prefix="tmst_tol_")
    nrec = 8000

    print("== 窗口常量必须覆盖 ±1% PLL 规格 ==")
    check(br.TMST_RAW_LO <= 116,
          "TMST_RAW_LO=%d ≤ 116（容纳 ODR +1%% 的 Δ=116）" % br.TMST_RAW_LO)
    check(br.TMST_RAW_HI >= 119,
          "TMST_RAW_HI=%d ≥ 119（容纳 ODR -1%% 的 Δ=119）" % br.TMST_RAW_HI)
    check(br.TMST_RAW_HI < 232,
          "TMST_RAW_HI=%d < 232（漏样本 2× 名义仍在窗口外）" % br.TMST_RAW_HI)

    for rate, label in ((8000, "名义"), (8080, "+1.0%"), (7920, "-1.0%")):
        print("== ODR %d Hz（%s）==" % (rate, label))
        path = build(os.path.join(tmp, "tol_%d.bin" % rate), nrec, rate)
        st = br.BinReader(path).run()
        tm = st["timing"]
        ig = st["integrity"]

        check(tm["tmst_gap"] == 0,
              "无 gap（实际 %s）" % tm["tmst_gap"])
        check(tm["tmst_breaks"] == 0,
              "无时间轴断裂（实际 %s）" % tm["tmst_breaks"])
        check(tm["tmst_untrusted"] == 0,
              "时间轴可信（不得因规格内 ODR 偏移判不可信）")
        check(tm["tmst_out_of_window"] == 0,
              "无越窗（实际 %s）" % tm["tmst_out_of_window"])
        meas = tm["measured_odr_hz"]
        check(meas is not None and abs(meas - rate) / rate < 0.003,
              "实测 ODR ≈ %d Hz（实际 %s）" % (rate, meas))
        check(st["ok"], "整体判定 OK")

        # 注入真实缺口（漏 1 样本 ⇒ 2× 名义）仍必须断段
        gap_ticks = 2 * (30000000 // (rate * 32))
        path_g = build(os.path.join(tmp, "gap_%d.bin" % rate), nrec,
                       rate, gap_at=4000, gap_ticks=gap_ticks)
        stg = br.BinReader(path_g).run()
        print("  -- 注入 %d tick（≈2×名义）漏样本 --" % gap_ticks)
        check(stg["timing"]["tmst_gap"] == 1,
              "检出 1 处缺口（实际 %s）" % stg["timing"]["tmst_gap"])
        check(stg["timing"]["tmst_untrusted"] == 1, "缺口 ⇒ 不可信")
        check(not stg["ok"], "缺口 ⇒ 整体判定不 OK")

    if FAILS:
        print("\nassert_tmst_tolerance: %d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nassert_tmst_tolerance: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
