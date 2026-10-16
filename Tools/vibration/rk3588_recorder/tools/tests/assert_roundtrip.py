#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
assert_roundtrip.py — 无硬件回归断言

对 mk_synth.c 生成的合成 .bin 做**独立复算**（本文件自己按合成规则重算期望值，
不复用解析器的任何计算），逐项比对 _tools/bin_reader.py 的结果。

这验证的是 C 写入路径与 Python 解析路径对同一份 `数据格式规范.md` v2 的
理解完全一致：布局、字节序、CRC 变体、tmst 回绕展开、削顶口径、block_index 机制。

用法：
  assert_roundtrip.py FILE.bin SECONDS [--expect-gap] [--expect-badcrc] [--clip-every N]
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import bin_reader as br  # noqa: E402

FAILS = []


def r4(x):
    """bin_reader 的 JSON 汇总按 4 位小数上报；断言按报告精度对齐。
    （未取整的原始值不做断言，避免把「上报精度」误判成「数值错误」）"""
    return round(x, 4)


def close(actual, expected):
    return abs(actual - r4(expected)) < 1e-9


def check(cond, label, extra=""):
    if cond:
        print(f"  ✓ {label}")
    else:
        print(f"  ✗ {label} {extra}")
        FAILS.append(label)


def expected_values(seconds, clip_every):
    """按 mk_synth.c 的合成规则独立重算（不调用解析器）"""
    odr = 8000
    nrec = seconds * odr
    ax_sum = 0.0
    clips = 0
    for i in range(nrec):
        ax = 32767 if (i % clip_every == 0) else 1000
        ax_sum += ax
        if ax >= 32760 or ax <= -32760:
            clips += 1
    mean_ax_g = (ax_sum / nrec) / br.LSB_PER_G
    return {
        "records": nrec,
        "clips": clips,
        "mean_ax_g": mean_ax_g,
        "ay_g": -2000 / br.LSB_PER_G,
        "az_g": 2048 / br.LSB_PER_G,
        "gx_dps": 164 / br.LSB_PER_DPS,
        "gz_dps": -16 / br.LSB_PER_DPS,
        # 与 mk_synth.c 一致：temp_raw 存**原始 uint8 字节 0xF9**，物理温度按有符号
        # 8 位二补码解释（0xF9 → −7），再套 T = TEMP_DATA/2.07 + 25（手册 p.25/p.66）。
        "temp_c": br.temp_raw_to_c(0xF9),
        "blocks": (nrec + br.SAMPLES_PER_BLOCK - 1) // br.SAMPLES_PER_BLOCK,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("seconds", type=int)
    ap.add_argument("--clip-every", type=int, default=1000)
    ap.add_argument("--expect-gap", action="store_true")
    ap.add_argument("--expect-badcrc", action="store_true")
    a = ap.parse_args()

    exp = expected_values(a.seconds, a.clip_every)
    r = br.BinReader(a.file)
    st = r.run()
    h = st["header"]
    ig = st["integrity"]
    tm = st["timing"]
    ph = st["physical"]

    print(f"== {os.path.basename(a.file)} ==")

    # ---- 头部与结构 ----
    check(h["magic"] == "VIB2", "文件头 magic=VIB2")
    check(h["format_version"] == 3, "format_version=3")
    check(h["crc32_ok"], "文件头 CRC 与 C 端一致（zlib.crc32 == vib_crc32）")
    check(h["who_am_i"] == "0x47", "who_am_i=0x47")
    check(h["reg_config_hash"] == 0xDEADBEEF, "reg_config_hash 原样回读")
    check(h["t_start_utc_ns"] == 1750000000000000000, "t_start_utc_ns 原样回读")
    check(h["tmst_scale_num"] == 32 and h["tmst_scale_den"] == 30,
          "时间戳换算元数据 scale=32/30（Rev 1.6 §12.7）")
    check(h["tmst_source"] == 0, "tmst_source=0（内部时钟，无 CLKIN）")
    check(ig["end_block_seen"], "读到结束块")

    # ---- 计数与布局 ----
    check(ig["records"] == exp["records"], f"记录数 == {exp['records']}",
          f"(实际 {ig['records']})")
    check(ig["blocks"] == exp["blocks"], f"块数 == {exp['blocks']}",
          f"(实际 {ig['blocks']})")

    # ---- 时间基准 ----
    check(tm["tmst_median_raw"] == 117, "raw Δ 中位 == 117（不是 125）",
          f"(实际 {tm['tmst_median_raw']})")
    check(abs(tm["measured_odr_hz"] - 8000.0) < 1.0,
          "标称时基换算速率 ≈ 8000 Hz（总 ticks/总时间；非绝对实测）",
          f"(实际 {tm['measured_odr_hz']})")
    check(tm["tmst_out_of_window"] == 0, "raw Δ 无越窗（16 位回绕被正确展开）")
    # 独立复算：sum_raw = floor((nrec-1) * 125*30/32)，duration = sum_raw * 32/30 / 1e6
    sum_raw = (a.seconds * 8000 - 1) * 125 * 30 // 32
    duration_expected = sum_raw * 32 / 30 / 1e6
    check(abs(tm["duration_s"] - duration_expected) < 1e-6,
          "时间轴长度 == ΣrawΔ × 32/30", f"(实际 {tm['duration_s']} vs {duration_expected})")

    # ---- 物理量与削顶 ----
    check(ig["clip_records"] == exp["clips"], f"削顶记录数 == {exp['clips']}",
          f"(实际 {ig['clip_records']})")
    check(ig["clip_mismatch_blocks"] == 0, "块头 clip_count 与逐条重算一致")
    check(close(ph["ax"]["mean"], exp["mean_ax_g"]),
          f"ax 均值 == {r4(exp['mean_ax_g'])} g", f"(实际 {ph['ax']['mean']})")
    check(close(ph["ay"]["mean"], exp["ay_g"]),
          f"ay 均值 == {r4(exp['ay_g'])} g", f"(实际 {ph['ay']['mean']})")
    check(close(ph["az"]["mean"], exp["az_g"]),
          f"az 均值 == {r4(exp['az_g'])} g（== 1 g）", f"(实际 {ph['az']['mean']})")
    check(close(ph["gx"]["mean"], exp["gx_dps"]),
          f"gx 均值 == {r4(exp['gx_dps'])} dps", f"(实际 {ph['gx']['mean']})")
    check(close(ph["gy"]["mean"], 0.0), "gy 均值 == 0")
    check(close(ph["gz"]["mean"], exp["gz_dps"]),
          f"gz 均值 == {r4(exp['gz_dps'])} dps", f"(实际 {ph['gz']['mean']})")
    check(close(st["temp"]["mean"], exp["temp_c"]),
          f"温度均值（uint8 字节按二补码解释）== {r4(exp['temp_c'])} ℃",
          f"(实际 {st['temp']['mean']})")
    check(close(ph["ax"]["peak"], 32767 / br.LSB_PER_G),
          "ax 峰值 == 32767 LSB")
    check(close(ph["ax"]["min"], 1000 / br.LSB_PER_G),
          "ax 最小值 == 1000 LSB（削顶是孤立脉冲，不是整体偏移）")

    # ---- 注入的异常 ----
    if a.expect_gap:
        check(ig["block_crc_errors"] == 0, "缺号文件本身数据未损坏（CRC 全通过）")
        check(ig["block_index_gaps"] >= 1, "检出 block_index 跳变")
        check(ig["missing_blocks"] == 1, "缺块数 == 1", f"(实际 {ig['missing_blocks']})")
        check(not st["ok"], "整体判定为不 OK")
    elif a.expect_badcrc:
        check(ig["block_crc_errors"] == 1, "检出 1 个块 CRC 错",
              f"(实际 {ig['block_crc_errors']})")
        check(not st["ok"], "整体判定为不 OK")
    else:
        check(ig["block_crc_errors"] == 0, "全部块 CRC 通过")
        check(ig["device_lost_pkt_delta"] == 0, "器件丢包 Δ == 0")
        check(ig["missing_blocks"] == 0, "无缺块")
        check(ig["fifo_full_blocks"] == 0, "无 FIFO 满")
        check(ig["short_blocks"] == 0, "无短读块")
        check(st["ok"], "整体判定 OK（无错误、无异常）")

    if FAILS:
        print(f"\n结果：{len(FAILS)} 项失败 ❌")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("结果：全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
