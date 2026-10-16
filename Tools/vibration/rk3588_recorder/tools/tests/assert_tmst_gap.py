#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
assert_tmst_gap.py — 时间戳「单样本缺口」必须切断时间轴的回归

背景（独立复核反例）：C 侧 tmst.c 已把 Δ∈[1,4096) 但非 117/118 的 Δ（如 234/236
raw ticks，= 漏掉一个样本）计作 gap/breaks/untrusted；而 Python 侧 bin_reader 只
计 out_of_window、仍保持单连续段，于是 sync_bin_ulog 可以在缺口两侧跨段做拟合。

本测试用**真实二进制 fixture**（自建 VIB3 文件，块 CRC 逐块实算，可被真解析器解析）
制造一次 234 tick 间隔，独立按字节复算 Δ（不复用解析器分类逻辑），断言：
  * 缺口被显式计数（tmst_gap/tmst_breaks）、时间轴被判不可信（ok=False）
  * 连续段被真正切开（≥2 段），碎段首尾不再强拼成「可信全程 rate」
    （measured_odr_hz=None，仅留 *_diagnostic；逐段 rate 见 segments[*]）
  * 16 位回绕跨越的缺口同样被检出（回绕展开 + 缺口判定）
  * 结构跳过（FIFO header msg）与无效哨兵样本不跨接
  * sync_bin_ulog.fit_events 拒绝跨缺口的匹配事件（只允许单段拟合）
  * 原 clean/回绕文件仍然正常（单段、ok、ODR≈8000、无越窗）

只依赖标准库 + 本仓库模块，无硬件。
用法: python3 assert_tmst_gap.py   （退出码 0=通过）
"""

import os
import shutil
import struct
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)                       # rk3588_recorder/tools
sys.path.insert(0, TOOLS)
_PARENT = os.path.dirname(os.path.dirname(TOOLS))   # Tools/vibration（含 ulog_reader）
sys.path.insert(0, _PARENT)

import bin_reader as br          # noqa: E402
import sync_bin_ulog as sync     # noqa: E402

FAILS = []

REC_FMT = "<3h3hBHB"
TMST_WRAP = 65536


def check(cond, label, extra=""):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s %s" % (label, extra))
        FAILS.append(label)


# 缺口/断段语义的必需字段；旧实现（只计 out_of_window）缺这些键时给出清晰失败，
# 而不是 KeyError 中断，便于把本测试直接当作反例判据。
NEW_KEYS = ("tmst_gap", "tmst_breaks", "tmst_untrusted", "rate_trusted",
            "measured_odr_hz_diagnostic", "tmst_raw_total_ticks",
            "tmst_skips", "tmst_invalid_breaks")


def have_gap_semantics(st):
    tm, tl = st["timing"], st["timeline"]
    missing = [k for k in NEW_KEYS if k not in tm]
    if "n_segments" not in tl:
        missing.append("n_segments")
    if missing:
        check(False, "解析器缺少时间轴断续字段 %s：缺口/断段语义未实现" % missing)
        return False
    return True


# ---------------------------------------------------------------------------
# fixture 生成（真实字节，逐块 CRC 实算；不调用 bin_reader 的解析逻辑）
# ---------------------------------------------------------------------------
def nominal_ticks(i):
    """i 号样本的**名义**累计 raw ticks（严格 8 kHz：物理 125 µs → raw 117/118 交替）"""
    return i * 125 * 30 // 32


def tmst_sequence(nrec, gap_at=None, gap_ticks=234):
    """名义序列 + 在 gap_at 处插入一次 gap_ticks 的间隔（其余 Δ 仍是 117/118）"""
    out = []
    offset = 0
    for i in range(nrec):
        if gap_at is not None and i == gap_at:
            offset = gap_ticks - (nominal_ticks(i) - nominal_ticks(i - 1))
        out.append((nominal_ticks(i) + offset) & 0xFFFF)
    return out


def write_vib3(path, nrec, gap_at=None, gap_ticks=234,
               invalid_at=None, header_at=None):
    """写一个真实 VIB3 .bin（块头/记录/结束块齐全，CRC 逐块实算）。"""
    header = struct.pack(
        br.FILE_HDR_FMT,
        b"VIB2", 3, 256, 0, 0x000201, 0xDEADBEEF, 8000, 16, 2000, 16, 0,
        123456789, 1750000000000000000, 0x47, 0, b"\x00" * 6, nrec,
        32, 30, 0, 0, 1, 1, 0, b"\x00" * 176, 0)
    crc = zlib.crc32(header[:252]) & 0xFFFFFFFF
    header = header[:252] + struct.pack("<I", crc)

    tmsts = tmst_sequence(nrec, gap_at, gap_ticks)
    blocks = []
    for start in range(0, nrec, br.SAMPLES_PER_BLOCK):
        n = min(br.SAMPLES_PER_BLOCK, nrec - start)
        payload = b""
        for i in range(start, start + n):
            flags = 0
            if invalid_at is not None and i == invalid_at:
                flags |= br.REC_FLAG_INVALID
            if header_at is not None and i == header_at:
                flags |= br.REC_FLAG_HEADER_MSG
            payload += struct.pack(REC_FMT, 1000, -2000, 2048, 164, 0, -16, 21,
                                   tmsts[i], flags)
        braw = struct.pack("<HH IQIHHII".replace(" ", ""),
                           0x0001, n, start // br.SAMPLES_PER_BLOCK,
                           123456789 + start * 125000, 0, 0, 0, 0, 0)
        bhdr = braw[:28] + struct.pack(
            "<I", zlib.crc32(payload, zlib.crc32(braw[:28])) & 0xFFFFFFFF)
        blocks.append(bhdr + payload)

    end = struct.pack("<HH IQIHHII".replace(" ", ""),
                      0xFFFF, 0, len(blocks), 123456789 + nrec * 125000, 0, 0, 0, 0, 0)
    end = end[:28] + struct.pack("<I", zlib.crc32(b"", zlib.crc32(end[:28])) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(header + b"".join(blocks) + end)
    return path


def raw_tmsts(path):
    """独立按字节扫描记录内 16-bit raw ticks（原样，不展开回绕）。"""
    out = []
    with open(path, "rb") as f:
        f.seek(br.FILE_HEADER_SIZE)
        while True:
            braw = f.read(br.BLOCK_HEADER_SIZE)
            if len(braw) < br.BLOCK_HEADER_SIZE:
                break
            bt, n = struct.unpack_from("<HH", braw)
            payload = f.read(n * br.RECORD_SIZE)
            if bt == 0xFFFF:
                break
            for off in range(0, len(payload), br.RECORD_SIZE):
                out.append(struct.unpack_from(REC_FMT, payload, off)[7])
    return out


def raw_deltas(path):
    """独立按字节复算 Δ（回绕展开），不复用解析器分类。

    返回 {记录序号: Δ}，键是 Δ 的**后一条**记录序号（首条无 Δ）。
    """
    out = {}
    prev = None
    for idx, tmst in enumerate(raw_tmsts(path)):
        if prev is not None:
            d = tmst - prev
            if d < 0:
                d += TMST_WRAP
            out[idx] = d
        prev = tmst
    return out


# ---------------------------------------------------------------------------
# 用例
# ---------------------------------------------------------------------------
def case_clean(tmp, nrec, label):
    print("== %s：原 clean/回绕文件仍正常（单段、可信、无越窗）==\n" % label)
    path = write_vib3(os.path.join(tmp, "clean.bin"), nrec)
    deltas = raw_deltas(path)
    tmst_seq = raw_tmsts(path)
    vals = sorted(set(deltas.values()))
    wraps = sum(1 for i in range(1, len(tmst_seq)) if tmst_seq[i] < tmst_seq[i - 1])
    st = br.BinReader(path).run()
    tm = st["timing"]
    tl = st["timeline"]
    if not have_gap_semantics(st):
        return st
    check(st["ok"], "ok=True（无错误无异常）")
    check(tl["n_segments"] == 1, "1 个连续段（实际 %d）" % tl["n_segments"])
    check(not tm["tmst_untrusted"] and tm["rate_trusted"], "时间轴可信")
    check(tm["tmst_out_of_window"] == 0 and tm["tmst_gap"] == 0 and tm["tmst_breaks"] == 0,
          "无越窗/无缺口/无断裂（实际 %d/%d/%d）"
          % (tm["tmst_out_of_window"], tm["tmst_gap"], tm["tmst_breaks"]))
    check(tm["measured_odr_hz"] is not None
          and abs(tm["measured_odr_hz"] - 8000.0) < 1.0,
          "标称时基换算速率 ≈ 8000 Hz（实际 %s）" % tm["measured_odr_hz"])
    check(tm["measured_odr_hz_diagnostic"] is None, "可信文件不产生「诊断 rate」")
    check(vals == [117, 118], "独立复算：Δ 只有 117/118 两种步长（实际 %s）" % vals)
    check(wraps > 0, "独立复算：raw 序列经历了 %d 次 16 位回绕" % wraps)
    return st


def case_gap(tmp, nrec, gap_at, label, expect_wrap=False):
    print("== %s：一次 234 tick 单样本缺口 → 断段/不可信 ==\n" % label)
    path = write_vib3(os.path.join(tmp, "gap.bin"), nrec, gap_at=gap_at)
    deltas = raw_deltas(path)
    odd = [i for i, d in deltas.items() if d not in (117, 118)]
    check(odd == [gap_at],
          "独立按字节复算：恰好在记录 %d 处出现唯一非 117/118 的 Δ（实际 %s）"
          % (gap_at, odd[:5]))
    check(deltas.get(gap_at) == 234,
          "该 Δ == 234 raw ticks（实际 %s）" % deltas.get(gap_at))
    if expect_wrap:
        seq = tmst_sequence(nrec, gap_at=gap_at)
        check(seq[gap_at] < seq[gap_at - 1],
              "该缺口跨越 16 位回绕（%d → %d）" % (seq[gap_at - 1], seq[gap_at]))

    st = br.BinReader(path).run()
    tm = st["timing"]
    tl = st["timeline"]
    segs = tl["segments"]
    if not have_gap_semantics(st):
        return path, st
    check(not st["ok"], "ok=False（缺口不再被当作健康全程）")
    check(tm["tmst_gap"] == 1, "tmst_gap == 1（实际 %s）" % tm["tmst_gap"])
    check(tm["tmst_breaks"] == 1, "tmst_breaks == 1（实际 %s）" % tm["tmst_breaks"])
    check(tm["tmst_out_of_window"] == 1, "缺口也计入 out_of_window（与 C 侧一致）")
    check(tm["tmst_untrusted"] and not tm["rate_trusted"], "时间轴判为不可信")
    check(len(segs) == 2, "时间轴被切成 2 段（实际 %d）" % len(segs))
    check(tm["measured_odr_hz"] is None, "全程 rate 不再给出（碎段不强拼）")
    check(tm["measured_odr_hz_diagnostic"] is not None,
          "仅保留诊断 rate（值 %s）" % tm["measured_odr_hz_diagnostic"])
    if len(segs) == 2:
        check(segs[0]["samples"] == gap_at and segs[1]["samples"] == nrec - gap_at,
              "逐段样本数按缺口切开（%d / %d，实际 %d / %d）"
              % (gap_at, nrec - gap_at, segs[0]["samples"], segs[1]["samples"]))
        check(segs[1].get("break_before") == "gap", "第二段 break_before=gap（实际 %s）"
              % segs[1].get("break_before"))
        for k, s in enumerate(segs):
            r = s.get("measured_odr_hz")
            check(r is not None and abs(r - 8000.0) < 1.0,
                  "段%d 内 rate ≈ 8000 Hz（实际 %s）" % (k, r))
        seg_sum = sum(s.get("raw_ticks") or 0 for s in segs)
        check(seg_sum + 234 == tm["tmst_raw_total_ticks"],
              "不变量：Σ段内 rawΔ + 缺口Δ == 全程诊断 rawΔ（%d + 234 == %s）"
              % (seg_sum, tm["tmst_raw_total_ticks"]))
        check(segs[1]["first_us"] - segs[0]["last_us"] > 0,
              "缺口两侧段跨度不重叠、且缺口区不计入任何段（%.1f µs）"
              % (segs[1]["first_us"] - segs[0]["last_us"]))
        # 缺口处不得被跨段补 Δ：段0 的跨度只由段内 Δ 组成
        check(segs[0].get("raw_ticks") == sum(d for i, d in deltas.items() if i < gap_at),
              "段0 rawΔ 只由本段 Δ 累加（无跨接；实际 %s）" % segs[0].get("raw_ticks"))
    return path, st


def case_skips(tmp, nrec, invalid_at=None, header_at=None):
    label = "无效哨兵样本" if invalid_at is not None else "FIFO header 结构跳过"
    print("== %s：不跨接补 Δ → 断段/不可信 ==\n" % label)
    path = write_vib3(os.path.join(tmp, "skip.bin"), nrec,
                      invalid_at=invalid_at, header_at=header_at)
    st = br.BinReader(path).run()
    tm = st["timing"]
    ig = st["integrity"]
    segs = st["timeline"]["segments"]
    at = invalid_at if invalid_at is not None else header_at
    if not have_gap_semantics(st):
        return path, st
    check(not st["ok"], "ok=False")
    check(tm["tmst_breaks"] == 1 and tm["tmst_untrusted"], "tmst_breaks==1 且不可信")
    check(tm["tmst_gap"] == 0, "不是「缺口」，而是结构/无效记录断裂（gap=0）")
    check(len(segs) == 2, "切成 2 段（实际 %d）" % len(segs))
    if len(segs) == 2:
        check(segs[1]["break_before"] == ("invalid" if invalid_at is not None else "skip"),
              "第二段 break_before=%s（实际 %s）"
              % ("invalid" if invalid_at is not None else "skip", segs[1]["break_before"]))
    if invalid_at is not None:
        check(ig["invalid_flag_records"] == 1, "invalid_flag_records == 1")
        check(tm["tmst_invalid_breaks"] == 1, "tmst_invalid_breaks == 1")
        check(ig["records"] == nrec, "无效样本仍计入记录数（%d）" % ig["records"])
        if len(segs) == 2:
            check(segs[0]["samples"] == at and segs[1]["samples"] == nrec - at - 1,
                  "无效样本自身不属于任何段（%d / %d）"
                  % (segs[0]["samples"], segs[1]["samples"]))
    else:
        check(ig["bad_record_flags"] == 1, "bad_record_flags == 1")
        check(tm["tmst_skips"] == 1, "tmst_skips == 1")
        check(ig["records"] == nrec - 1, "header 消息不计入记录数（%d）" % ig["records"])
        if len(segs) == 2:
            check(segs[0]["samples"] == at and segs[1]["samples"] == nrec - at - 1,
                  "跳过记录两侧各自成段（%d / %d）"
                  % (segs[0]["samples"], segs[1]["samples"]))
    return path, st


def case_sync_refuses_cross_gap(tmp, st):
    print("== sync：缺口两侧的事件不得跨段拟合（不只看 summary/OLS）==\n")
    segs = st["timeline"]["segments"]
    if len(segs) != 2:
        check(False, "需要 2 段才能验证跨段拒绝（实际 %d）" % len(segs))
        return

    def ev(ext, x7, k):
        return {"external_us": ext, "x7_us": x7, "label": "e%d" % k,
                "provenance": "manual-tap", "uncertainty_us": 50.0}

    a = 1.0 + 100.0 / 1e6
    base = 10_000_000.0
    s0, s1 = segs[0], segs[1]
    # 3 个事件落在段0 内 → 单段拟合可用
    e_in = [ev(s0["first_us"] + 1000.0 * (k + 1), 0, k) for k in range(3)]
    e_in = [{"external_us": e["external_us"],
             "x7_us": a * e["external_us"] + base, "label": e["label"],
             "provenance": e["provenance"], "uncertainty_us": e["uncertainty_us"]}
            for e in e_in]
    x7_seg = {"device_id": 0x36001A, "first_us": base - 1e6, "last_us": base + 1e8,
              "packets": 1, "samples": 1}
    fit_in = sync.fit_events(e_in, segs, x7_seg, 0x36001A, phase_budget_us=200.0)
    check(fit_in["status"] == "ok", "段内 3 个事件 → 仍可拟合（实际 %s）"
          % fit_in["status"])

    # 跨缺口：2 个在段0、1 个在段1 → 必须拒绝
    e_cross = [ev(s0["first_us"] + 1000.0, a * (s0["first_us"] + 1000.0) + base, 0),
               ev(s0["last_us"] - 1000.0, a * (s0["last_us"] - 1000.0) + base, 1),
               ev(s1["first_us"] + 1000.0, a * (s1["first_us"] + 1000.0) + base, 2)]
    fit_cross = sync.fit_events(e_cross, segs, x7_seg, 0x36001A)
    check(fit_cross["status"] == "out_of_range",
          "跨缺口事件 → out_of_range（实际 %s）" % fit_cross["status"])


def case_csv_segments(tmp, path):
    print("== CSV 导出：segment 列在缺口处切开，逐段 t_us 单调 ==\n")
    csv_path = os.path.join(tmp, "gap.csv")
    br.BinReader(path, csv_path=csv_path).run()
    with open(csv_path, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        segs = {}
        prev = {}
        mono = True
        for line in f:
            row = line.strip().split(",")
            if len(row) < 3:
                continue
            s = int(row[1])
            t = float(row[2])
            segs[s] = segs.get(s, 0) + 1
            if s in prev and t < prev[s]:
                mono = False
            prev[s] = t
    check(header[:3] == ["index", "segment", "t_us"], "CSV 头含 index,segment,t_us")
    check(sorted(segs) == [0, 1], "segment 列被切成 {0,1}（实际 %s）" % sorted(segs))
    check(mono, "每段内 t_us 单调不减（不跨段拼轴）")


def main():
    nrec = 2 * 8000
    tmp = tempfile.mkdtemp(prefix="tmst_gap_")
    try:
        case_clean(tmp, nrec, "clean")

        # 段内缺口：挑一个不在回绕附近的记录
        gap_at = 5000
        path, st = case_gap(tmp, nrec, gap_at, "单样本缺口(段内)")
        case_sync_refuses_cross_gap(tmp, st)
        case_csv_segments(tmp, path)

        # 缺口跨越 16 位回绕
        wrap_at = next(i for i in range(1, nrec)
                       if (nominal_ticks(i) & 0xFFFF) > 65536 - 120)
        case_gap(tmp, nrec, wrap_at + 1, "单样本缺口(跨 16 位回绕)", expect_wrap=True)

        case_skips(tmp, nrec, invalid_at=3000)
        case_skips(tmp, nrec, header_at=3000)
    finally:
        if FAILS:
            print("\n（保留 fixture 目录供排查：%s）" % tmp)
        else:
            shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print("\n结果：%d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nassert_tmst_gap: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
