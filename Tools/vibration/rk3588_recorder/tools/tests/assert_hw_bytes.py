#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
assert_hw_bytes.py — 独立字节级 fixture 回归（无硬件）

覆盖三件事（都只用**真实二进制字节**，逐块 CRC 实算，可被真解析器解析）：

  A) 温度：v3 记录内 temp 字段是**原始 uint8 字节**（布局/CRC/格式版本不变），
     解析物理温度时必须按**有符号 8 位二补码**解释（≥128 减 256）再套
     T = TEMP_DATA/2.07 + 25（TDK DS-000347 Rev1.6 p.25/p.66）。断言：
       0x00 → 25.0 ℃、0xFF → 24.5169 ℃、0xF9 → 21.6184 ℃、正值（如 0x15）不变；
     --csv 导出与 stats 的温度**同值**（同一口径，不是两条路径各算一次）。
     同时独立复算（本文件自己按 &0xFF / 减 256 重算），不复用解析器逻辑。
  B) 短读：SHORT_READ 标志（records < 水位）只表示**正常不足水位排空**，必须是
     信息级 notice（stats["notices"]），不得单独把健康文件判成异常/退出 2；
     但 short_blocks 计数与块头 flag 必须原样保留（可追溯）。反面证据同时成立：
     真实 SPI_ERR、缺包（缺块）、坏 CRC、缺结束块任一存在 ⇒ ok=False 且退出码 2。
  C) 时基：机器可读元数据必须显式声明「芯片内部标称时基换算、未做绝对时钟校准、
     不得声明绝对 8000 Hz 已校准」，并与独立参考量 host_rate_hz 并列给出。

用法: python3 assert_hw_bytes.py   （退出码 0=通过）
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

import bin_reader as br                             # noqa: E402

FAILS = []
REC_FMT = "<3h3hBHB"

# 与 C 端 vib_format.h / 数据格式规范.md 一致：水位 = 32 条
WATERMARK = 32
# 记录里的其余字段用固定值，保证「温度是唯一变量」
FIXED_ACCEL = (1000, -2000, 2048)
FIXED_GYRO = (164, 0, -16)


def check(cond, label, extra=""):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s %s" % (label, extra))
        FAILS.append(label)


def build_vib3(path, nrec, temp_at=None, temp_byte=0x00,
               block_flag0=0, gap_blocks=False, bad_crc=False,
               no_end=False, header_at=None):
    """写一个真实 VIB3 .bin。

    temp_at: {记录序号: 字节} 只覆盖列出的记录，其余用 temp_byte（默认 0x00）。
    block_flag0: 第 0 个数据块的 flags 原样写入（用于制造 SHORT_READ / SPI_ERR）。
    gap_blocks: 跳过 block_index=1（制造 block_index 缺号）。
    bad_crc: 破坏最后一个数据块的 payload（CRC 必须检出）。
    no_end: 不写结束块（模拟被强杀/掉电）。
    """
    temp_at = temp_at or {}
    header = struct.pack(
        br.FILE_HDR_FMT,
        b"VIB2", 3, 256, 0, 0x000201, 0xDEADBEEF, 8000, 16, 2000, 16, 0,
        123456789, 1750000000000000000, 0x47, 0, b"\x00" * 6, nrec,
        32, 30, 0, 0, 1, 1, 0, b"\x00" * 176, 0)
    header = header[:252] + struct.pack("<I", zlib.crc32(header[:252]) & 0xFFFFFFFF)

    # 名义 8 kHz：物理 125 µs → raw 117/118 交替（与 assert_tmst_gap 同口径）
    tmsts = [(i * 125 * 30 // 32) & 0xFFFF for i in range(nrec)]

    blocks = []
    for bi, start in enumerate(range(0, nrec, WATERMARK)):
        if gap_blocks and bi == 1:
            continue                                   # 缺号：不写这一块
        n = min(WATERMARK, nrec - start)
        payload = b""
        for i in range(start, start + n):
            flags = 0
            if header_at is not None and i == header_at:
                flags |= br.REC_FLAG_HEADER_MSG
            payload += struct.pack(REC_FMT, *FIXED_ACCEL, *FIXED_GYRO,
                                   temp_at.get(i, temp_byte), tmsts[i], flags)
        bflags = block_flag0 if bi == 0 else 0
        braw = struct.pack("<HH IQIHHII".replace(" ", ""),
                           0x0001, n, bi, 123456789 + start * 125000,
                           0, bflags, 0, 0, 0)
        bhdr = braw[:28] + struct.pack(
            "<I", zlib.crc32(payload, zlib.crc32(braw[:28])) & 0xFFFFFFFF)
        blocks.append([bhdr, payload])
    if not blocks:
        raise ValueError("nrec 与水位组合没有数据块")

    if bad_crc:
        # 篡改最后一个数据块的 payload 字节：块内 CRC 必须对不上
        blk = blocks[-1][1]
        blocks[-1][1] = blk[:-1] + bytes([blk[-1] ^ 0xFF])

    body = b"".join(h + p for h, p in blocks)
    if not no_end:
        end = struct.pack("<HH IQIHHII".replace(" ", ""),
                          0xFFFF, 0, len(blocks), 123456789 + nrec * 125000,
                          0, 0, 0, 0, 0)
        end = end[:28] + struct.pack(
            "<I", zlib.crc32(b"", zlib.crc32(end[:28])) & 0xFFFFFFFF)
        body += end
    with open(path, "wb") as f:
        f.write(header + body)
    return path


def csv_temps(csv_path):
    """读回 CSV 里的 temp_c 列（第 9 列，index,segment,t_us,ax..gz,temp_c,flags）。"""
    out = []
    with open(csv_path, "r", encoding="utf-8") as f:
        head = f.readline().strip().split(",")
        ci = head.index("temp_c")
        for line in f:
            row = line.strip().split(",")
            if len(row) > ci:
                out.append(float(row[ci]))
    return out


def case_temperature(tmp):
    print("== A) 温度：uint8 存盘 → 有符号 8 位二补码物理温度 ==")
    # 每条记录用不同字节：0x00 / 0xFF / 0xF9 / 0x15（正）
    cases = [
        (0x00, "0x00"), (0xFF, "0xFF"), (0xF9, "0xF9"), (0x15, "0x15"),
    ]
    temp_at = {i: b for i, (b, _) in enumerate(cases)}
    nrec = WATERMARK * 2

    # 独立复算（不复用解析器）：二补码 + /2.07 + 25
    def expect(b):
        v = b - 256 if b >= 128 else b
        return v / 2.07 + 25.0

    exp = [expect(b) for b, _ in cases]
    check(abs(exp[0] - 25.0) < 1e-12, "独立复算：0x00 → 25.0 ℃（实际 %.4f）" % exp[0])
    check(abs(exp[1] - 24.5169) < 5e-5, "独立复算：0xFF → 24.5169 ℃（实际 %.4f）" % exp[1])
    check(abs(exp[2] - 21.6184) < 5e-5, "独立复算：0xF9 → 21.6184 ℃（实际 %.4f）" % exp[2])
    check(abs(exp[3] - (0x15 / 2.07 + 25.0)) < 1e-12,
          "独立复算：正字节 0x15 不做符号变换（实际 %.4f）" % exp[3])

    path = build_vib3(os.path.join(tmp, "temp.bin"), nrec, temp_at=temp_at)
    csv_path = os.path.join(tmp, "temp.csv")
    st = br.BinReader(path, csv_path=csv_path).run()

    # 前 4 条按字节换算；其余记录 0x00 → 25 ℃
    check(st["ok"], "温度 fixture 自身不得引入异常（ok=True）")
    check(abs(st["temp"]["min"] - min(min(exp), 25.0)) < 5e-4,
          "stats 温度最小值 == 期望（%.4f，实际 %.4f）" % (min(min(exp), 25.0), st["temp"]["min"]))
    check(abs(st["temp"]["max"] - max(max(exp), 25.0)) < 5e-4,
          "stats 温度最大值 == 期望（%.4f，实际 %.4f）" % (max(max(exp), 25.0), st["temp"]["max"]))
    n_neg = sum(1 for b, _ in cases if b >= 128)
    check(n_neg == 2, "本 fixture 含 2 个需二补码还原的负温度字节")

    # 与 CSV 同值：CSV 以 %.3f 落盘，逐样本由 CSV 复算 mean 必须等于 stats mean
    # （容差取 CSV 打印精度 5e-4，而不是 1e-9 —— 比的是「同一口径」，不是打印位数）
    got = csv_temps(csv_path)
    check(len(got) == nrec, "CSV 行数 == 记录数（%d，实际 %d）" % (nrec, len(got)))
    if got:
        csv_mean = sum(got) / len(got)
        check(abs(csv_mean - st["temp"]["mean"]) < 5e-4,
              "CSV 逐样本温度均值 == stats 温度均值（%.4f vs %.4f）"
              % (csv_mean, st["temp"]["mean"]))
        check(all(abs(got[i] - exp[i]) < 5e-4 for i in range(len(cases))),
              "CSV 前 4 行 == 独立复算值（%s）"
              % ["%.4f" % x for x in got[:len(cases)]])
        check(all(abs(v - 25.0) < 1e-9 for v in got[len(cases):]),
              "其余记录（0x00）CSV 温度 == 25.0 ℃")

    # 直接调用共享换算函数（CSV/stats 共用路径）
    check(abs(br.temp_raw_to_c(0xFF) - 24.5169) < 5e-5, "temp_raw_to_c(0xFF) == 24.5169")
    check(abs(br.temp_raw_to_c(0xF9) - 21.6184) < 5e-5, "temp_raw_to_c(0xF9) == 21.6184")
    check(abs(br.temp_raw_to_c(0x00) - 25.0) < 1e-12, "temp_raw_to_c(0x00) == 25.0")

    # 帧布局不变：记录 16 B，且字段仍在偏移 12
    check(br.RECORD_SIZE == 16 and br.RECORD_FMT == "<3h3hBHB",
          "记录布局/CRC 未变（16 B，temp 仍在偏移 12）")


def case_short_read_ok(tmp):
    print("== B1) 短读：信息级 notice，不得单独判异常 ==")
    nrec = WATERMARK + 5                      # 第二个块只有 5 条 < 水位
    path = build_vib3(os.path.join(tmp, "short.bin"), nrec, block_flag0=br.BLOCK_FLAG_SHORT_READ)
    st = br.BinReader(path).run()
    ig = st["integrity"]

    check(ig["short_blocks"] == 1, "short_blocks 计数保留 == 1（实际 %s）" % ig["short_blocks"])
    check(st["ok"], "短读不得单独把健康文件判成异常（ok 应为 True，实际 %s）" % st["ok"])
    check(not any("短读" in a for a in st["anomalies"]),
          "短读不得出现在 anomalies：%s" % st["anomalies"])
    check(any("短读" in n for n in st.get("notices", [])),
          "短读必须出现在 notices（信息级）：%s" % st.get("notices"))
    check(ig["records"] == nrec, "记录数完整（%d，实际 %d）" % (nrec, ig["records"]))
    check(ig["end_block_seen"], "短读文件仍读到结束块")

    # 逐字节独立复算：块 0 有水位条、块 1 有 5 条
    sizes = []
    with open(path, "rb") as f:
        f.seek(br.FILE_HEADER_SIZE)
        while True:
            b = f.read(br.BLOCK_HEADER_SIZE)
            if len(b) < br.BLOCK_HEADER_SIZE:
                break
            bt, n = struct.unpack_from("<HH", b)
            f.seek(n * br.RECORD_SIZE, 1)
            if bt == 0xFFFF:
                break
            sizes.append(n)
    check(sizes == [WATERMARK, 5], "独立按字节复算块内记录数 == [%d, 5]（实际 %s）"
          % (WATERMARK, sizes))


def case_real_errors_still_fail(tmp):
    print("== B2) 真实错误仍必须判异常（不得被短读改动弱化）==")
    base_nrec = 3 * WATERMARK

    # 真实 SPI_ERR（本块 FIFO 健康未知）
    p = build_vib3(os.path.join(tmp, "spierr.bin"), base_nrec, block_flag0=br.BLOCK_FLAG_SPI_ERR)
    st = br.BinReader(p).run()
    check(not st["ok"], "SPI_ERR 块 ⇒ ok=False")
    check(st["integrity"]["spi_err_blocks"] == 1, "spi_err_blocks == 1")
    check(not any("短读" in n for n in st.get("notices", [])),
          "没有短读时不得凭空产生短读 notice")

    # 缺块（block_index 缺号 = 主机侧丢包证据）
    p = build_vib3(os.path.join(tmp, "gapblk.bin"), base_nrec, gap_blocks=True)
    st = br.BinReader(p).run()
    check(not st["ok"], "缺块 ⇒ ok=False")
    check(st["integrity"]["missing_blocks"] >= 1, "missing_blocks ≥ 1（实际 %s）"
          % st["integrity"]["missing_blocks"])

    # 坏 CRC
    p = build_vib3(os.path.join(tmp, "badcrc.bin"), base_nrec, bad_crc=True)
    st = br.BinReader(p).run()
    check(not st["ok"], "坏 CRC ⇒ ok=False")
    check(st["integrity"]["block_crc_errors"] == 1, "block_crc_errors == 1（实际 %s）"
          % st["integrity"]["block_crc_errors"])

    # 未结束（被强杀/掉电）
    p = build_vib3(os.path.join(tmp, "noend.bin"), base_nrec, no_end=True)
    st = br.BinReader(p).run()
    check(not st["ok"], "缺结束块 ⇒ ok=False")
    check(not st["integrity"]["end_block_seen"], "end_block_seen=False")

    # 组合：短读 + 真实 SPI_ERR ⇒ 仍然不允许通过（notice 不能掩盖真错误）
    p = build_vib3(os.path.join(tmp, "short_spierr.bin"), base_nrec,
                   block_flag0=br.BLOCK_FLAG_SHORT_READ | br.BLOCK_FLAG_SPI_ERR)
    st = br.BinReader(p).run()
    check(not st["ok"], "短读 + SPI_ERR 组合 ⇒ ok=False（notice 不掩盖真错误）")
    check(st["integrity"]["short_blocks"] == 1 and st["integrity"]["spi_err_blocks"] == 1,
          "两个计数都被如实保留")


def case_timebase_metadata(tmp):
    print("== C) 时基：机器可读元数据 + 独立参考速率 ==")
    nrec = 2 * WATERMARK
    path = build_vib3(os.path.join(tmp, "tb.bin"), nrec)
    st = br.BinReader(path).run()
    tm = st["timing"]
    tb = tm.get("timebase")

    check(isinstance(tb, dict), "timing.timebase 存在且是对象（实际 %r）" % type(tb))
    if not isinstance(tb, dict):
        return
    check(tb.get("kind") == "chip_nominal_scale",
          "kind == chip_nominal_scale（实际 %r）" % tb.get("kind"))
    check(tb.get("absolute_clock_calibrated") is False,
          "absolute_clock_calibrated is False（不得声称已校准）")
    check(str(tb.get("absolute_clock_calibrated")).lower() != "true",
          "没有任何字段把绝对时钟校准写成 true")
    check(tb.get("source") == "internal", "source == internal（无 CLKIN）")
    check(tb.get("conversion") == "raw_ticks*32/30",
          "conversion == raw_ticks*32/30（实际 %r）" % tb.get("conversion"))
    check(tb.get("raw_delta_window") == [br.TMST_RAW_LO, br.TMST_RAW_HI] == [115, 120],
          "raw 窗口元数据 == [115, 120]（现选用的保守诊断门限不变）")
    check("绝对 8000" in str(tb.get("raw_delta_window_note", "")),
          "note 明确不得声明绝对 8000 Hz 已校准")
    check(tb.get("reference_independent_rate") == "host_rate_hz",
          "独立参考量指向 host_rate_hz")
    check(tb.get("independent_rate_hz") == tm["host_rate_hz"],
          "independent_rate_hz 与 timing.host_rate_hz 一致（%s vs %s）"
          % (tb.get("independent_rate_hz"), tm["host_rate_hz"]))
    check(tm["host_rate_hz"] is not None and abs(tm["host_rate_hz"] - 8000.0) < 200.0,
          "host_rate_hz 被独立给出（%s）" % tm["host_rate_hz"])
    # 芯片标称时基换算速率仍按 raw*32/30（未通过改系数抹平 7911 vs 8000）
    check(abs(tm["measured_odr_hz"] - 8000.0) < 1.0,
          "measured_odr_hz 仍是 raw*32/30 换算（≈8000，实际 %s）" % tm["measured_odr_hz"])
    check(br.TMST_SCALE_NUM == 32 and br.TMST_SCALE_DEN == 30,
          "换算系数未被改动（32/30）")


def main():
    tmp = tempfile.mkdtemp(prefix="hw_bytes_")
    try:
        case_temperature(tmp)
        case_short_read_ok(tmp)
        case_real_errors_still_fail(tmp)
        case_timebase_metadata(tmp)
    finally:
        if FAILS:
            print("\n（保留 fixture 目录供排查：%s）" % tmp)
        else:
            shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print("\nassert_hw_bytes: %d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nassert_hw_bytes: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
