#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
assert_legacy_refused.py — 反例：legacy v2（raw=µs 旧假设）不得被同步工具当可信时间基

背景（真实缺陷反例）：
  v2 文件头没有 raw ticks→µs 换算元数据，`bin_reader` 只能按「raw=µs」处理，并把它
  记为 **anomaly（不是 error）**。而 `sync_bin_ulog.main()` 原先只检查 `ext["errors"]`，
  于是 legacy v2 会被**接受**并基于明确不可信的物理时间轴产出 offset/drift/逐样本映射
  —— 正是「旧 VIB2 不得冒充可信物理时间」所禁止的。

  本测试构造真实字节的 v2 .bin（结构合法、CRC 正确、无 structural error），断言：
    1) bin_reader 明确判它不可信（ok=False 且给出「旧假设」anomaly），且 timing.timebase
       的 source="unknown" / kind="legacy_unknown" —— legacy 元数据是当年的 reserved
       （读到 0 或任意残留字节都不是时钟源元数据），不得冒充 clkin（外部钟）、internal
       （内部钟）或 chip_nominal_scale（标称标定）；**reserved 字节非 0 时也不得摇身
       变成 clkin**（这正是「tmst_source 非 0 一律标 clkin」的缺陷反例）；
    2) sync_bin_ulog 在**读取 ULog 之前**就以退出码 2 拒绝（dummy ULog 路径即证明）；
    3) 正向对照：v3 源 0 → internal/chip_nominal_scale；v3 + tmst_source=1（CLKIN）
       仍按原行为被拒绝且标为 clkin。

用法: python3 assert_legacy_refused.py   （退出码 0=通过）
"""
import os
import struct
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))       # tools/
sys.path.insert(0, HERE)                        # tools/tests/
import bin_reader as br                          # noqa: E402
import sync_bin_ulog as sb                       # noqa: E402

FAILS = []


def check(cond, label):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s" % label)
        FAILS.append(label)


REC_FMT = "<3h3hBHB"


def nominal_ticks(i):
    return i * 125 * 30 // 32


def write_vib2(path, nrec):
    """真实 v2 .bin：format_version=2，64..75 处的换算元数据全 0（v2 当年是 reserved2）。"""
    header = struct.pack(
        br.FILE_HDR_FMT,
        b"VIB2", 2, 256, 0, 0x000200, 0xDEADBEEF, 8000, 16, 2000, 16, 0,
        123456789, 1750000000000000000, 0x47, 0, b"\x00" * 6, nrec,
        0, 0, 0, 0, 0, 0, 0, b"\x00" * 176, 0)     # scale/source/res/unit/endian 全 0
    header = header[:252] + struct.pack("<I", zlib.crc32(header[:252]) & 0xFFFFFFFF)

    blocks = []
    for start in range(0, nrec, br.SAMPLES_PER_BLOCK):
        n = min(br.SAMPLES_PER_BLOCK, nrec - start)
        payload = b"".join(
            struct.pack(REC_FMT, 1000, -2000, 2048, 164, 0, -16, 21,
                        nominal_ticks(i) & 0xFFFF, 0)
            for i in range(start, start + n))
        braw = struct.pack("<HHIQIHHII", 0x0001, n, start // br.SAMPLES_PER_BLOCK,
                           123456789 + start * 125000, 0, 0, 0, 0, 0)
        bhdr = braw[:28] + struct.pack(
            "<I", zlib.crc32(payload, zlib.crc32(braw[:28])) & 0xFFFFFFFF)
        blocks.append(bhdr + payload)

    end = struct.pack("<HHIQIHHII", 0xFFFF, 0, len(blocks),
                      123456789 + nrec * 125000, 0, 0, 0, 0, 0)
    end = end[:28] + struct.pack("<I", zlib.crc32(b"", zlib.crc32(end[:28])) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(header + b"".join(blocks) + end)
    return path


def main():
    tmp = tempfile.mkdtemp(prefix="legacy_refuse_")
    path = write_vib2(os.path.join(tmp, "legacy_v2.bin"), 2000)

    print("== v2 文件必须被标为「时间基不可信」==")
    st = br.BinReader(path).run()
    check(st["header"]["format_version"] == 2,
          "解析到 format_version=2（实际 %s）" % st["header"]["format_version"])
    check(not st["errors"],
          "无结构性错误（否则下面的拒绝可能只是结构错误导致的假通过）：%s" % st["errors"])
    check(not st["ok"], "整体判定 not ok（旧假设不得算健康）")
    joined = " ".join(st["anomalies"])
    check("旧假设" in joined or "不可信" in joined,
          "anomaly 明确标注「旧假设/物理时间轴不可信」")

    print("== legacy v2 的时基标签不得冒充 clkin / internal / 标称标定 ==")
    tb = st["timing"]["timebase"]
    check(tb["source"] == "unknown" and tb["kind"] == "legacy_unknown",
          "source=unknown / kind=legacy_unknown（实际 %s/%s）" % (tb["source"], tb["kind"]))
    check(tb["source"] not in ("internal", "clkin", "legacy_unknown"),
          "缺源元数据时 source 不得复用 tmst_source 取值或旧 kind 名（实际 %s）"
          % tb["source"])
    check(tb["source"] != "clkin", "legacy 不得被称作 clkin（外部钟）")
    check(tb["source"] != "internal", "legacy 不得被称作 internal（内部钟）")
    check(tb["kind"] != "chip_nominal_scale",
          "legacy 不得被称作 chip_nominal_scale（标称标定）")
    check(tb["absolute_clock_calibrated"] is False,
          "legacy 不得声称绝对时钟已校准")
    check(tb.get("metadata_present") is False and tb.get("raw_delta_window") is None,
          "legacy 明确无换算元数据（metadata_present=False、tick 窗口不适用）")

    print("== legacy v2 的 reserved 字节非 0（残留）也不得被标成 clkin ==")
    noisy = os.path.join(tmp, "legacy_v2_reserved_nonzero.bin")
    d2 = bytearray(open(path, "rb").read())
    d2[68] = 1                                  # v2 的 64..75 是 reserved，不是时钟源元数据
    d2[252:256] = struct.pack("<I", zlib.crc32(bytes(d2[:252])) & 0xFFFFFFFF)
    open(noisy, "wb").write(bytes(d2))
    stn = br.BinReader(noisy).run()
    tbn = stn["timing"]["timebase"]
    check(tbn["source"] == "unknown" and tbn["kind"] == "legacy_unknown",
          "reserved=1（非 0 残留）仍 source=unknown / kind=legacy_unknown（实际 %s/%s）"
          % (tbn["source"], tbn["kind"]))
    check(tbn["source"] != "clkin",
          "不得因 reserved 残留字节非 0 就标 clkin（旧缺陷反例）")
    check(not stn["ok"], "reserved 残留非 0 也仍 not ok（旧假设不得算健康）")
    check(tbn.get("metadata_present") is False,
          "仍判无换算元数据（metadata_present=False）")
    rcn = sb.main(["--bin", noisy,
                   "--ulog", os.path.join(tmp, "DOES_NOT_EXIST.ulg"),
                   "--events", os.path.join(tmp, "DOES_NOT_EXIST.json")])
    check(rcn == 2, "同步门禁对 reserved 非 0 的 legacy v2 仍返回 2（实际 %s）" % rcn)

    print("== sync_bin_ulog 必须在读 ULog 之前拒绝 ==")
    rc = sb.main(["--bin", path,
                  "--ulog", os.path.join(tmp, "DOES_NOT_EXIST.ulg"),
                  "--events", os.path.join(tmp, "DOES_NOT_EXIST.json")])
    check(rc == 2, "退出码 2（实际 %s）" % rc)

    print("== v3（有换算元数据）不得被同一门禁拦下 ==")
    # 用同一 builder 但写上 scale 元数据：应通过时间基门禁，继续走到 ULog 加载。
    # ulog_reader 对不存在的 ULog 会抛异常（既有行为），故以「是否越过了门禁」为断言，
    # 而不是以返回码为准——门禁只在旧格式情形返回 2。
    v3 = os.path.join(tmp, "v3.bin")
    data = bytearray(open(path, "rb").read())
    data[4:6] = struct.pack("<H", 3)
    struct.pack_into("<HH", data, 64, 32, 30)          # tmst_scale_num/den
    data[252:256] = struct.pack("<I", zlib.crc32(bytes(data[:252])) & 0xFFFFFFFF)
    open(v3, "wb").write(bytes(data))

    passed_gate = False
    try:
        rc3 = sb.main(["--bin", v3,
                       "--ulog", os.path.join(tmp, "DOES_NOT_EXIST.ulg"),
                       "--events", os.path.join(tmp, "DOES_NOT_EXIST.json")])
        passed_gate = (rc3 != 2)
    except Exception:
        passed_gate = True             # 已进入 ULog 加载 ⇒ 时间基门禁放行
    check(passed_gate, "v3 越过时间基门禁（门禁只拦 legacy v2）")

    # 正向对照：v3 源 0 才是 internal + chip_nominal_scale（legacy 的 unknown 不是默认值）
    st3 = br.BinReader(v3).run()
    tb3 = st3["timing"]["timebase"]
    check(tb3["source"] == "internal" and tb3["kind"] == "chip_nominal_scale",
          "v3 源 0 → internal / chip_nominal_scale（实际 %s/%s）"
          % (tb3["source"], tb3["kind"]))
    check(tb3.get("metadata_present") is True
          and tb3.get("raw_delta_window") == [br.TMST_RAW_LO, br.TMST_RAW_HI],
          "v3 元数据齐全（metadata_present=True、tick 窗口适用）")

    print("== v3 + tmst_source=1（CLKIN）：仍须被拒绝，且不得冒充 internal ==")
    clk = os.path.join(tmp, "v3_clkin.bin")
    data = bytearray(open(v3, "rb").read())
    data[68] = 1                                        # tmst_source = 1（CLKIN）
    data[252:256] = struct.pack("<I", zlib.crc32(bytes(data[:252])) & 0xFFFFFFFF)
    open(clk, "wb").write(bytes(data))
    stc = br.BinReader(clk).run()
    check(not stc["ok"], "CLKIN v3 ⇒ not ok（保留原「拒绝不支持的 clkin」行为）")
    check(any("tmst_source" in e for e in stc["errors"]),
          "errors 明确给出 tmst_source 不受支持（实际 %s）" % stc["errors"])
    check(stc["timing"]["timebase"]["source"] == "clkin",
          "标为 clkin（不得冒充 internal；实际 %s）"
          % stc["timing"]["timebase"]["source"])

    if FAILS:
        print("\nassert_legacy_refused: %d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nassert_legacy_refused: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
