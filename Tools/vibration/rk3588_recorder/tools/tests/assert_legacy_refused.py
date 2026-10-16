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
    1) bin_reader 明确判它不可信（ok=False 且给出「旧假设」anomaly）；
    2) sync_bin_ulog 在**读取 ULog 之前**就以退出码 2 拒绝（dummy ULog 路径即证明）。

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

    if FAILS:
        print("\nassert_legacy_refused: %d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nassert_legacy_refused: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
