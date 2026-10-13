#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_test.py — 外置 .bin ↔ X7 native FIFO ULog 时间映射工具的测试

覆盖（fix 2 的测试需求）：
  1) 已知 offset+drift 拟合被正确恢复（绝对 offset 大没问题）
  2) 噪声 / 离群事件被标记，且不污染整体拟合
  3) 缺事件（少样本）仍能拟合
  4) X7 时间轴按 device_id 分段，timestamp_sample 是最后样本锚点，dt 用 µs
  5) 外置 VIB3 时间轴来源读取（raw ticks → µs 由文件头 scale 给出）

只依赖标准库 + 本仓库模块，无硬件。
"""

import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)                       # rk3588_recorder/tools
sys.path.insert(0, TOOLS)
_PARENT = os.path.dirname(os.path.dirname(TOOLS))   # Tools/vibration
sys.path.insert(0, _PARENT)

import bin_reader as br          # noqa: E402
import sync_bin_ulog as sync     # noqa: E402
import ulog_reader               # noqa: E402

FAILS = []


def check(cond, label, extra=""):
    if cond:
        print("  ✓ %s" % label)
    else:
        print("  ✗ %s %s" % (label, extra))
        FAILS.append(label)


def write_vib3(path, seconds):
    """写一个最小合法 VIB3 .bin（raw ticks 严格 8 kHz，首样本为 0 相对时间）。"""
    nrec = seconds * 8000
    header = struct.pack(
        br.FILE_HDR_FMT,
        b"VIB2", 3, 256, 0, 0x000200, 0xDEADBEEF, 8000, 16, 2000, 16, 0,
        123456789, 1750000000000000000, 0x47, 0, b"\x00" * 6, nrec,
        32, 30, 0, 0, 1, 1, 0, b"\x00" * 176, 0)
    crc = zlib.crc32(header[:252]) & 0xFFFFFFFF
    header = header[:252] + struct.pack("<I", crc)

    blocks = []
    for start in range(0, nrec, br.SAMPLES_PER_BLOCK):
        n = min(br.SAMPLES_PER_BLOCK, nrec - start)
        payload = b""
        for i in range(start, start + n):
            tmst = (i * 125 * 30 // 32) & 0xFFFF
            rec = struct.pack("<3h3hBHB", 1000, -2000, 2048, 164, 0, -16,
                              21, tmst, 0)
            payload += rec
        braw = struct.pack("<HH IQIHHII".replace(" ", ""),
                           0x0001, n, start // br.SAMPLES_PER_BLOCK,
                           123456789 + start * 125000, 0, 0, 0, 0, 0)
        bhdr = braw[:28] + struct.pack("<I", zlib.crc32(payload, zlib.crc32(braw[:28])) & 0xFFFFFFFF)
        blocks.append(bhdr + payload)
    end = struct.pack("<HH IQIHHII".replace(" ", ""),
                      0xFFFF, 0, len(blocks), 123456789 + nrec * 125000, 0, 0, 0, 0, 0)
    end = end[:28] + struct.pack("<I", zlib.crc32(b"", zlib.crc32(end[:28])) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(header + b"".join(blocks) + end)
    return path


def fifo_dataset(rows, name="sensor_gyro_fifo", multi_id=0):
    ds = ulog_reader.Dataset(name, multi_id, 1, {})
    for key in ("timestamp", "timestamp_sample", "device_id", "dt", "scale",
                "samples", "x", "y", "z"):
        ds.fields[key] = []
    for row in rows:
        ds.fields["timestamp"].append(row["timestamp"])
        ds.fields["timestamp_sample"].append(row["anchor"])
        ds.fields["device_id"].append(row["device_id"])
        ds.fields["dt"].append(row["dt"])
        ds.fields["scale"].append(row.get("scale", 0.001))
        ds.fields["samples"].append(row["samples"])
        ds.fields["x"].append([0] * 32)
        ds.fields["y"].append([0] * 32)
        ds.fields["z"].append([0] * 32)
        ds.count += 1
    return ds


def test_fit_recovers_offset_and_drift():
    print("== 已知 offset+drift 拟合 ==\n")
    big_offset = 5_000_000.0        # 外置 0 起点 vs X7 绝对 µs，很大是正常的
    drift_ppm = 100.0               # X7 时钟相对外置快 100 ppm
    a = 1.0 + drift_ppm / 1e6
    ext = [i * 1_000_000.0 for i in range(6)]      # 0..5 s，每 1 s 一个事件
    events = [{"external_us": x, "x7_us": a * x + big_offset, "label": "e%d" % i}
              for i, x in enumerate(ext)]
    fit = sync.fit_events(events)
    check(fit["drift_ppm"] is not None and abs(fit["drift_ppm"] - drift_ppm) < 1.0,
          "drift 恢复 ≈ %.0f ppm（实际 %.3f）" % (drift_ppm, fit["drift_ppm"]))
    check(abs(fit["intercept_us"] - big_offset) < 1.0,
          "intercept（外置 t=0 处的 X7 时刻）恢复 ≈ %.0f µs（实际 %.1f）"
          % (big_offset, fit["intercept_us"]))
    check(fit["residual_rms_us"] < 1e-6, "无噪声时残差 RMS ≈ 0")
    check(not fit["outlier_labels"], "无离群")


def test_fit_large_offset_ok():
    print("== 绝对 offset 大（50 s）不作为同步判据 ==\n")
    drift_ppm = -50.0
    a = 1.0 + drift_ppm / 1e6
    big = 50_000_000.0
    events = [{"external_us": x, "x7_us": a * x + big} for x in (0.0, 1e6, 2e6, 3e6)]
    fit = sync.fit_events(events)
    check(abs(fit["drift_ppm"] - drift_ppm) < 1.0,
          "50 s offset 下 drift 仍正确（实际 %.3f ppm）" % fit["drift_ppm"])
    check(any("offset" in n for n in fit["notes"]), "报告注明 offset 大小不作为判据")


def test_fit_noise_and_outlier():
    print("== 噪声 / 离群事件 ==\n")
    drift_ppm = 20.0
    a = 1.0 + drift_ppm / 1e6
    base = 8_000_000.0
    ext = [i * 1_000_000.0 for i in range(8)]
    x7 = [a * x + base for x in ext]
    # 加入小噪声 + 一个 50 ms 离群
    x7[2] += 300.0
    x7[5] += 50_000.0
    events = [{"external_us": x, "x7_us": y, "label": "e%d" % i}
              for i, (x, y) in enumerate(zip(ext, x7))]
    fit = sync.fit_events(events)
    check("e5" in fit["outlier_labels"], "50 ms 离群事件被标记（实际 %s）"
          % fit["outlier_labels"])
    check(fit["residual_rms_us"] > 1000.0, "离群使残差 RMS 增大（实际 %.1f µs）"
          % fit["residual_rms_us"])


def test_fit_missing_event():
    print("== 缺事件仍能拟合 ==\n")
    drift_ppm = 30.0
    a = 1.0 + drift_ppm / 1e6
    base = 3_000_000.0
    events = [{"external_us": x, "x7_us": a * x + base} for x in (0.0, 2e6, 5e6)]
    fit = sync.fit_events(events)
    check(abs(fit["drift_ppm"] - drift_ppm) < 1.0,
          "3 个事件（含缺失段）drift 恢复（实际 %.3f ppm）" % fit["drift_ppm"])


def test_reconstruct_segments():
    print("== X7 时间轴按 device_id 分段 + 最后样本锚点还原 ==\n")
    # dt=125 µs，每包 20 样本；锚点是「最后样本」，首样本 = anchor - 19*125
    rows = [
        {"timestamp": 1_000_250, "anchor": 1_000_000, "device_id": 0x36001A,
         "dt": 125.0, "samples": 20},
        {"timestamp": 1_002_750, "anchor": 1_002_500, "device_id": 0x36001A,
         "dt": 125.0, "samples": 20},
        {"timestamp": 5_000_250, "anchor": 5_000_000, "device_id": 0x99001A,
         "dt": 125.0, "samples": 20},
    ]
    segs = sync.reconstruct_x7_segments(fifo_dataset(rows))
    check(len(segs) == 2, "两个 device_id → 两段")
    first0 = segs[0]["packets"][0]["first_us"]
    check(first0 == 1_000_000 - 19 * 125, "首样本 = anchor - (N-1)*dt（实际 %.0f）" % first0)
    spans = sync.x7_timeline_span(segs)
    check(len(spans) == 2, "跨度按 device_id 分开（跨器件不假合并）")


def test_read_external_axis(tmpdir):
    print("== 外置 VIB3 时间轴读取 ==\n")
    path = write_vib3(os.path.join(tmpdir, "ext.bin"), 2)
    ext = sync.read_external_axis(path)
    check(ext["ticks_to_us_num"] == 32 and ext["ticks_to_us_den"] == 30,
          "外置 scale=32/30 来自文件头")
    check(abs(ext["duration_s"] - (2 * 8000 - 1) * 125 / 1e6) < 1e-3,
          "外置时长 ≈ (n-1)*125 µs（实际 %.6f s）" % ext["duration_s"])


def main():
    import tempfile
    test_fit_recovers_offset_and_drift()
    test_fit_large_offset_ok()
    test_fit_noise_and_outlier()
    test_fit_missing_event()
    test_reconstruct_segments()
    with tempfile.TemporaryDirectory() as tmp:
        test_read_external_axis(tmp)

    if FAILS:
        print("\n结果：%d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nsync_test: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
