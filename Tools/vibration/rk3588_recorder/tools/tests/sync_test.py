#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_test.py — 外置 .bin ↔ X7 native FIFO ULog 时间映射工具的测试

覆盖：
  拟合判据：无事件 / 单/双事件 / missing provenance / nan·invalid / range 外 /
            leverage 离群 / 不合理时钟比 / 高不确定度不宣称 500Hz /
            事件跨外置断裂段被拒
  X7 分段：device=0 拒绝、dt/scale/samples 无效拒绝、回退/重复/缺包真正断段、padding 忽略
  CLI smoke：真实 .bin + native ULog + 真实格式 events JSON，校验：
            映射后的逐样本时刻/单位、--csv-mapped 的 t_px4_us（已知 offset/drift）、
            native reader 是 diagnostic（非 0 退出，不伪装 pyulog 级验收）
只依赖标准库 + 本仓库模块，无硬件。
"""

import json
import math
import os
import struct
import sys
import tempfile
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


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def seg(device_id, first_us, last_us):
    """构造单个 X7 连续段跨度（fit_events 用，packets 是计数）。"""
    return {"device_id": device_id, "first_us": first_us, "last_us": last_us,
            "packets": 1, "samples": 1}


def ext_seg(first_us, last_us):
    """构造外置连续段（fit_events 用）。"""
    return {"segment_id": 0, "first_us": first_us, "last_us": last_us, "samples": 1}


def ev(ext, x7, label="e", provenance="tap", uncertainty_us=100.0):
    return {"external_us": ext, "x7_us": x7, "label": label,
            "provenance": provenance, "uncertainty_us": uncertainty_us}


def write_vib3(path, seconds):
    """写一个最小合法 VIB3 .bin（raw ticks 严格 8 kHz，首样本为 0 相对时间）。"""
    nrec = seconds * 8000
    header = struct.pack(
        br.FILE_HDR_FMT,
        b"VIB2", 3, 256, 0, 0x000201, 0xDEADBEEF, 8000, 16, 2000, 16, 0,
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
            rec = struct.pack("<3h3hBHB", 1000, -2000, 2048, 164, 0, -16, 21, tmst, 0)
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
        ds.fields["x"].append(row.get("x", [0] * 32))
        ds.fields["y"].append(row.get("y", [0] * 32))
        ds.fields["z"].append(row.get("z", [0] * 32))
        ds.count += 1
    return ds


# ---------------------------------------------------------------------------
# 拟合判据
# ---------------------------------------------------------------------------
def test_fit_recovers_offset_and_drift():
    print("== 已知 offset+drift 拟合 + 事件都在段内 ==\n")
    big_offset = 5_000_000.0
    drift_ppm = 100.0
    a = 1.0 + drift_ppm / 1e6
    ext = [i * 1_000_000.0 for i in range(5)]
    events = [ev(x, a * x + big_offset, "e%d" % i) for i, x in enumerate(ext)]
    fit = sync.fit_events(events, [ext_seg(0.0, 6_000_000.0)],
                          seg(0x36001A, 5_000_000.0, 12_000_000.0), 0x36001A,
                          phase_budget_us=1000.0)
    check(fit["status"] == "ok", "状态 ok（实际 %s）" % fit["status"], str(fit["notes"]))
    check(abs(fit["drift_ppm"] - drift_ppm) < 1.0, "drift 恢复 ≈ %.0f ppm" % drift_ppm,
          str(fit["drift_ppm"]))
    check(abs(fit["intercept_us"] - big_offset) < 1.0, "intercept 恢复 ≈ %.0f µs" % big_offset,
          str(fit["intercept_us"]))
    check(fit["residual_rms_us"] < 1e-3, "无噪声残差 RMS ≈ 0")
    check(fit["event_alignment_within_budget"], "事件拟合符合给定容许误差")
    check(not fit["usable_for_500hz"], "用户容许误差不等于实机样本相位校准证据")
    check(fit["phase_suitability"] == "unverified", "实机样本相位仍为 unverified")


def test_fit_insufficient_events():
    print("== 无事件 / 单/双事件 → 缺证据 ==\n")
    f0 = sync.fit_events([], [ext_seg(0, 1000)], seg(1, 0, 1000), 1)
    check(f0["status"] == "insufficient_events", "0 事件 → insufficient_events")
    f1 = sync.fit_events([ev(1, 1)], [ext_seg(0, 1000)], seg(1, 0, 1000), 1)
    check(f1["status"] == "insufficient_events", "1 事件 → insufficient_events")
    f2 = sync.fit_events([ev(1, 1), ev(2, 2)], [ext_seg(0, 1000)], seg(1, 0, 1000), 1)
    check(f2["status"] == "insufficient_events", "2 事件 → insufficient_events")


def test_validate_missing_provenance_nan():
    print("== missing provenance / nan / invalid uncertainty ==\n")
    bad = [
        {"external_us": 1, "x7_us": 1, "label": "a", "uncertainty_us": 10},          # no provenance
        {"external_us": 1, "x7_us": 1, "provenance": "p", "uncertainty_us": 10},      # no label
        {"external_us": float("nan"), "x7_us": 1, "label": "b", "provenance": "p",
         "uncertainty_us": 10},                                                       # nan ext
        {"external_us": 1, "x7_us": 1, "label": "c", "provenance": "p",
         "uncertainty_us": 0.0},                                                      # zero unc
        {"external_us": 1, "x7_us": 1, "label": "d", "provenance": "p",
         "uncertainty_us": -5.0},                                                     # negative unc
    ]
    cleaned, problems = sync.validate_events(bad)
    check(len(cleaned) == 0, "5 个坏事件全部被拒（清理后 0 个）", str(cleaned))
    check(len(problems) == 5, "5 条问题（实际 %d）" % len(problems))
    any_missing = any("provenance" in p for p in problems)
    check(any_missing, "provenance 缺失被点名")


def test_fit_out_of_range():
    print("== 事件落在段/文件之外 → out_of_range ==\n")
    events = [ev(500.0, 5_000_500.0, "e%d" % i) for i in range(4)]
    events[-1] = ev(1500.0, 5_001_500.0, "e3")
    fit = sync.fit_events(events, [ext_seg(0.0, 400.0)], seg(1, 5_000_000, 6_000_000), 1)
    check(fit["status"] == "out_of_range", "external 超界 → out_of_range（实际 %s）"
          % fit["status"])
    # x7 超出段
    events2 = [ev(100.0, 7_000_000.0, "e%d" % i) for i in range(4)]
    fit2 = sync.fit_events(events2, [ext_seg(0.0, 1000.0)], seg(1, 5_000_000, 6_000_000), 1)
    check(fit2["status"] == "out_of_range", "x7 超段 → out_of_range（实际 %s）"
          % fit2["status"])


def test_fit_events_span_external_segments():
    print("== 事件跨外置断裂段 → 拒绝（不跨断裂拼轴）==\n")
    # 两个外置连续段 [0,1000] 与 [2000,3000]（中间断裂）；事件落在两段 → 拒绝
    segments = [ext_seg(0.0, 1000.0), {"segment_id": 1, "first_us": 2000.0,
                                       "last_us": 3000.0, "samples": 1}]
    events = [ev(500.0, 5_000_500.0, "e%d" % i) for i in range(3)]
    events.append(ev(2500.0, 5_002_500.0, "e3"))   # 落在第二段
    fit = sync.fit_events(events, segments, seg(1, 5_000_000, 6_000_000), 1)
    check(fit["status"] == "out_of_range", "跨外置断裂段 → 拒绝（实际 %s）" % fit["status"])


def test_fit_leverage_outlier():
    print("== leverage 离群被稳健拟合剔除，refit 恢复 drift ==\n")
    drift_ppm = 50.0
    a = 1.0 + drift_ppm / 1e6
    base = 8_000_000.0
    ext = [1_000_000.0 * i for i in range(5)]
    x7 = [a * x + base for x in ext]
    ext.append(50_000_000.0)
    x7.append(8_100_000.0)
    events = [ev(x, y, "e%d" % i) for i, (x, y) in enumerate(zip(ext, x7))]
    fit = sync.fit_events(events, [ext_seg(0.0, 60_000_000.0)],
                          seg(1, 7_000_000.0, 20_000_000.0), 1)
    check(fit["status"] == "ok", "稳健拟合后仍 ok（实际 %s）" % fit["status"], str(fit["notes"]))
    check("e5" in fit["outlier_labels"], "杠杆离群 e5 被剔除（实际 %s）" % fit["outlier_labels"])
    check(abs(fit["drift_ppm"] - drift_ppm) < 2.0, "refit 后 drift 恢复 ≈ %.0f ppm" % drift_ppm,
          str(fit["drift_ppm"]))


def test_fit_unreasonable_clock_ratio():
    print("== 不合理时钟比 → 拒绝 ==\n")
    events = [ev(i * 1_000_000.0, 1.5 * i * 1_000_000.0 + 5_000_000, "e%d" % i)
              for i in range(5)]
    fit = sync.fit_events(events, [ext_seg(0.0, 10_000_000.0)],
                          seg(1, 5_000_000.0, 30_000_000.0), 1, max_drift_ppm=1000.0)
    check(fit["status"] == "unreasonable_clock_ratio",
          "50%% drift → unreasonable_clock_ratio（实际 %s）" % fit["status"])


def test_fit_high_uncertainty_not_500hz():
    print("== 高不确定度即使零残差也不宣称 500Hz（默认 unverified）==\n")
    a = 1.0 + 10.0 / 1e6
    base = 5_000_000.0
    events = [ev(i * 1_000_000.0, a * i * 1_000_000.0 + base, "e%d" % i,
                 uncertainty_us=5000.0) for i in range(5)]
    fit = sync.fit_events(events, [ext_seg(0.0, 10_000_000.0)],
                          seg(1, 5_000_000.0, 15_000_000.0), 1)
    check(fit["status"] == "ok", "残差零 → ok")
    check(fit["residual_rms_us"] < 1e-3, "零残差")
    check(not fit["usable_for_500hz"], "默认（无相位预算）不宣称 500Hz（usable=%s）"
          % fit["usable_for_500hz"])
    check(fit["phase_suitability"] == "unverified", "phase_suitability=unverified")
    check(fit["worst_inlier_uncertainty_us"] == 5000.0, "worst inlier 不确定度=5000（不用 median）")

    # 显式预算 100 µs，但 worst=5000 > 100 → exceeds_budget
    fit2 = sync.fit_events(events, [ext_seg(0.0, 10_000_000.0)],
                           seg(1, 5_000_000.0, 15_000_000.0), 1, phase_budget_us=100.0)
    check(not fit2["usable_for_500hz"], "显式预算 100 µs 但 worst=5000 → 不宣称")
    check(fit2["event_alignment_within_budget"] is False, "事件拟合不确定度超过容许误差")
    check(fit2["phase_suitability"] == "unverified", "实机样本相位仍为 unverified")


# ---------------------------------------------------------------------------
# X7 分段
# ---------------------------------------------------------------------------
def test_reconstruct_device0_and_invalid():
    print("== device=0 拒绝 + dt/scale/samples 无效拒绝 ==\n")
    rows = [
        {"timestamp": 1000, "anchor": 1000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 2000, "anchor": 2000, "device_id": 0, "dt": 125.0, "samples": 20},
        {"timestamp": 3000, "anchor": 3000, "device_id": 0x36001A, "dt": 0.0,
         "samples": 20},
        {"timestamp": 4000, "anchor": 4000, "device_id": 0x36001A, "dt": 125.0,
         "scale": 0.0, "samples": 20},
        {"timestamp": 5000, "anchor": 5000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 100},
        {"timestamp": 6000, "anchor": 6000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
    ]
    segments, stats = sync.reconstruct_x7_segments(fifo_dataset(rows))
    check(stats["device0_packets"] == 1, "device=0 计 1 个（实际 %d）" % stats["device0_packets"])
    check(stats["invalid_dt"] == 1, "dt=0 计 1 个")
    check(stats["invalid_scale"] == 1, "scale=0 计 1 个")
    check(stats["invalid_samples"] == 1, "samples=100 计 1 个")
    total_valid = sum(len(s["packets"]) for s in segments)
    check(total_valid == 2, "有效包 2 个（实际 %d）" % total_valid)


def test_reconstruct_breaks():
    print("== 回退/重复/缺包真正断段 ==\n")
    rows = [
        {"timestamp": 1000, "anchor": 100000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 2000, "anchor": 102500, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 3000, "anchor": 122500, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 4000, "anchor": 120000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 4000, "anchor": 120000, "device_id": 0x36001A, "dt": 125.0,
         "samples": 20},
        {"timestamp": 5000, "anchor": 200000, "device_id": 0x99001A, "dt": 125.0,
         "samples": 20},
    ]
    segments, stats = sync.reconstruct_x7_segments(fifo_dataset(rows))
    check(len(segments) == 5, "5 个连续段（device 变化 1 + 缺包 1 + 回退 1 + 重复 1）"
          "（实际 %d）" % len(segments))
    check(stats["breaks"] == 3, "断段计数 3（缺包+回退+重复，实际 %d）" % stats["breaks"])


def test_reconstruct_padding_ignored():
    print("== padding 忽略（只取 samples 有效前缀）==\n")
    x = list(range(5)) + [999] * 27
    rows = [{"timestamp": 1000, "anchor": 1000, "device_id": 0x36001A, "dt": 125.0,
             "samples": 5, "x": x, "y": [0] * 32, "z": [0] * 32}]
    ds = fifo_dataset(rows)
    segments, _ = sync.reconstruct_x7_segments(ds)
    check(len(segments) == 1 and segments[0]["packets"][0]["n"] == 5,
          "每包只取有效前缀 n=5（实际 %s）" % [s["packets"][0]["n"] for s in segments])
    got = list(sync.x7_sample_iter(ds, segments[0], "gyro"))
    check(len(got) == 5, "逐样本只产出 5 个（实际 %d）" % len(got))
    check(all(v[2] != 999 * 0.001 for v in got), "padding 值（999*scale）未泄漏进样本")


# ---------------------------------------------------------------------------
# CLI smoke：真实 bin + native ULog + 真实格式 events
# ---------------------------------------------------------------------------
GYRO_FMT = ("uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;"
            "float dt;float scale;uint8_t samples;"
            "int16_t[32] x;int16_t[32] y;int16_t[32] z;")


def write_native_ulog(path, rows, topic="sensor_gyro_fifo"):
    """写一个 native 读取器能解码的真实格式 ULog（一个 topic，msg_id=0）。"""
    header = b"ULog\x01\x125\x01" + struct.pack("<Q", 0)
    out = bytearray(header)

    def add(mtype, payload):
        out.extend(struct.pack("<HB", len(payload), mtype) + payload)

    add(0x46, (topic + ":" + GYRO_FMT).encode())            # MSG_FORMAT
    add(0x41, struct.pack("<BH", 0, 0) + topic.encode())    # ADD_LOGGED
    for r in rows:
        ts, anchor, devid, dt, scale, n, x, y, z = r
        body = struct.pack("<QQIffB", ts, anchor, devid, dt, scale, n)
        body += struct.pack("<32h", *x) + struct.pack("<32h", *y) + struct.pack("<32h", *z)
        add(0x44, struct.pack("<H", 0) + body)              # MSG_DATA
    with open(path, "wb") as f:
        f.write(bytes(out))
    return path


def test_cli_smoke(tmpdir):
    print("== CLI smoke：真实 bin + native ULog + events，校验映射时刻/单位/diagnostic ==\n")
    devid = 0x36001A
    binpath = write_vib3(os.path.join(tmpdir, "ext.bin"), 2)
    # X7 native ULog：500 包，dt=125 µs，samples=32，锚点从 10,000,000 起
    rows = []
    for k in range(500):
        anchor = 10_000_000 + k * 32 * 125
        ts = anchor + 2500
        x = [i for i in range(32)]
        y = [i + 1 for i in range(32)]
        z = [i + 2 for i in range(32)]
        rows.append((ts, anchor, devid, 125.0, 0.001, 32, x, y, z))
    ulogpath = write_native_ulog(os.path.join(tmpdir, "x7.ulg"), rows)
    # 事件：drift 100 ppm，offset 10,000,000 µs
    a = 1.0 + 100.0 / 1e6
    events = {"events": [
        {"external_us": 100000.0, "x7_us": a * 100000.0 + 10_000_000.0,
         "label": "e0", "provenance": "manual-tap", "uncertainty_us": 50.0},
        {"external_us": 500000.0, "x7_us": a * 500000.0 + 10_000_000.0,
         "label": "e1", "provenance": "manual-tap", "uncertainty_us": 50.0},
        {"external_us": 1000000.0, "x7_us": a * 1000000.0 + 10_000_000.0,
         "label": "e2", "provenance": "manual-tap", "uncertainty_us": 50.0},
        {"external_us": 1500000.0, "x7_us": a * 1500000.0 + 10_000_000.0,
         "label": "e3", "provenance": "manual-tap", "uncertainty_us": 50.0},
    ]}
    eventspath = os.path.join(tmpdir, "events.json")
    with open(eventspath, "w", encoding="utf-8") as f:
        json.dump(events, f)

    csv_ext = os.path.join(tmpdir, "ext.csv")
    csv_x7 = os.path.join(tmpdir, "x7.csv")
    csv_mapped = os.path.join(tmpdir, "mapped.csv")
    json_out = os.path.join(tmpdir, "out.json")

    rc = sync.main([
        "--bin", binpath, "--ulog", ulogpath, "--events", eventspath,
        "--axis", "gyro", "--reader", "native", "--device", str(devid),
        "--csv-ext", csv_ext, "--csv-x7", csv_x7, "--csv-mapped", csv_mapped,
        "--json", json_out,
    ])
    # native reader 是 diagnostic fallback：即使 fit ok 也非 0 退出（不伪装 pyulog 级验收）
    check(rc == 2, "native reader → 退出码 2（diagnostic，不伪装验收；实际 %d）" % rc)

    with open(json_out, "r", encoding="utf-8") as f:
        payload = json.load(f)
    check(payload["mapping"]["status"] == "ok", "mapping.status ok（实际 %s）"
          % payload["mapping"]["status"])
    check(payload["ulog"]["reader_grade"] == "diagnostic", "reader_grade=diagnostic")
    check(abs(payload["mapping"]["drift_ppm"] - 100.0) < 2.0,
          "drift ≈ 100 ppm（实际 %.3f）" % payload["mapping"]["drift_ppm"])

    # 外置 CSV：header 含 segment/t_us；首样本 t_us=0
    with open(csv_ext, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        first = f.readline().strip().split(",")
    check(header[:3] == ["index", "segment", "t_us"], "外置 CSV 头含 index,segment,t_us",
          str(header[:3]))
    check(first[1] == "0" and abs(float(first[2])) < 1e-6,
          "外置首样本 t_us=0（实际 %s）" % first[2])

    # X7 CSV：header 单位 + 首样本时刻 = anchor - (N-1)*dt + 值 = raw*scale
    with open(csv_x7, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        row0 = f.readline().strip().split(",")
    check("t_us" in header and "x_rad_s" in header, "X7 CSV 头含 t_us 与 x_rad_s（rad/s）",
          str(header))
    exp_first_t = 10_000_000 - 31 * 125.0
    check(abs(float(row0[2]) - exp_first_t) < 1e-6,
          "X7 首样本 t_us = anchor-(N-1)*dt = %.0f（实际 %s）" % (exp_first_t, row0[2]))
    check(abs(float(row0[4]) - 0.0) < 1e-9, "X7 首样本 x = raw[0]*scale = 0.0（实际 %s）"
          % row0[4])
    check(int(row0[3]) == devid, "X7 CSV device_id == 选定 device（实际 %s）" % row0[3])

    # 映射 CSV：header 含 t_px4_us；首样本 t_px4 ≈ intercept（10,000,000）
    with open(csv_mapped, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        row0 = f.readline().strip().split(",")   # index 0
        f.readline()                              # index 1
        row2 = f.readline().strip().split(",")   # index 2
    check("t_px4_us" in header, "映射 CSV 头含 t_px4_us（实际 %s）" % header[:4])
    inter = payload["mapping"]["intercept_us"]
    slope = payload["mapping"]["slope"]
    check(abs(float(row0[3]) - inter) < 1.0,
          "映射首样本 t_px4 ≈ intercept %.0f（实际 %s）" % (inter, row0[3]))
    # index=2 的外置 t_us ≈ 2*125=250 µs；t_px4 ≈ slope*250 + intercept
    t_ext2 = float(row2[2])
    exp_px4 = slope * t_ext2 + inter
    check(abs(float(row2[3]) - exp_px4) < 1e-6,
          "映射逐样本 t_px4 = slope*t_ext + intercept（实际 %.3f vs %.3f）"
          % (float(row2[3]), exp_px4))


def main():
    test_fit_recovers_offset_and_drift()
    test_fit_insufficient_events()
    test_validate_missing_provenance_nan()
    test_fit_out_of_range()
    test_fit_events_span_external_segments()
    test_fit_leverage_outlier()
    test_fit_unreasonable_clock_ratio()
    test_fit_high_uncertainty_not_500hz()
    test_reconstruct_device0_and_invalid()
    test_reconstruct_breaks()
    test_reconstruct_padding_ignored()
    with tempfile.TemporaryDirectory() as tmp:
        test_cli_smoke(tmp)

    if FAILS:
        print("\n结果：%d 项失败 ❌" % len(FAILS))
        for f in FAILS:
            print("  - %s" % f)
        return 1
    print("\nsync_test: 全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
