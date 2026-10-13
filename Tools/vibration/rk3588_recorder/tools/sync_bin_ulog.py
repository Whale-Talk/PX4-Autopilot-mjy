#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_bin_ulog.py — 外置 .bin（RK3588 直接采集）与 X7 native FIFO ULog 的时间映射

目的：把外置原始记录（raw ticks → 物理 µs）的样本时间轴，映射到 X7 原生
sensor_accel_fifo / sensor_gyro_fifo 的样本时间轴，输出 offset + drift(ppm) 与
校正残差。**绝对 offset 可以很大**——外置时间轴从 0 起，X7 时间轴是绝对 µs，
两者相差整个采集起点，验收只看校正后残差，不看 offset<10ms，也不看谱峰一致。

关键口径（与振动工程读取契约一致，见 Tools/vibration/vibration_analysis.py）：
  * X7 原生 FIFO 的 ``timestamp_sample`` 是**本包最后一条样本**的锚点，``dt`` 单位
    是 µs（不硬编码 125），还原 t_i = T - (N-1-i)*dt。
  * X7 的 timestamp 已经是 µs，**不要再乘 32/30**（那是外置 ICM raw ticks 的换算）。
  * device_id / 实例 / 缺包变化分段；每包按 ``samples`` 有效前缀，每包独立 ``scale``。
  * 外置 raw ticks → µs = raw * 32/30（Rev 1.6 §12.7），由 bin 文件头元数据给出。

边界与不宣称：
  * 无物理 GPIO/FSYNC，无硬件同步完成宣称。本工具只做软件层面的时间轴对齐。
  * 互相关仅作粗估（且含机械滤波群延迟），本工具不把互相关当同步判据。
  * 旧 200 Hz 日志不冒充高率：只接受 native FIFO 主题。

匹配事件输入（--events events.json）是同步的**唯一真源**，格式：
  {"events": [{"external_us": 123.4, "x7_us": 987654.3,
               "label": "tap1", "uncertainty_us": 500.0}, ...]}
  external_us 以外置 .bin 首样本为 0 的相对物理 µs；x7_us 为 X7 绝对 µs。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# 外置工具目录（rk3588_recorder/tools）与父振动工具目录（Tools/vibration）
sys.path.insert(0, HERE)
_VIBDIR = os.path.dirname(HERE)          # rk3588_recorder
_PARENT = os.path.dirname(_VIBDIR)       # Tools/vibration
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

import bin_reader as br          # noqa: E402  (本目录)
import ulog_reader               # noqa: E402  (父目录)
import vibration_analysis as va  # noqa: E402  (父目录)


def read_external_axis(binpath):
    """读取外置 .bin 的物理时间轴来源/跨度（首样本为 0 的相对 µs）。"""
    reader = br.BinReader(binpath)
    stats = reader.run()
    timing = stats["timing"]
    header = stats["header"]
    return {
        "path": binpath,
        "records": stats["integrity"]["records"],
        "duration_s": timing["duration_s"],
        "t_end_us": round(timing["duration_s"] * 1e6, 1),
        "ticks_to_us_num": header.get("tmst_scale_num"),
        "ticks_to_us_den": header.get("tmst_scale_den"),
        "tmst_source": header.get("tmst_source"),
        "measured_odr_hz": timing["measured_odr_hz"],
        "ok": stats["ok"],
    }


def reconstruct_x7_segments(dataset):
    """按读取契约还原 X7 native FIFO 每包首样本时间。

    t_i = T - (N-1-i)*dt，T = timestamp_sample（µs，最后样本锚点）。
    返回按 device_id 分段的列表，每段含每包 (anchor_us, n, dt_us, first_us, last_us)。
    缺包（锚点跳变远大于 N*dt）在段内被标记为 gap。
    """
    segments = []
    prev_device = None
    for k in range(dataset.count):
        anchor = int(dataset.get("timestamp_sample")[k])
        dt = float(dataset.get("dt")[k])
        n = int(dataset.get("samples")[k])
        device = int(dataset.get("device_id")[k])
        valid = n if 1 <= n <= va.FIFO_SAMPLES_MAX else 0
        first = anchor - (valid - 1) * dt if valid else anchor

        if device != prev_device:
            segments.append({"device_id": device, "packets": [], "gap_count": 0})
            prev_device = device
        seg = segments[-1]
        seg["packets"].append({
            "anchor_us": anchor, "n": valid, "dt_us": dt,
            "first_us": first, "last_us": anchor,
        })
        # 缺包：相邻锚点跳变远大于本包样本覆盖的时长（保守阈值 2×）
        if len(seg["packets"]) >= 2 and dt > 0 and valid:
            prev = seg["packets"][-2]
            expected_span = valid * dt
            jump = anchor - prev["anchor_us"]
            if jump > 2 * expected_span:
                seg["gap_count"] += 1
    return segments


def x7_timeline_span(segments):
    """X7 时间轴覆盖范围（按 device_id 分段，避免跨器件假跨度）。"""
    spans = []
    for seg in segments:
        pk = seg["packets"]
        if not pk:
            continue
        spans.append({
            "device_id": seg["device_id"],
            "first_us": min(p["first_us"] for p in pk),
            "last_us": max(p["last_us"] for p in pk),
            "packets": len(pk),
            "gap_count": seg["gap_count"],
        })
    return spans


def fit_linear(ext_us, x7_us):
    """最小二乘拟合 x7 = a*ext + b。返回 (a, b) 或 None（点不足/退化解）。"""
    n = len(ext_us)
    if n < 2:
        return None
    mx = sum(ext_us) / n
    my = sum(x7_us) / n
    sxx = sum((x - mx) ** 2 for x in ext_us)
    sxy = sum((x - mx) * (y - my) for x, y in zip(ext_us, x7_us))
    if sxx <= 0:
        return None
    a = sxy / sxx
    b = my - a * mx
    return a, b


def fit_events(events):
    """从匹配事件拟合 offset+drift，计算残差并识别离群。

    events: list of dict(external_us, x7_us, label?, uncertainty_us?)
    返回 dict: offset_us, drift_ppm, slope, intercept_us, residual_rms_us,
    residual_us (per event), outlier_labels, notes, n_events。
    """
    ext = [float(e["external_us"]) for e in events]
    x7 = [float(e["x7_us"]) for e in events]
    labels = [e.get("label") or "#%d" % (i + 1) for i, e in enumerate(events)]
    out = {
        "n_events": len(ext),
        "offset_us": None, "drift_ppm": None, "slope": None,
        "intercept_us": None, "residual_rms_us": None,
        "residual_us": [], "outlier_labels": [], "notes": [],
    }

    if len(ext) < 1:
        out["notes"].append("无匹配事件：无法拟合。")
        return out

    if len(ext) == 1:
        # 只有 1 个事件：只能给 offset，不给 drift
        out["offset_us"] = x7[0] - ext[0]
        out["residual_us"] = [0.0]
        out["notes"].append("仅 1 个匹配事件：只能给出 offset，无法估计 drift。")
        return out

    fit = fit_linear(ext, x7)
    if fit is None:
        out["notes"].append("匹配事件时间轴退化解（external_us 全同）：无法拟合。")
        return out

    a, b = fit
    resid = [y - (a * x + b) for x, y in zip(ext, x7)]
    rms = (sum(r * r for r in resid) / len(resid)) ** 0.5
    # 离群识别：把「最大残差」与「其余点的 RMS」比较，避免离群自身把 3×RMS 抬高。
    # 小噪声（亚毫秒）不被误判；明显离群（几十 ms）被标记。
    outliers = []
    if len(resid) >= 3:
        order = sorted(range(len(resid)), key=lambda i: abs(resid[i]), reverse=True)
        worst_i = order[0]
        others = [resid[i] for i in order[1:]]
        rms_others = (sum(r * r for r in others) / len(others)) ** 0.5
        threshold = max(3.0 * rms_others, 1000.0)
        if abs(resid[worst_i]) > threshold:
            outliers.append(labels[worst_i])

    out.update({
        "slope": a,
        "drift_ppm": (a - 1.0) * 1e6,
        "intercept_us": b,
        "offset_us": (sum(y - x for x, y in zip(ext, x7)) / len(ext)),
        "residual_rms_us": rms,
        "residual_us": [round(r, 3) for r in resid],
        "outlier_labels": outliers,
    })
    if outliers:
        out["notes"].append("检测到离群事件（|残差|>3×RMS 且 >1ms）：%s；"
                            "可剔除后重拟合。" % ", ".join(outliers))
    # 绝对 offset 大是正常的（外置 0 起点 vs X7 绝对 µs），不做「offset<10ms」门槛
    out["notes"].append("绝对 offset 大小不作为同步判据；验收看校正后残差。")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="外置 .bin 与 X7 native FIFO ULog 的软件时间映射（offset+drift+残差）")
    ap.add_argument("--bin", required=True, help="外置原始 .bin（VIB3）")
    ap.add_argument("--ulog", required=True, help="X7 ULog 文件")
    ap.add_argument("--events", required=True, help="匹配事件 JSON")
    ap.add_argument("--axis", choices=("accel", "gyro"), default="gyro",
                    help="X7 native FIFO 轴（默认 gyro）")
    ap.add_argument("--instance", type=int, default=0, help="X7 实例（默认 0）")
    ap.add_argument("--reader", default="auto", help="ULog 读取器（auto/pyulog/native）")
    ap.add_argument("--json", nargs="?", const="-", default=None,
                    help="输出 JSON（默认 stdout；给路径写文件）")
    args = ap.parse_args(argv)

    ext = read_external_axis(args.bin)
    content = ulog_reader.load_ulog(args.ulog, reader=args.reader)
    topic = "sensor_%s_fifo" % args.axis
    dataset = content.dataset(topic, args.instance)
    if dataset is None:
        print("错误：ULog 里没有 %s[%d]（native FIFO 主题缺失）" % (topic, args.instance),
              file=sys.stderr)
        return 1

    report, findings = va.analyze_fifo_dataset(dataset)
    segments = reconstruct_x7_segments(dataset)
    spans = x7_timeline_span(segments)

    with open(args.events, "r", encoding="utf-8") as f:
        events = json.load(f).get("events", [])
    fit = fit_events(events)

    payload = {
        "external": ext,
        "ulog": {
            "path": args.ulog,
            "reader": content.reader,
            "topic": topic,
            "instance": args.instance,
            "packets": dataset.count,
            "device_segments": spans,
            "findings": [f.as_dict() for f in findings],
        },
        "mapping": fit,
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.json and args.json != "-":
        with open(args.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print("JSON 已写入 %s" % args.json, file=sys.stderr)
    else:
        print(text)

    # 退出门槛：能给出 drift 且残差不过大即 0；无事件/退化 2
    if fit["drift_ppm"] is None and fit["n_events"] > 1:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
