#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_bin_ulog.py — 外置 .bin（RK3588 直接采集）与 X7 native FIFO ULog 的软件时间映射

目的：把外置原始记录（raw ticks → 物理 µs）的逐样本时间轴，与 X7 原生
sensor_accel_fifo / sensor_gyro_fifo 的逐样本时间轴（timestamp_sample 最后样本
锚点 + dt 还原），通过一组**匹配事件**拟合 offset + drift(ppm)，并给出校正后
残差与可用性判据。**无物理 GPIO/FSYNC，无硬件同步完成宣称**——绝对 offset 大小
不是判据，验收只看校正后残差是否与事件的明确不确定度一致。

导出（可检验、可流式，恒定内存）：
  --csv-ext    PATH  外置逐样本时间轴：index,segment,t_us(相对首样本=0),ax[g],...
  --csv-x7     PATH  X7 选定 device 的**一个**连续段逐样本：
                     packet,index_in_packet,t_us(绝对µs),device_id,x_<unit>,...
  --csv-mapped PATH  有效 fit 后外置逐样本映射到 PX4 时间轴：
                     index,segment,t_us,t_px4_us(=slope*t_us+intercept),ax[g],...
                     **无有效 fit 不导出**（不伪对齐）。

事件输入（--events events.json，唯一真源）：
  {"events": [{"external_us": 123.4, "x7_us": 987654.3, "label": "tap1",
               "provenance": "manual-tap", "uncertainty_us": 500.0}, ...]}
  external_us = 以外置 .bin 首样本为 0 的相对物理 µs；x7_us = X7 绝对 µs；
  uncertainty_us 必须为正有限；label / provenance 必须非空（provenance 缺失即拒绝）。

口径（与 Tools/vibration/vibration_analysis.py 读取契约一致）：
  * X7 timestamp_sample 是**本包最后样本**锚点；dt 单位 µs（不硬编码 125）；
    t_i = T - (N-1-i)*dt。X7 timestamp 已是 µs，**不再乘 32/30**。
  * 每包按 samples 有效前缀（padding 忽略），每包独立 scale。
  * device_id=0 拒绝；device_id / 实例 / 缺包 / 回退 / 重复真正分段。
  * 外置 raw ticks → µs = raw * 32/30（由 .bin 文件头元数据给出）。

判据（明示，见 fit_events）：
  至少 3 个独立事件；事件都必须落在**同一个外置连续段**与**选定的一个 X7 连续段**
  内（不得跨断裂/缺口/复位拟合）；抗离群（Theil-Sen）稳健拟合后对内点重拟合；
  斜率受合理时钟比约束（|drift_ppm| ≤ --max-drift-ppm）；按「内点 |残差| ≤
  3×uncertainty」判定可用。phase/high-frequency 适用性默认 **unverified/False**：
  --phase-budget-us 只检查事件模型是否符合用户容许误差，不认证 ADC 样本相位。
  事件误差用 **worst inlier（最大）不确定度**，不用 median 掩盖高不确定点。
  200 µs 对应 500 Hz 的 36° 相位误差——这**不是**一条保守通用合格线。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_PARENT = os.path.dirname(os.path.dirname(HERE))   # Tools/vibration
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

import bin_reader as br          # noqa: E402
import ulog_reader               # noqa: E402
import vibration_analysis as va  # noqa: E402

MIN_EVENTS = 3
DEFAULT_MAX_DRIFT_PPM = 10000.0       # 1%：更松会允许明显错配
GAP_TOLERANCE_US = 2000.0             # X7 相邻锚点跳变容忍（与振动分析一致）

UNIT_BY_AXIS = {"accel": ("m/s^2", "x_m_s2", "y_m_s2", "z_m_s2"),
                "gyro": ("rad/s", "x_rad_s", "y_rad_s", "z_rad_s")}


def _finite(v):
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    if n % 2:
        return float(s[n // 2])
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


# ---------------------------------------------------------------------------
# 外置 .bin 时间轴
# ---------------------------------------------------------------------------
def read_external(binpath):
    """读取外置 .bin 的时间轴来源/跨度/连续段（首样本为 0 的相对 µs）。"""
    reader = br.BinReader(binpath)
    stats = reader.run()
    timing = stats["timing"]
    header = stats["header"]
    timeline = stats.get("timeline", {})
    return {
        "path": binpath,
        "records": stats["integrity"]["records"],
        "duration_s": timing["duration_s"],
        "t_end_us": round(timing["duration_s"] * 1e6, 1),
        "format_version": header.get("format_version"),
        "ticks_to_us_num": header.get("tmst_scale_num"),
        "ticks_to_us_den": header.get("tmst_scale_den"),
        "tmst_source": header.get("tmst_source"),
        "tmst_res": header.get("tmst_res"),
        "measured_odr_hz": timing["measured_odr_hz"],
        "host_rate_hz": timing.get("host_rate_hz"),
        "segments": timeline.get("segments", []),
        "tmst_breaks": timeline.get("tmst_breaks", 0),
        "ok": stats["ok"],
        "errors": stats["errors"],
        "anomalies": stats["anomalies"],
    }


# ---------------------------------------------------------------------------
# X7 native FIFO 分段与逐样本
# ---------------------------------------------------------------------------
def _sample_value(axis_rows, k, i):
    row = axis_rows[k]
    if isinstance(row, (list, tuple)) and i < len(row):
        return row[i]
    return row


def reconstruct_x7_segments(dataset, gap_tolerance_us=GAP_TOLERANCE_US):
    """按读取契约还原 X7 native FIFO 每包样本时间，并按连续段切分。

    返回 dict:
      segments: list[{device_id, packets:[{packet_index, anchor_us, n, dt_us,
                  first_us, last_us, scale}]}]
      device0_packets / invalid_dt / invalid_scale / invalid_samples / breaks
    断段条件（真正断段，不是计数）：device_id 变化、锚点回退、重复包、缺包
    （相邻锚点跳变 > 上包覆盖时长 + 容忍）。device_id=0 拒绝。padding 忽略。
    """
    segments = []
    stats = {"device0_packets": 0, "invalid_dt": 0, "invalid_scale": 0,
             "invalid_samples": 0, "breaks": 0}

    anchors = dataset.get("timestamp_sample")
    ts_rows = dataset.get("timestamp")
    devices = dataset.get("device_id")
    dts = dataset.get("dt")
    scales = dataset.get("scale")
    samples = dataset.get("samples")

    cur = None
    prev = None          # last valid packet dict
    for k in range(dataset.count):
        device_id = int(devices[k])
        if device_id == 0:
            stats["device0_packets"] += 1
            cur = None
            prev = None
            continue
        dt = float(dts[k])
        scale = float(scales[k])
        count = int(samples[k])
        if not _finite(dt) or dt <= 0.0:
            stats["invalid_dt"] += 1
            cur = None
            prev = None
            continue
        if not _finite(scale) or scale == 0.0:
            stats["invalid_scale"] += 1
            cur = None
            prev = None
            continue
        if count < 1 or count > va.FIFO_SAMPLES_MAX:
            stats["invalid_samples"] += 1
            cur = None
            prev = None
            continue

        anchor = int(anchors[k])
        ts = int(ts_rows[k])
        valid = count
        first = anchor - (valid - 1) * dt
        pkt = {"packet_index": k, "anchor_us": anchor, "n": valid, "dt_us": dt,
               "first_us": first, "last_us": anchor, "scale": scale}

        # 断段判定
        new_segment = False
        if cur is None or device_id != cur["device_id"]:
            new_segment = True
        elif prev is not None:
            if anchor < prev["anchor_us"]:
                stats["breaks"] += 1
                new_segment = True
            elif anchor == prev["anchor_us"] and ts == int(ts_rows[prev["packet_index"]]):
                stats["breaks"] += 1
                new_segment = True
            else:
                expected = prev["n"] * prev["dt_us"]
                if anchor - prev["anchor_us"] > expected + gap_tolerance_us:
                    stats["breaks"] += 1
                    new_segment = True

        if new_segment:
            cur = {"device_id": device_id, "packets": []}
            segments.append(cur)
        cur["packets"].append(pkt)
        prev = pkt

    return segments, stats


def x7_spans(segments):
    """按 segment 汇总每个连续段的 [first_us, last_us]（跨器件/断裂不假合并）。"""
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
            "samples": sum(p["n"] for p in pk),
        })
    return spans


def x7_sample_iter(dataset, segment, axis):
    """流式产出**一个**连续段的逐样本 (t_us, device_id, x, y, z)。

    值 = raw * per-packet scale（accel m/s^2，gyro rad/s）；t_us = 绝对 µs。
    """
    xs = dataset.get("x")
    ys = dataset.get("y")
    zs = dataset.get("z")
    for pkt in segment["packets"]:
        k = pkt["packet_index"]
        n, dt, anchor, scale = pkt["n"], pkt["dt_us"], pkt["anchor_us"], pkt["scale"]
        for i in range(n):
            t_us = anchor - (n - 1 - i) * dt
            x = _sample_value(xs, k, i) * scale
            y = _sample_value(ys, k, i) * scale
            z = _sample_value(zs, k, i) * scale
            yield (t_us, segment["device_id"], x, y, z)


# ---------------------------------------------------------------------------
# 事件校验与稳健拟合
# ---------------------------------------------------------------------------
def validate_events(events):
    """校验事件：正有限不确定度、非空 label/provenance、有限时间。返回 (cleaned, problems)。"""
    cleaned, problems = [], []
    for i, e in enumerate(events):
        label = e.get("label")
        provenance = e.get("provenance")
        ext = e.get("external_us")
        x7 = e.get("x7_us")
        unc = e.get("uncertainty_us")
        if not isinstance(label, str) or not label.strip():
            problems.append("event %d missing/empty label" % i)
            continue
        if not isinstance(provenance, str) or not provenance.strip():
            problems.append("event %r missing provenance" % label)
            continue
        if not _finite(ext) or not _finite(x7):
            problems.append("event %r has non-finite external_us/x7_us" % label)
            continue
        if not _finite(unc) or float(unc) <= 0.0:
            problems.append("event %r has non-positive/non-finite uncertainty_us" % label)
            continue
        cleaned.append({"label": label, "provenance": provenance,
                        "external_us": float(ext), "x7_us": float(x7),
                        "uncertainty_us": float(unc)})
    return cleaned, problems


def _ols(ext, x7):
    n = len(ext)
    if n < 2:
        return None
    mx = sum(ext) / n
    my = sum(x7) / n
    sxx = sum((x - mx) ** 2 for x in ext)
    if sxx <= 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(ext, x7))
    a = sxy / sxx
    b = my - a * mx
    return a, b


def _wls(ext, x7, w):
    sw = sum(w)
    if sw <= 0 or len(ext) < 2:
        return None
    mx = sum(wi * x for wi, x in zip(w, ext)) / sw
    my = sum(wi * y for wi, y in zip(w, x7)) / sw
    sxx = sum(wi * (x - mx) ** 2 for wi, x in zip(w, ext))
    if sxx <= 0:
        return None
    sxy = sum(wi * (x - mx) * (y - my) for wi, x, y in zip(w, ext, x7))
    a = sxy / sxx
    b = my - a * mx
    return a, b


def _theil_sen(ext, x7):
    """Theil-Sen 稳健斜率 + 中位截距。对抗单点杠杆/离群。"""
    n = len(ext)
    slopes = []
    for i in range(n):
        for j in range(i + 1, n):
            if ext[j] != ext[i]:
                slopes.append((x7[j] - x7[i]) / (ext[j] - ext[i]))
    if not slopes:
        return None
    slope = _median(slopes)
    intercept = _median(y - slope * x for x, y in zip(ext, x7))
    return slope, intercept


def fit_events(events, ext_segments, x7_segment, device_id,
               max_drift_ppm=DEFAULT_MAX_DRIFT_PPM, phase_budget_us=None):
    """从匹配事件拟合 offset+drift，按明示判据给出结论。

    参数：
      events       校验后的事件列表（≥3 个独立事件）。
      ext_segments 外置连续段列表 [{segment_id, first_us, last_us, samples}]。
                   事件必须全部落在**同一个**外置连续段内（不得跨断裂/缺口）。
      x7_segment   **单个**选定的 X7 连续段 {device_id, first_us, last_us, packets, samples}。
                   事件必须全部落在该段内（不得跨复位/缺口）。
      phase_budget_us 事件模型容许误差（µs）；样本重建与捕获延迟未校准，
                   phase/high-frequency 适用性仍为 unverified/False。

    返回 dict：status / slope / drift_ppm / intercept_us / offset_us /
    residual_rms_us / residual_max_us / residual_us / residual_uncertainty_ratio /
    inlier_labels / outlier_labels / n_events / n_inliers /
    median_uncertainty_us / worst_inlier_uncertainty_us /
    phase_suitability / usable_for_500hz / phase_budget_us / phase_error_deg_at_500hz /
    notes。
    """
    out = {
        "n_events": len(events), "n_inliers": 0,
        "status": "invalid_events",
        "slope": None, "drift_ppm": None, "intercept_us": None, "offset_us": None,
        "residual_rms_us": None, "residual_max_us": None,
        "residual_us": [], "residual_uncertainty_ratio": None,
        "inlier_labels": [], "outlier_labels": [],
        "median_uncertainty_us": None, "worst_inlier_uncertainty_us": None,
        "phase_suitability": "unverified", "usable_for_500hz": False,
        "event_alignment_within_budget": None,
        "phase_error_scope": "event_alignment_only",
        "external_segment_id": None,
        "phase_budget_us": None, "phase_error_deg_at_500hz": None,
        "notes": [],
    }

    if len(events) < MIN_EVENTS:
        out["status"] = "insufficient_events"
        out["notes"].append("事件数 %d < %d：不足以估计 offset+drift（缺证据）"
                            % (len(events), MIN_EVENTS))
        return out

    ext_us = [e["external_us"] for e in events]
    x7_us = [e["x7_us"] for e in events]
    unc = [e["uncertainty_us"] for e in events]
    labels = [e["label"] for e in events]

    # ---- 选定 X7 连续段必须有效 ----
    if not x7_segment or not x7_segment.get("packets"):
        out["status"] = "no_x7_segment"
        out["notes"].append("选定的 X7 device/segment 无有效连续段（或无 packets）")
        return out
    x7_first = x7_segment["first_us"]
    x7_last = x7_segment["last_us"]

    # ---- 事件必须落在同一个外置连续段内（不得跨断裂/缺口拼轴）----
    ext_seg = None
    for seg in ext_segments:
        if all(seg["first_us"] <= e <= seg["last_us"] for e in ext_us):
            ext_seg = seg
            break
    if ext_seg is None:
        out["status"] = "out_of_range"
        out["notes"].append(
            "事件未落在同一个外置连续段内（越界或跨断裂/缺口/复位），拒绝拟合")
        return out

    out["external_segment_id"] = ext_seg.get("segment_id", 0)

    # ---- 事件必须落在选定的 X7 连续段内 ----
    for e in events:
        if not (x7_first <= e["x7_us"] <= x7_last):
            out["status"] = "out_of_range"
            out["notes"].append("事件 %r x7_us=%g 不在选定 X7 段 [%g,%g] 内"
                                % (e["label"], e["x7_us"], x7_first, x7_last))
            return out

    # 稳健拟合（Theil-Sen）→ 内点 → 重拟合（不确定度加权）
    robust = _theil_sen(ext_us, x7_us)
    if robust is None:
        out["status"] = "degenerate"
        out["notes"].append("事件时间轴退化（external_us 全同或不足）")
        return out
    r_slope, r_intercept = robust

    # 稳健残差 → MAD 内点
    resid_robust = [y - (r_slope * x + r_intercept) for x, y in zip(ext_us, x7_us)]
    med_resid = _median(resid_robust)
    mad = _median([abs(r - med_resid) for r in resid_robust])
    thresh = max(3.0 * 1.4826 * mad, 1.0)
    inlier_idx = [i for i, r in enumerate(resid_robust) if abs(r - med_resid) <= thresh]

    if len(inlier_idx) < MIN_EVENTS:
        out["status"] = "residual_exceeds_uncertainty"
        out["outlier_labels"] = labels
        out["notes"].append("稳健拟合后内点仅 %d 个（< %d）：多数事件不一致，无法给出可信映射"
                            % (len(inlier_idx), MIN_EVENTS))
        return out

    w = [1.0 / (unc[i] ** 2) for i in inlier_idx]
    refit = _wls([ext_us[i] for i in inlier_idx], [x7_us[i] for i in inlier_idx], w)
    if refit is None:
        out["status"] = "degenerate"
        out["notes"].append("内点重拟合退化解")
        return out
    slope, intercept = refit

    drift_ppm = (slope - 1.0) * 1e6
    if abs(drift_ppm) > max_drift_ppm:
        out["status"] = "unreasonable_clock_ratio"
        out["slope"] = slope
        out["drift_ppm"] = drift_ppm
        out["notes"].append("时钟比不合理：drift=%.1f ppm 超出 ±%.1f ppm"
                            % (drift_ppm, max_drift_ppm))
        return out

    resid = [y - (slope * x + intercept) for x, y in zip(ext_us, x7_us)]
    inlier_resid = [resid[i] for i in inlier_idx]
    rms = math.sqrt(sum(r * r for r in inlier_resid) / len(inlier_resid))
    max_resid = max(abs(r) for r in inlier_resid)
    ratio = max(abs(resid[i]) / unc[i] for i in inlier_idx)
    outlier_labels = [labels[i] for i in range(len(labels)) if i not in inlier_idx]
    worst_inlier_unc = max(unc[i] for i in inlier_idx)

    out.update({
        "n_inliers": len(inlier_idx),
        "slope": slope,
        "drift_ppm": drift_ppm,
        "intercept_us": intercept,
        "offset_us": sum(x7_us[i] - ext_us[i] for i in inlier_idx) / len(inlier_idx),
        "residual_rms_us": rms,
        "residual_max_us": max_resid,
        "residual_us": [round(r, 3) for r in resid],
        "residual_uncertainty_ratio": ratio,
        "inlier_labels": [labels[i] for i in inlier_idx],
        "outlier_labels": outlier_labels,
        "median_uncertainty_us": _median([unc[i] for i in inlier_idx]),
        "worst_inlier_uncertainty_us": worst_inlier_unc,
    })

    if ratio > 3.0:
        out["status"] = "residual_exceeds_uncertainty"
        out["notes"].append("内点残差与不确定度之比 %.1f > 3：事件与拟合不一致（或不确定度被低估）"
                            % ratio)
    else:
        out["status"] = "ok"

    # ---- phase / high-frequency 适用性：默认 unverified，绝不靠 median 冒充 ----
    if out["status"] == "ok":
        # 保守判据用 worst inlier 残差与 worst inlier 不确定度的较大者
        worst = max(out["residual_max_us"], out["worst_inlier_uncertainty_us"])
        out["phase_error_deg_at_500hz"] = worst * 1e-6 * 360.0 * 500.0
        if phase_budget_us is not None and _finite(phase_budget_us) and phase_budget_us > 0:
            out["phase_budget_us"] = float(phase_budget_us)
            out["event_alignment_within_budget"] = worst <= float(phase_budget_us)
            out["notes"].append(
                "允许误差阈值只能检验事件拟合；FIFO 样本重建和捕获延迟尚未校准，"
                "不能据此认证 500 Hz 样本相位精度。")
        else:
            out["notes"].append(
                "未提供显式相位误差预算 --phase-budget-us：phase/high-frequency 适用性为 "
                "unverified（不用事件残差+中位不确定度冒充；%.0f µs 对应 500 Hz 的 "
                "%.0f° 相位误差，不是保守通用合格线）"
                % (worst, out["phase_error_deg_at_500hz"]))
    else:
        out["notes"].append("拟合未通过（status=%s），phase/high-frequency 适用性不可用"
                            % out["status"])

    out["notes"].append("绝对 offset 大小不作为同步判据；验收看校正后残差与不确定度。")
    if outlier_labels:
        out["notes"].append("离群事件（稳健拟合剔除）：%s" % ", ".join(outlier_labels))
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="外置 .bin 与 X7 native FIFO ULog 的软件时间映射（offset+drift+残差）")
    ap.add_argument("--bin", required=True, help="外置原始 .bin（VIB3）")
    ap.add_argument("--ulog", required=True, help="X7 ULog 文件")
    ap.add_argument("--events", required=True, help="匹配事件 JSON（唯一真源）")
    ap.add_argument("--axis", choices=("accel", "gyro"), default="gyro",
                    help="X7 native FIFO 轴（默认 gyro）")
    ap.add_argument("--instance", type=int, default=0, help="X7 实例（默认 0）")
    ap.add_argument("--device", type=int, default=None,
                    help="选定 X7 device_id 段（默认：唯一非 0 device，多个则报错列出）")
    ap.add_argument("--segment", type=int, default=None,
                    help="选定 X7 device 的连续段索引（多个连续段时必选；CSV/输出只该段）")
    ap.add_argument("--reader", default="auto", help="ULog 读取器（auto/pyulog/native）")
    ap.add_argument("--max-drift-ppm", type=float, default=DEFAULT_MAX_DRIFT_PPM,
                    help="合理时钟比上限 ppm（默认 %.0f）" % DEFAULT_MAX_DRIFT_PPM)
    ap.add_argument("--phase-budget-us", type=float, default=None,
                    help="事件模型容许误差 µs；不代表实机样本相位已校准")
    ap.add_argument("--csv-ext", default=None, help="导出外置逐样本时间轴 CSV")
    ap.add_argument("--csv-x7", default=None, help="导出 X7 选定段逐样本 CSV")
    ap.add_argument("--csv-mapped", default=None,
                    help="有效 fit 后导出外置逐样本映射到 PX4 时间轴的 CSV")
    ap.add_argument("--json", nargs="?", const="-", default=None,
                    help="输出 JSON（默认 stdout；给路径写文件）")
    args = ap.parse_args(argv)

    # 外置时间轴（含连续段）
    ext = read_external(args.bin)
    if ext["errors"]:
        print("错误：外置 .bin 存在结构错误：%s" % ext["errors"], file=sys.stderr)
        return 2
    # 物理时间基必须可信才允许映射：legacy v2（无 raw ticks→µs 换算元数据）把 raw 当 µs，
    # 这只是旧假设、物理时间轴不可信 ⇒ 拒绝，不得据此产出 offset/drift/映射。
    # （该情形在 bin_reader 里是 anomaly 而非 error，故必须在 errors 之外单独拦。）
    if (ext.get("format_version") or 0) < 3 or not ext.get("ticks_to_us_num") \
            or not ext.get("ticks_to_us_den"):
        print("错误：外置 .bin 无可信的 raw ticks→µs 换算元数据"
              "（format_version=%s，scale=%s/%s）。legacy v2 的 raw=µs 仅是旧假设，"
              "物理时间轴不可信 —— 拒绝映射。"
              % (ext.get("format_version"), ext.get("ticks_to_us_num"),
                 ext.get("ticks_to_us_den")), file=sys.stderr)
        return 2
    ext_segments = [s for s in ext["segments"] if s.get("samples", 0) > 0]
    if not ext_segments:
        print("错误：外置 .bin 无有效连续段（空/中止采集）", file=sys.stderr)
        return 2

    # X7 native FIFO
    content = ulog_reader.load_ulog(args.ulog, reader=args.reader)
    diagnostic = content.reader.startswith("native")
    topic = "sensor_%s_fifo" % args.axis
    dataset = content.dataset(topic, args.instance)
    if dataset is None:
        print("错误：ULog 里没有 %s[%d]（native FIFO 主题缺失）" % (topic, args.instance),
              file=sys.stderr)
        return 1

    report, findings = va.analyze_fifo_dataset(dataset)
    segments, seg_stats = reconstruct_x7_segments(dataset)

    # 选定 device：默认唯一非 0 device；多个则要求 --device
    devices = sorted({s["device_id"] for s in segments if s["packets"]})
    if args.device is None:
        if len(devices) == 1:
            device_id = devices[0]
        elif len(devices) == 0:
            print("错误：X7 无有效（非 device=0）连续段，无法映射", file=sys.stderr)
            return 2
        else:
            print("错误：X7 有多个 device_id 段 %s，请用 --device 指定" % devices,
                  file=sys.stderr)
            return 2
    else:
        device_id = args.device
        if device_id not in devices:
            print("错误：--device %d 不在有效 device 段 %s" % (device_id, devices),
                  file=sys.stderr)
            return 2

    # 选定该 device 的一个连续段（不跨复位/缺口拟合）
    dev_segs = [s for s in segments if s["device_id"] == device_id and s["packets"]]
    dev_spans = x7_spans(dev_segs)
    if args.segment is not None:
        if not (0 <= args.segment < len(dev_segs)):
            print("错误：--segment %d 越界（该 device 有 %d 个连续段 %s）"
                  % (args.segment, len(dev_segs),
                     ["[%.0f,%.0f]µs" % (sp["first_us"], sp["last_us"]) for sp in dev_spans]),
                  file=sys.stderr)
            return 2
        sel_seg = dev_segs[args.segment]
        sel_span = dev_spans[args.segment]
    else:
        if len(dev_segs) == 1:
            sel_seg = dev_segs[0]
            sel_span = dev_spans[0]
        elif len(dev_segs) == 0:
            print("错误：X7 该 device 无有效连续段", file=sys.stderr)
            return 2
        else:
            print("错误：X7 device %d 有 %d 个连续段 %s，请用 --segment 指定"
                  % (device_id, len(dev_segs),
                     ["[%.0f,%.0f]µs" % (sp["first_us"], sp["last_us"]) for sp in dev_spans]),
                  file=sys.stderr)
            return 2

    # 事件
    with open(args.events, "r", encoding="utf-8") as f:
        raw_events = json.load(f).get("events", [])
    cleaned, problems = validate_events(raw_events)
    if problems:
        for p in problems:
            print("事件校验失败：%s" % p, file=sys.stderr)

    # 拟合（选定单段）
    fit = fit_events(cleaned, ext_segments, sel_span, device_id,
                     max_drift_ppm=args.max_drift_ppm,
                     phase_budget_us=args.phase_budget_us)

    # 导出外置逐样本时间轴（相对首样本=0，复用 bin_reader 流式 CSV，含 segment 列）
    if args.csv_ext:
        br.BinReader(args.bin, csv_path=args.csv_ext).run()

    # 导出 X7 选定段逐样本 CSV（per-packet scale，物理量带单位；只该段）
    if args.csv_x7:
        unit, cx, cy, cz = UNIT_BY_AXIS[args.axis]
        with open(args.csv_x7, "w", encoding="utf-8", newline="") as f:
            f.write("packet,index_in_packet,t_us,device_id,%s,%s,%s\n" % (cx, cy, cz))
            for pkt in sel_seg["packets"]:
                k = pkt["packet_index"]
                n, dt, anchor, scale = pkt["n"], pkt["dt_us"], pkt["anchor_us"], pkt["scale"]
                xs = dataset.get("x")
                ys = dataset.get("y")
                zs = dataset.get("z")
                for i in range(n):
                    t_us = anchor - (n - 1 - i) * dt
                    x = _sample_value(xs, k, i) * scale
                    y = _sample_value(ys, k, i) * scale
                    z = _sample_value(zs, k, i) * scale
                    f.write("%d,%d,%.3f,%d,%.9g,%.9g,%.9g\n"
                            % (k, i, t_us, device_id, x, y, z))

    # 映射后的外置逐样本（t_px4_us = slope*t_us + intercept）：仅有效 fit 才导出
    if args.csv_mapped:
        if fit["status"] != "ok":
            print("错误：无有效 fit（status=%s），不导出伪对齐的映射 CSV" % fit["status"],
                  file=sys.stderr)
            return 2
        slope, intercept = fit["slope"], fit["intercept_us"]
        br.BinReader(args.bin, csv_path=args.csv_mapped, t_map=(slope, intercept),
                     csv_segment_id=fit["external_segment_id"]).run()

    payload = {
        "external": ext,
        "ulog": {
            "path": args.ulog,
            "reader": content.reader,
            "reader_grade": "diagnostic" if diagnostic else "acceptance",
            "topic": topic,
            "instance": args.instance,
            "device_id": device_id,
            "segment_index": args.segment if args.segment is not None else 0,
            "packets": dataset.count,
            "device_segments": dev_spans,
            "selected_segment": sel_span,
            "segment_stats": seg_stats,
            "findings": [fnd.as_dict() for fnd in findings],
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

    if fit["status"] != "ok":
        return 2
    if diagnostic:
        print("警告：--reader native 是 diagnostic fallback，不是 pyulog 级验收结果",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
