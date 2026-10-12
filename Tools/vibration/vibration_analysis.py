# -*- coding: utf-8 -*-
"""
Analysis core for raw FIFO vibration logs.

Data semantics implemented here come from the PX4 v1.16.1 sources:

  * ``samples`` is the number of valid entries at the start of the int16 x/y/z
    arrays (1..32). Entries at index >= samples are padding and are excluded.
  * ``x/y/z`` are raw int16 in the FRD board frame with the driver rotation
    applied; the physical value is ``raw * scale`` (acc in m/s^2, gyro in rad/s).
    Full calibration (offsets/thermal) is *not* applied to the raw FIFO data.
  * ``timestamp_sample`` is the nominal anchor of the **last** sample of the
    batch and ``dt`` is in microseconds, so ``t_i = T - (N - 1 - i) * dt``.
  * the cross-packet residual is ``T[k] - T[k-1] - N[k] * dt[k]``; it is a
    consistency metric, not a drop counter.
  * endpoint sample rate = ``sum(N[k] for k >= 1) / (T_last - T_first) * 1e6``.

Known chip level caveats (must not be turned into hard assertions):
  * ICM42688P: ``dt`` is the last chip timestamp delta of the batch, quantized to
    roughly 17.0667 us and possibly varying inside one batch.
  * ICM20689: the accel phase can be off by ~125 us relative to the gyro, so the
    two streams are not strictly sample synchronous.
"""

import math
import statistics

from vibration_common import (
    FIFO_SAMPLES_MAX,
    INT16_MAX,
    INT16_MIN,
    SEVERITY_ERROR,
    SEVERITY_INFO,
    SEVERITY_WARNING,
    Finding,
    TOPIC_ACCEL_FIFO,
    TOPIC_GYRO_FIFO,
    TOPIC_LOGGER_STATUS,
    TOPIC_VEHICLE_IMU_STATUS,
)

DEFAULT_GAP_TOLERANCE_US = 2000.0
DEFAULT_FULL_SCALE_FRACTION = 0.95
DEFAULT_LATENCY_LIMIT_US = 500000

KIND_ACCEL = "accel"
KIND_GYRO = "gyro"

UNITS = {KIND_ACCEL: "m/s^2", KIND_GYRO: "rad/s"}


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def _stats(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "max": max(values),
        "p95": _percentile(values, 0.95),
    }


def kind_for_topic(topic_name):
    if topic_name == TOPIC_ACCEL_FIFO:
        return KIND_ACCEL
    if topic_name == TOPIC_GYRO_FIFO:
        return KIND_GYRO
    return None


def analyze_fifo_dataset(dataset, gap_tolerance_us=DEFAULT_GAP_TOLERANCE_US,
                         full_scale_fraction=DEFAULT_FULL_SCALE_FRACTION,
                         latency_limit_us=DEFAULT_LATENCY_LIMIT_US):
    """Check one sensor_*_fifo subscription and reconstruct its data stream."""
    kind = kind_for_topic(dataset.name)
    findings = []
    where = "%s[%d]" % (dataset.name, dataset.multi_id)

    timestamps = dataset.get("timestamp")
    anchors = dataset.get("timestamp_sample")
    device_ids = dataset.get("device_id")
    dts = dataset.get("dt")
    scales = dataset.get("scale")
    samples = dataset.get("samples")
    xs, ys, zs = dataset.get("x"), dataset.get("y"), dataset.get("z")

    report = {
        "topic": dataset.name,
        "multi_id": dataset.multi_id,
        "kind": kind,
        "unit": UNITS.get(kind),
        "packets": dataset.count,
        "samples_total": 0,
        "device_segments": [],
        "residual_us": {},
        "endpoint_rate_hz": None,
        "publish_rate_hz": None,
        "samples_per_packet": {},
        "dt_us": {},
        "scale": {},
        "latency_us": {},
        "raw": {"int16_limit_samples": 0, "near_int16_limit_samples": 0,
                "min": None, "max": None, "unit": "raw int16"},
        "device_id_zero_packets": 0,
        "physical_abs_max": None,
        "padding_tail_nonzero_packets": 0,
        "duplicate_packets": 0,
        "timestamp_regressions": 0,
        "anchor_regressions": 0,
        "fingerprint_fields_present": {
            "timestamp": bool(timestamps), "timestamp_sample": bool(anchors),
            "device_id": bool(device_ids), "dt": bool(dts), "scale": bool(scales),
            "samples": bool(samples), "x": bool(xs), "y": bool(ys), "z": bool(zs),
        },
    }

    missing = [name for name, present in report["fingerprint_fields_present"].items() if not present]
    if missing:
        findings.append(Finding(SEVERITY_ERROR, "fifo/missing-fields",
                                "%s missing required FIFO fields: %s" % (where, ", ".join(sorted(missing))),
                                where))
        return report, findings

    n_records = min(len(timestamps), len(anchors), len(device_ids), len(dts),
                    len(scales), len(samples), len(xs), len(ys), len(zs))
    if n_records != dataset.count:
        findings.append(Finding(SEVERITY_WARNING, "fifo/field-length-mismatch",
                                "%s field arrays have inconsistent lengths (dataset count %d, usable %d)"
                                % (where, dataset.count, n_records), where))

    bad_dt = 0
    bad_scale = 0
    bad_samples = 0
    int16_limit = 0
    near_int16_limit = 0
    raw_min = None
    raw_max = None
    physical_abs_max = 0.0
    duplicates = 0
    ts_regressions = 0
    anchor_regressions = 0
    padding_nonzero = 0
    latencies = []
    dt_values = []
    samples_per_packet = {}
    scale_values = []
    residuals = []
    segments = []
    samples_total = 0
    device_zero_packets = 0

    prev_ts = None
    prev_anchor = None
    prev_device = None
    current_segment = None

    for k in range(n_records):
        # pyulog keeps uint64 numpy timestamps. Convert before subtraction so
        # negative latency remains negative instead of wrapping to 2**64.
        ts = int(timestamps[k])
        anchor = int(anchors[k])
        device_id = int(device_ids[k])
        dt = float(dts[k])
        scale = float(scales[k])
        count = int(samples[k])

        if not _finite(dt) or dt <= 0.0:
            bad_dt += 1
        else:
            dt_values.append(dt)

        if not _finite(scale) or scale == 0.0:
            bad_scale += 1
        else:
            scale_values.append(scale)

        if count < 1 or count > FIFO_SAMPLES_MAX:
            bad_samples += 1
        else:
            samples_per_packet[count] = samples_per_packet.get(count, 0) + 1
            samples_total += count

        if prev_ts is not None:
            if ts < prev_ts:
                ts_regressions += 1
            elif ts == prev_ts and anchor == prev_anchor:
                duplicates += 1
        if prev_anchor is not None and anchor < prev_anchor:
            anchor_regressions += 1

        # publication latency: driver publication timestamp minus the batch anchor of the last
        # sample (this is NOT the SD-card write time)
        latencies.append((ts or 0) - anchor)

        # device changes segment the stream (accel and gyro instance numbering is separate).
        # same_device must be decided *before* prev_device is updated, otherwise the first packet
        # of a new device would be compared against the previous device's anchor.
        same_device = (device_id == prev_device)

        if not same_device:
            current_segment = {"device_id": device_id, "packets": 0, "samples": 0,
                               "first_anchor_us": anchor, "last_anchor_us": anchor,
                               "first_timestamp_us": ts, "last_timestamp_us": ts,
                               "anchors": [], "samples_list": [], "residuals": []}
            segments.append(current_segment)

            if device_id == 0:
                device_zero_packets += 1

        current_segment["packets"] += 1
        current_segment["samples"] += max(count, 0) if count <= FIFO_SAMPLES_MAX else 0
        current_segment["last_anchor_us"] = anchor
        current_segment["last_timestamp_us"] = ts
        current_segment["anchors"].append(anchor)
        current_segment["samples_list"].append(count if 1 <= count <= FIFO_SAMPLES_MAX else 0)

        # stream reconstruction for the valid prefix only
        valid = max(0, min(count, FIFO_SAMPLES_MAX))
        for index in range(valid):
            for axis in (xs, ys, zs):
                value = axis[k][index] if isinstance(axis[k], list) else axis[k]
                if value is None:
                    continue
                raw_min = value if raw_min is None else min(raw_min, value)
                raw_max = value if raw_max is None else max(raw_max, value)
                if value >= INT16_MAX or value <= INT16_MIN:
                    int16_limit += 1
                elif abs(value) >= full_scale_fraction * INT16_MAX:
                    near_int16_limit += 1
                if _finite(scale):
                    physical_abs_max = max(physical_abs_max, abs(value * scale))

        # padding area must not contribute: record if it is not zero (informational)
        for axis in (xs, ys, zs):
            row = axis[k]
            if isinstance(row, list) and len(row) > valid:
                if any(v != 0 for v in row[valid:]):
                    padding_nonzero += 1
                    break

        # residual is only comparable inside one device_id segment
        if same_device and k > 0 and _finite(dt) and dt > 0 and 1 <= count <= FIFO_SAMPLES_MAX:
            residual = anchor - prev_anchor - count * dt
            residuals.append(residual)
            current_segment["residuals"].append(residual)

        prev_ts = ts
        prev_anchor = anchor
        prev_device = device_id

    # Endpoint rate per device_id segment: sum(N[k] for k >= 1) / (last_anchor - first_anchor) * 1e6.
    # A span across different device_ids would produce a false rate, so it is never used globally.
    for segment in segments:
        segment_anchors = segment.pop("anchors")
        segment_samples = segment.pop("samples_list")
        segment["residual_us"] = _stats(segment["residuals"])

        if len(segment_anchors) >= 2 and segment_anchors[-1] > segment_anchors[0]:
            span_us = segment_anchors[-1] - segment_anchors[0]
            segment["endpoint_rate_hz"] = sum(segment_samples[1:]) / span_us * 1e6
            segment["publish_rate_hz"] = (len(segment_anchors) - 1) / span_us * 1e6
            segment["anchor_span_us"] = span_us
        else:
            segment["endpoint_rate_hz"] = None
            segment["publish_rate_hz"] = None
            segment["anchor_span_us"] = None

    if len(segments) == 1:
        report["endpoint_rate_hz"] = segments[0]["endpoint_rate_hz"]
        report["publish_rate_hz"] = segments[0]["publish_rate_hz"]
        report["anchor_span_us"] = segments[0]["anchor_span_us"]

    report["samples_total"] = samples_total
    report["device_segments"] = segments
    report["device_id_zero_packets"] = device_zero_packets
    report["samples_per_packet"] = samples_per_packet
    report["dt_us"] = _stats(dt_values)
    report["scale"] = _stats(scale_values)
    report["latency_us"] = _stats(latencies)
    report["residual_us"] = _stats(residuals)
    report["raw"]["int16_limit_samples"] = int16_limit
    report["raw"]["near_int16_limit_samples"] = near_int16_limit
    report["raw"]["min"] = raw_min
    report["raw"]["max"] = raw_max
    report["physical_abs_max"] = physical_abs_max
    report["padding_tail_nonzero_packets"] = padding_nonzero
    report["duplicate_packets"] = duplicates
    report["timestamp_regressions"] = ts_regressions
    report["anchor_regressions"] = anchor_regressions

    if bad_dt:
        findings.append(Finding(SEVERITY_ERROR, "fifo/invalid-dt",
                                "%s has %d packet(s) with non-finite or non-positive dt" % (where, bad_dt), where))
    if bad_scale:
        findings.append(Finding(SEVERITY_ERROR, "fifo/invalid-scale",
                                "%s has %d packet(s) with zero or non-finite scale" % (where, bad_scale), where))
    if bad_samples:
        findings.append(Finding(SEVERITY_WARNING, "fifo/invalid-samples",
                                "%s has %d packet(s) with samples outside 1..%d" % (where, bad_samples, FIFO_SAMPLES_MAX),
                                where))
    if ts_regressions:
        findings.append(Finding(SEVERITY_ERROR, "fifo/timestamp-regression",
                                "%s has %d non-monotonic packet timestamp(s)" % (where, ts_regressions), where))
    if anchor_regressions:
        findings.append(Finding(SEVERITY_WARNING, "fifo/sample-anchor-regression",
                                "%s has %d decreasing timestamp_sample value(s)" % (where, anchor_regressions), where))
    if duplicates:
        findings.append(Finding(SEVERITY_WARNING, "fifo/duplicate-packets",
                                "%s repeats %d packet(s) with identical timestamp and anchor" % (where, duplicates),
                                where))
    if padding_nonzero:
        findings.append(Finding(SEVERITY_INFO, "fifo/padding-tail-nonzero",
                                "%s has %d packet(s) with non-zero values beyond samples (excluded from analysis)"
                                % (where, padding_nonzero), where))
    if int16_limit:
        findings.append(Finding(SEVERITY_INFO, "fifo/raw-at-int16-limit",
                                "%s: %d raw sample(s) reach the int16 numeric limit (+/-32767/-32768). "
                                "This is a numeric-limit observation only; true sensor clipping must be "
                                "confirmed from the driver counters (vehicle_imu_status "
                                "accel_clipping/gyro_clipping)" % (where, int16_limit), where))
    if near_int16_limit:
        findings.append(Finding(SEVERITY_INFO, "fifo/raw-near-int16-limit",
                                "%s: %d raw sample(s) are >= %.0f%% of the int16 limit without reaching it"
                                % (where, near_int16_limit, full_scale_fraction * 100.0), where))

    if device_zero_packets:
        findings.append(Finding(SEVERITY_WARNING, "fifo/device-id-unset",
                                "%s has %d packet(s) with device_id=0: this indicates a missing driver "
                                "device mapping, so the instance-to-chip mapping cannot be trusted"
                                % (where, device_zero_packets), where))

    if residuals:
        beyond = [r for r in residuals if abs(r) > gap_tolerance_us]
        report["residual_us"]["beyond_tolerance"] = len(beyond)
        report["residual_us"]["tolerance_us"] = gap_tolerance_us
        if beyond:
            per_segment = ""
            if len(segments) > 1:
                parts = []
                for segment in segments:
                    values = segment.get("residuals", [])
                    parts.append("%s:%d/%d" % (segment["device_id"],
                                               len([v for v in values if abs(v) > gap_tolerance_us]),
                                               len(values)))
                per_segment = " (per device_id " + ", ".join(parts) + ")"
            findings.append(Finding(SEVERITY_WARNING, "fifo/residual-beyond-tolerance",
                                    "%s: %d/%d packet residual(s) |T[k]-T[k-1]-N[k]*dt[k]| > %.0f us "
                                    "(max |residual| %.0f us)%s. Residuals are only computed inside one "
                                    "device_id segment. This is a timing consistency metric, it does not "
                                    "measure an exact number of dropped samples."
                                    % (where, len(beyond), len(residuals), gap_tolerance_us,
                                       max(abs(r) for r in beyond), per_segment), where))

    if len(segments) > 1:
        findings.append(Finding(SEVERITY_INFO, "fifo/device-change",
                                "%s contains %d device_id segment(s) (%s): rates and residuals are reported "
                                "per segment; a global span across devices would give a false rate"
                                % (where, len(segments), ", ".join(str(s["device_id"]) for s in segments)), where))

    if latencies:
        if min(latencies) < 0:
            findings.append(Finding(SEVERITY_ERROR, "fifo/timestamp-before-anchor",
                                    "%s: driver publication timestamp is earlier than the batch anchor "
                                    "timestamp_sample (min %d us)" % (where, min(latencies)), where))
        if max(latencies) > latency_limit_us:
            findings.append(Finding(SEVERITY_WARNING, "fifo/publication-latency",
                                    "%s max driver publication latency %d us exceeds %d us"
                                    % (where, max(latencies), latency_limit_us), where))

    if samples_per_packet:
        if max(samples_per_packet) == 1 and n_records > 1:
            findings.append(Finding(SEVERITY_INFO, "fifo/single-sample-packets",
                                    "%s publishes 1 sample per packet; endpoint rate equals the publish rate"
                                    % where, where))

    return report, findings


LOGGER_TYPE = {0: "full", 1: "mission"}
BACKEND = {0: "none", 1: "file", 2: "mavlink", 3: "all"}


def analyze_health(content):
    """Summarize logger/health evidence present in the log.

    An empty health section is reported as missing evidence, never as healthy.
    ``message_gaps`` is described as a generation-jump counter, not a loss count.
    """
    findings = []
    health = {"logger_status": [], "vehicle_imu_status": [], "dropouts": {},
              "evidence": []}

    health["dropouts"] = {
        "records": len(content.dropouts),
        "total_ms": sum(content.dropouts),
        "max_ms": max(content.dropouts) if content.dropouts else 0,
    }

    for dataset in content.datasets_named(TOPIC_LOGGER_STATUS):
        where = "%s[%d]" % (dataset.name, dataset.multi_id)
        types = dataset.get("type")
        is_logging = dataset.get("is_logging")
        backends = dataset.get("backend")
        buffer_size = dataset.get("buffer_size_bytes")
        buffer_used = dataset.get("buffer_used_bytes")
        dropouts = dataset.get("dropouts")
        gaps = dataset.get("message_gaps")

        def row_values(values, index):
            return values[index] if index < len(values) else None

        entry = {
            "multi_id": dataset.multi_id,
            "packets": dataset.count,
            "types_seen": sorted(set(types)) if types else [],
            "is_logging_any": any(bool(v) for v in is_logging) if is_logging else None,
            "backends_seen": sorted(set(backends)) if backends else [],
            "buffer_size_max": max(buffer_size) if buffer_size else None,
            "buffer_used_max": max(buffer_used) if buffer_used else None,
            "dropouts_max": max(dropouts) if dropouts else None,
            "message_gaps_max": max(gaps) if gaps else None,
            # gating counters: only rows with type=FULL, is_logging and a file backend are
            # acceptance evidence, so columns from different rows are never combined
            "active_full_file_rows": 0,
            "inactive_rows": 0,
            "mavlink_only_rows": 0,
            "full_log_zero_buffer_rows": 0,
        }
        health["logger_status"].append(entry)

        # Identify the log kind by the message ``type`` field, not the instance number.
        kinds = sorted(set(LOGGER_TYPE.get(t, "unknown(%s)" % t) for t in entry["types_seen"]))
        entry["log_kinds"] = kinds

        max_ratio = None
        max_dropouts_active = 0
        max_gaps_active = 0

        for index in range(dataset.count):
            row_type = row_values(types, index)
            logging_now = bool(row_values(is_logging, index))
            backend = row_values(backends, index) or 0
            size = row_values(buffer_size, index) or 0
            used = row_values(buffer_used, index) or 0
            file_backend = backend in (1, 3)

            if row_type == 0 and logging_now and file_backend:
                entry["active_full_file_rows"] += 1

                if size > 0:
                    ratio = float(used) / float(size)
                    max_ratio = ratio if max_ratio is None else max(max_ratio, ratio)

                else:
                    entry["full_log_zero_buffer_rows"] += 1

                max_dropouts_active = max(max_dropouts_active, row_values(dropouts, index) or 0)
                max_gaps_active = max(max_gaps_active, row_values(gaps, index) or 0)

            elif not logging_now and size == 0:
                entry["inactive_rows"] += 1

            elif logging_now and not file_backend:
                entry["mavlink_only_rows"] += 1

        if max_ratio is not None:
            entry["buffer_fill_ratio_max"] = max_ratio

            if max_ratio > 0.8:
                findings.append(Finding(SEVERITY_WARNING, "health/logger-buffer-pressure",
                                        "%s active full-log row buffer fill reached %.0f%%"
                                        % (where, max_ratio * 100.0), where))

        if entry["full_log_zero_buffer_rows"]:
            findings.append(Finding(SEVERITY_WARNING, "health/full-log-zero-buffer",
                                    "%s: %d active full-log row(s) report buffer_size_bytes=0, so the "
                                    "buffer cannot be assessed for those rows"
                                    % (where, entry["full_log_zero_buffer_rows"]), where))

        if max_dropouts_active:
            findings.append(Finding(SEVERITY_WARNING, "health/logger-dropouts",
                                    "%s reports %d logger dropout(s) (buffer overrun) in active full-log rows"
                                    % (where, max_dropouts_active), where))

        if max_gaps_active:
            findings.append(Finding(SEVERITY_INFO, "health/logger-message-gaps",
                                    "%s reports message_gaps=%d in active full-log rows; this counts "
                                    "generation jump events, not missing samples"
                                    % (where, max_gaps_active), where))

        if entry["inactive_rows"]:
            findings.append(Finding(SEVERITY_INFO, "health/logger-status-inactive",
                                    "%s: %d row(s) are inactive (is_logging=False, buffer_size_bytes=0); "
                                    "not counted as a fault" % (where, entry["inactive_rows"]), where))

        if entry["mavlink_only_rows"]:
            findings.append(Finding(SEVERITY_INFO, "health/logger-status-mavlink-only",
                                    "%s: %d logging row(s) use backend %s (no file backend); buffer "
                                    "occupancy is not assessed for them"
                                    % (where, entry["mavlink_only_rows"],
                                       ", ".join(BACKEND.get(b, str(b)) for b in entry["backends_seen"])),
                                    where))

    if health["logger_status"] and not any(entry["active_full_file_rows"]
                                           for entry in health["logger_status"]):
        findings.append(Finding(SEVERITY_WARNING, "health/no-active-full-file-row",
                                "no logger_status row has type=FULL together with is_logging and a file "
                                "backend: buffer/dropout acceptance evidence is missing (missing evidence, "
                                "not a pass)", "logger_status"))

    for dataset in content.datasets_named(TOPIC_VEHICLE_IMU_STATUS):
        where = "%s[%d]" % (dataset.name, dataset.multi_id)
        entry = {"multi_id": dataset.multi_id, "packets": dataset.count}
        accel_ids = dataset.get("accel_device_id")
        gyro_ids = dataset.get("gyro_device_id")
        entry["accel_device_ids"] = sorted(set(accel_ids)) if accel_ids else []
        entry["gyro_device_ids"] = sorted(set(gyro_ids)) if gyro_ids else []
        for field_name in ("accel_error_count", "gyro_error_count"):
            values = dataset.get(field_name)
            if values:
                entry[field_name + "_max"] = max(values)
                entry[field_name + "_delta"] = max(values) - min(values)
        for field_name in ("accel_clipping", "gyro_clipping"):
            values = dataset.get(field_name)
            if values:
                rows = [v for v in values if isinstance(v, list)]
                if rows:
                    width = len(rows[0])
                    entry[field_name + "_per_axis_max"] = [max(r[i] for r in rows) for i in range(width)]
        for field_name in ("accel_vibration_metric", "gyro_vibration_metric",
                           "accel_rate_hz", "gyro_rate_hz", "accel_raw_rate_hz", "gyro_raw_rate_hz"):
            values = [v for v in dataset.get(field_name) if v is not None]
            if values:
                entry[field_name + "_max"] = max(values)
                entry[field_name + "_median"] = statistics.median(values)
        health["vehicle_imu_status"].append(entry)

        for counter in ("accel_error_count", "gyro_error_count"):
            delta = entry.get(counter + "_delta")
            if delta:
                findings.append(Finding(SEVERITY_WARNING, "health/imu-error-count",
                                        "%s %s increased by %d during the log" % (where, counter, delta), where))
        for clipping in ("accel_clipping", "gyro_clipping"):
            maxima = entry.get(clipping + "_per_axis_max")
            if maxima and any(m > 0 for m in maxima):
                findings.append(Finding(SEVERITY_INFO, "health/imu-clipping",
                                        "%s %s per-axis max %s" % (where, clipping, maxima), where))

    if not health["logger_status"]:
        findings.append(Finding(SEVERITY_WARNING, "health/no-logger-status",
                                "no logger_status messages in the log: buffer/dropout health cannot be judged "
                                "(missing evidence, not a pass)", "topic missing"))
    if not health["vehicle_imu_status"]:
        findings.append(Finding(SEVERITY_WARNING, "health/no-vehicle-imu-status",
                                "no vehicle_imu_status messages: per-IMU clipping/error evidence missing "
                                "(missing evidence, not a pass)", "topic missing"))
    if not content.dropouts and health["logger_status"]:
        health["evidence"].append("no ULog dropout records (O messages) found")
    return health, findings
