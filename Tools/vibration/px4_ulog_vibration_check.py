#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline acceptance check for raw FIFO vibration logs.

Reads a ULog (pyulog when available, otherwise the built-in fallback reader),
reconstructs the FIFO sample streams and reports what was checked. The output is
deliberately explicit about the level of evidence:

  * "no anomaly found at the checked level" is NOT "no samples were lost";
  * a timing residual above tolerance is reported as such, never converted into a
    claimed number of dropped samples;
  * missing health topics are reported as missing evidence, not as healthy.

Usage:
  px4_ulog_vibration_check.py log.ulg [--gap-tolerance-us 2000] [--json out.json]
"""

import argparse
import json
import math
import os
import sys

from vibration_analysis import (
    DEFAULT_FULL_SCALE_FRACTION,
    DEFAULT_GAP_TOLERANCE_US,
    DEFAULT_LATENCY_LIMIT_US,
    KIND_ACCEL,
    KIND_GYRO,
    analyze_fifo_dataset,
    analyze_health,
)
from vibration_common import (
    FIFO_TOPICS,
    MASK_MAX_VALUE,
    PARAM_ACC_MASK,
    PARAM_GYR_MASK,
    PARAM_PROFILE,
    PROFILE_MASK_ACCEL_FIFO,
    PROFILE_MASK_GYRO_FIFO,
    SDLOG_MODE_VALUES,
    SEVERITY_ERROR,
    SEVERITY_INFO,
    SEVERITY_WARNING,
    Finding,
    count_severity,
    format_findings,
    instances_from_mask,
)
from ulog_reader import load_ulog, pyulog_available

EXIT_OK = 0
EXIT_ERRORS = 1
EXIT_USAGE = 2

CHECKS_APPLIED = [
    "samples is the valid prefix length of the x/y/z int16 arrays (1..32); entries at index >= samples are padding and ignored",
    "dt and scale must be finite and non-zero; x/y/z are multiplied by scale (acc m/s^2, gyro rad/s, driver rotation applied, no full calibration)",
    "timestamp_sample anchors the last sample of the batch: t_i = T - (N - 1 - i) * dt",
    "cross-packet residual T[k] - T[k-1] - N[k] * dt[k] is compared against --gap-tolerance-us; it is a consistency metric, not a drop count",
    "endpoint sample rate = sum(N[k] for k >= 1) / (T_last - T_first) * 1e6",
    "device_id changes segment the stream; accel and gyro instances are mapped separately",
    "raw values at int16 full scale are reported as true clipping, values near full scale separately",
    "logger_status is interpreted by its type field (full/mission); an instance with is_logging=False and buffer_size_bytes=0 is not a fault",
    "message_gaps counts generation jump events, not missing samples",
]

CHIP_CAVEATS = [
    "ICM42688P: dt is the last chip timestamp delta of the batch (quantized ~17.0667 us, may vary inside a batch)",
    "ICM20689: accel phase can differ from the gyro by ~125 us, so accel/gyro are not strictly sample synchronous",
]


def expected_selection(log_params, expect_acc_mask, expect_gyr_mask, orb_multi_max=10):
    """Compare the *explicitly requested* masks with the instances present in the log.

    Without an explicit request this returns nothing: the masks stored in the log
    describe the configuration, not a guarantee, so they are handled separately as
    an informational comparison.
    """
    findings = []
    selection = {}

    if expect_acc_mask is None and expect_gyr_mask is None:
        return selection, findings

    plan = ((PARAM_ACC_MASK, expect_acc_mask, "accel"), (PARAM_GYR_MASK, expect_gyr_mask, "gyro"))
    for param_name, explicit, label in plan:
        if explicit is None:
            continue
        raw = explicit
        if raw < 0 or raw > MASK_MAX_VALUE:
            findings.append(Finding(SEVERITY_WARNING, "expect/invalid-mask",
                                    "%s=%d is out of range; the implementation would use 1 for this boot"
                                    % (param_name, raw), param_name))
            raw = 1
        supported = (1 << orb_multi_max) - 1
        effective = raw & supported
        selection[label] = {"raw": raw, "effective": effective,
                            "instances": instances_from_mask(effective)}
    return selection, findings


def compare_log_params_with_streams(content, checked_labels, findings, orb_multi_max=10):
    """Informational: do the masks stored in the log match the instances present?"""
    for label, param_name in (("gyro", PARAM_GYR_MASK), ("accel", PARAM_ACC_MASK)):
        if label in checked_labels:
            continue
        raw = content.params.get(param_name)
        if raw is None or not isinstance(raw, int) or raw < 0 or raw > MASK_MAX_VALUE:
            continue
        topic = "sensor_%s_fifo" % label
        present = sorted(d.multi_id for d in content.datasets_named(topic))
        if not present:
            continue  # the "no FIFO data" warning already covers this case
        selected = instances_from_mask(raw & ((1 << orb_multi_max) - 1))
        missing = [i for i in selected if i not in present]
        extra = [i for i in present if i not in selected]
        if missing or extra:
            findings.append(Finding(SEVERITY_INFO, "params/selection-mismatch",
                                    "%s from the log is %s but %s contains instance(s) %s "
                                    "(missing %s, extra %s): the configuration does not fully match "
                                    "the recorded data" % (param_name, selected, topic, present,
                                                           missing or "none", extra or "none"),
                                    param_name))


def run_check(args):
    findings = []

    if not os.path.exists(args.ulog):
        print("error: file not found: %s" % args.ulog, file=sys.stderr)
        return None, None, EXIT_USAGE

    if args.reader == "pyulog" and not pyulog_available():
        print("error: pyulog is not installed in this interpreter (try --reader native for a limited read)",
              file=sys.stderr)
        return None, None, EXIT_USAGE

    try:
        content = load_ulog(args.ulog, reader=args.reader)
    except Exception as exc:
        print("error: cannot read %s: %s" % (args.ulog, exc), file=sys.stderr)
        return None, None, EXIT_USAGE

    result = {
        "file": {
            "path": os.path.abspath(args.ulog),
            "size_bytes": content.size_bytes,
            "reader": content.reader,
            "start_timestamp_us": content.start_timestamp_us,
            "message_counts": content.counts_by_name(),
            "topics_present": content.topic_names(),
            "unsupported_topics_native_reader": content.unsupported_topics,
            "reader_notes": content.notes[:20],
        },
        "params": {
            PARAM_PROFILE: content.params.get(PARAM_PROFILE),
            PARAM_ACC_MASK: content.params.get(PARAM_ACC_MASK),
            PARAM_GYR_MASK: content.params.get(PARAM_GYR_MASK),
            "SDLOG_MODE": content.params.get("SDLOG_MODE"),
            "IMU_GYRO_RATEMAX": content.params.get("IMU_GYRO_RATEMAX"),
            "IMU_ACCEL_RATEMAX": content.params.get("IMU_ACCEL_RATEMAX"),
        },
        "fifo_streams": [],
        "health": {},
        "findings": [],
        "checks_applied": CHECKS_APPLIED,
        "chip_caveats": CHIP_CAVEATS,
    }

    sdlog_mode = content.params.get("SDLOG_MODE")
    if sdlog_mode is not None:
        result["params"]["SDLOG_MODE_meaning"] = SDLOG_MODE_VALUES.get(sdlog_mode, "unknown")

    profile = content.params.get(PARAM_PROFILE)
    if profile is not None:
        result["params"]["profile_bits"] = {
            "gyro_fifo": bool(profile & PROFILE_MASK_GYRO_FIFO),
            "accel_fifo": bool(profile & PROFILE_MASK_ACCEL_FIFO),
        }

    # ---- FIFO streams ----------------------------------------------------------
    fifo_found = False
    for topic in FIFO_TOPICS:
        datasets = content.datasets_named(topic)
        if not datasets:
            continue
        fifo_found = True
        for dataset in datasets:
            report, stream_findings = analyze_fifo_dataset(
                dataset,
                gap_tolerance_us=args.gap_tolerance_us,
                full_scale_fraction=args.full_scale_fraction,
                latency_limit_us=args.latency_limit_us)
            result["fifo_streams"].append(report)
            findings.extend(stream_findings)

    if not fifo_found:
        hint = ""
        if profile is not None:
            bits = []
            if profile & PROFILE_MASK_GYRO_FIFO:
                bits.append("gyro FIFO bit set")
            if profile & PROFILE_MASK_ACCEL_FIFO:
                bits.append("accel FIFO bit set")
            hint = " (SDLOG_PROFILE=%d: %s)" % (profile, ", ".join(bits) if bits else "no raw FIFO bit set")
        findings.append(Finding(SEVERITY_WARNING, "fifo/no-data",
                                "no sensor_accel_fifo / sensor_gyro_fifo data in this log%s; "
                                "the FIFO acceptance cannot be judged" % hint, "topics missing"))

    # ---- expected instance selection ------------------------------------------
    selection, selection_findings = expected_selection(content.params, args.expect_acc_mask,
                                                       args.expect_gyr_mask, args.orb_multi_max)
    findings.extend(selection_findings)
    result["expected_selection"] = selection
    result["orb_multi_max_instances"] = {
        "value": args.orb_multi_max,
        "note": "user supplied: the instance limit cannot be proven from the log metadata",
    }
    for label, plan in selection.items():
        topic = "sensor_%s_fifo" % label
        present = sorted(d.multi_id for d in content.datasets_named(topic))
        plan["instances_present"] = present
        missing = [i for i in plan["instances"] if i not in present]
        extra = [i for i in present if i not in plan["instances"]]
        if missing:
            findings.append(Finding(SEVERITY_ERROR, "expect/instance-missing",
                                    "%s: instance(s) %s selected but absent from the log (present: %s)"
                                    % (topic, missing, present or "none"), "%s %s" % (PARAM_ACC_MASK, label)))
        if extra:
            findings.append(Finding(SEVERITY_WARNING, "expect/instance-extra",
                                    "%s: instance(s) %s present but not selected by the mask"
                                    % (topic, extra), topic))

    # informational cross-check: masks stored in the log vs streams actually recorded
    compare_log_params_with_streams(content, set(selection.keys()), findings, args.orb_multi_max)

    # reader policy: the native reader is diagnostic only, and its limitations must affect the report
    if content.reader.startswith("native"):
        findings.append(Finding(SEVERITY_INFO, "reader/native-diagnostic",
                                "diagnostic reader in use (%s): a full acceptance run must use pyulog"
                                % content.reader, "reader"))
    if content.unsupported_topics:
        findings.append(Finding(SEVERITY_WARNING, "reader/skipped-topics",
                                "%d topic(s) have field types the native reader cannot decode and were "
                                "skipped: %s. Fields of those topics are missing from this report "
                                "(limited evidence)"
                                % (len(content.unsupported_topics),
                                   ", ".join(sorted(content.unsupported_topics)[:8])), "reader"))
    if content.notes:
        findings.append(Finding(SEVERITY_WARNING, "reader/structural-note",
                                "structural problem(s) while reading: %s"
                                % "; ".join(content.notes[:5]), "reader"))

    # ---- health ---------------------------------------------------------------
    health, health_findings = analyze_health(content)
    result["health"] = health
    findings.extend(health_findings)

    result["findings"] = [f.as_dict() for f in findings]
    result["summary"] = {
        "errors": count_severity(findings, SEVERITY_ERROR),
        "warnings": count_severity(findings, SEVERITY_WARNING),
        "infos": count_severity(findings, SEVERITY_INFO),
        "fifo_streams": len(result["fifo_streams"]),
    }
    exit_code = EXIT_ERRORS if result["summary"]["errors"] else EXIT_OK
    return result, findings, exit_code


def print_report(result, findings, args):
    file_info = result["file"]
    print("PX4 raw-FIFO ULog check")
    print("  file        : %s (%.2f MiB)" % (file_info["path"], file_info["size_bytes"] / 1048576.0))
    print("  reader      : %s%s" % (file_info["reader"],
                                    "" if "pyulog" in str(file_info["reader"]) else
                                    "  [fallback reader: limited to scalar/array fields]"))
    print("  params      : SDLOG_PROFILE=%s SDLOG_ACC_MASK=%s SDLOG_GYR_MASK=%s SDLOG_MODE=%s"
          % tuple(_show(result["params"].get(key)) for key in
                  (PARAM_PROFILE, PARAM_ACC_MASK, PARAM_GYR_MASK, "SDLOG_MODE")))
    counts = file_info["message_counts"]
    print("  structure   : data=%s dropout(O)=%s sync(S)=%s"
          % (counts.get("D data", 0), counts.get("O dropout", 0), counts.get("S sync", 0)))
    print("  tolerance   : residual tolerance %.0f us, full-scale fraction %.2f"
          % (args.gap_tolerance_us, args.full_scale_fraction))
    print()

    print("Raw FIFO streams")
    if not result["fifo_streams"]:
        print("  none")
    for stream in result["fifo_streams"]:
        print("  %s[%d] (%s, %s)" % (stream["topic"], stream["multi_id"], stream["kind"], stream["unit"]))
        print("      packets=%d samples_total=%d samples/packet=%s"
              % (stream["packets"], stream["samples_total"], stream["samples_per_packet"]))
        print("      dt_us=%s scale=%s" % (_fmt_stats(stream["dt_us"]), _fmt_stats(stream["scale"])))
        for segment in stream["device_segments"]:
            residual = segment["residual_us"]
            beyond = " beyond_tolerance=%d" % residual["beyond_tolerance"] if residual.get("beyond_tolerance") else ""
            print("      device_id=%-10s packets=%-4d samples=%-6d endpoint_rate=%s Hz publish_rate=%s Hz residual_us=%s%s"
                  % (segment["device_id"], segment["packets"], segment["samples"],
                     "%.1f" % segment["endpoint_rate_hz"] if segment["endpoint_rate_hz"] else "n/a",
                     "%.1f" % segment["publish_rate_hz"] if segment["publish_rate_hz"] else "n/a",
                     _fmt_stats(residual), beyond))
        if len(stream["device_segments"]) > 1:
            print("      multiple device_id segments: rates and residuals are reported per segment "
                  "(no global span)")
        print("      latency_us=%s raw_at_int16_limit=%d raw_near_int16_limit=%d physical_abs_max=%.4g"
              % (_fmt_stats(stream["latency_us"]), stream["raw"]["int16_limit_samples"],
                 stream["raw"]["near_int16_limit_samples"], stream["physical_abs_max"] or 0.0))
        if stream["device_id_zero_packets"]:
            print("      device_id=0 packets: %d (missing driver mapping)"
                  % stream["device_id_zero_packets"])
        if stream["raw"]["min"] is not None:
            print("      raw range=%d..%d" % (stream["raw"]["min"], stream["raw"]["max"]))
    print()

    print("Health evidence")
    health = result["health"]
    dropouts = health.get("dropouts", {})
    print("  ULog dropouts : %s record(s), total %s ms, max %s ms"
          % (dropouts.get("records"), dropouts.get("total_ms"), dropouts.get("max_ms")))
    if not health.get("logger_status"):
        print("  logger_status : not present")
    for entry in health.get("logger_status", []):
        print("  logger_status[%d]: packets=%d kinds=%s is_logging=%s backend=%s buffer=%s/%s dropouts=%s message_gaps=%s"
              % (entry["multi_id"], entry["packets"], entry.get("log_kinds"),
                 entry.get("is_logging_any"), entry.get("backends_seen"),
                 entry.get("buffer_used_max"), entry.get("buffer_size_max"),
                 entry.get("dropouts_max"), entry.get("message_gaps_max")))
    if not health.get("vehicle_imu_status"):
        print("  vehicle_imu_status: not present")
    for entry in health.get("vehicle_imu_status", []):
        print("  vehicle_imu_status[%d]: packets=%d accel_dev=%s gyro_dev=%s clipping(acc/gyr)=%s/%s errors(acc/gyr)=%s/%s"
              % (entry["multi_id"], entry["packets"], entry.get("accel_device_ids"),
                 entry.get("gyro_device_ids"), entry.get("accel_clipping_per_axis_max"),
                 entry.get("gyro_clipping_per_axis_max"), entry.get("accel_error_count_max"),
                 entry.get("gyro_error_count_max")))
    if result["expected_selection"]:
        print()
        print("Expected selection vs log")
        for label, plan in result["expected_selection"].items():
            print("  %-5s raw=%-6s effective=0x%03x selected=%s present=%s"
                  % (label, plan["raw"], plan["effective"], plan["instances"], plan["instances_present"]))
    print()

    if result["file"]["unsupported_topics_native_reader"]:
        print("Reader limitation: %d topic(s) not decoded by the fallback reader: %s"
              % (len(result["file"]["unsupported_topics_native_reader"]),
                 ", ".join(sorted(result["file"]["unsupported_topics_native_reader"])[:8])))
        print()

    print("Findings")
    if findings:
        print(format_findings(findings))
    else:
        print("  none")
    print()
    print("Checks applied")
    for line in CHECKS_APPLIED:
        print("  - %s" % line)
    print()
    print("Chip caveats (do not turn these into hard assertions)")
    for line in CHIP_CAVEATS:
        print("  - %s" % line)
    print()
    print("Summary: %d error(s), %d warning(s), %d info; %d FIFO stream(s) analyzed"
          % (result["summary"]["errors"], result["summary"]["warnings"],
             result["summary"]["infos"], result["summary"]["fifo_streams"]))
    if result["summary"]["errors"]:
        print("Result: at least one ERROR-level anomaly was found at the checked level.")
    elif result["summary"]["warnings"]:
        print("Result: no ERROR-level anomaly was found, but %d warning(s) need review before this run "
              "is accepted." % result["summary"]["warnings"])
        print("        This is not a statement that no samples were lost.")
    else:
        print("Result: no anomaly found at the checked level. This is not a statement that no")
        print("        samples were lost, nor a flight-performance or CPU margin result.")


def _show(value):
    return "<absent>" if value is None else value


def _fmt_stats(stats):
    if not stats or not stats.get("count"):
        return "n/a"
    return "n=%d min=%.3f med=%.3f max=%.3f" % (stats["count"], stats["min"], stats["median"], stats["max"])


def _json_scalar(value):
    """Keep numpy scalars numeric in JSON rather than converting them to text."""
    if hasattr(value, "item"):
        return value.item()
    raise TypeError("unsupported JSON value: %s" % type(value).__name__)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline raw-FIFO ULog acceptance check")
    parser.add_argument("ulog", help="ULog file produced with the raw FIFO profile enabled")
    parser.add_argument("--reader", choices=("auto", "pyulog", "native"), default="auto",
                        help="data reader (default: pyulog when installed, else the fallback reader)")
    parser.add_argument("--gap-tolerance-us", type=float, default=DEFAULT_GAP_TOLERANCE_US,
                        help="tolerance for |T[k]-T[k-1]-N[k]*dt[k]| (default %.0f us)"
                             % DEFAULT_GAP_TOLERANCE_US)
    parser.add_argument("--full-scale-fraction", type=float, default=DEFAULT_FULL_SCALE_FRACTION,
                        help="fraction of int16 full scale counted as 'near full scale' (default %.2f)"
                             % DEFAULT_FULL_SCALE_FRACTION)
    parser.add_argument("--latency-limit-us", type=float, default=DEFAULT_LATENCY_LIMIT_US,
                        help="warn above this publication latency (default %.0f us)" % DEFAULT_LATENCY_LIMIT_US)
    parser.add_argument("--expect-acc-mask", type=int, help="SDLOG_ACC_MASK the log is expected to follow")
    parser.add_argument("--expect-gyr-mask", type=int, help="SDLOG_GYR_MASK the log is expected to follow")
    parser.add_argument("--orb-multi-max", type=int, default=10,
                        help="build instance limit used when evaluating masks (1..10, default 10). "
                             "The value is user supplied and cannot be proven from log metadata")
    parser.add_argument("--json", help="write the full result as JSON to this path ('-' for stdout)")
    args = parser.parse_args(argv)

    if not 1 <= args.orb_multi_max <= 10:
        print("error: --orb-multi-max must be within 1..10", file=sys.stderr)
        return EXIT_USAGE
    if not math.isfinite(args.gap_tolerance_us) or args.gap_tolerance_us < 0:
        print("error: --gap-tolerance-us must be a finite value >= 0", file=sys.stderr)
        return EXIT_USAGE
    if not math.isfinite(args.latency_limit_us) or args.latency_limit_us <= 0:
        print("error: --latency-limit-us must be a finite value > 0", file=sys.stderr)
        return EXIT_USAGE
    if not math.isfinite(args.full_scale_fraction) or not 0 < args.full_scale_fraction <= 1:
        print("error: --full-scale-fraction must be a finite value in (0, 1]", file=sys.stderr)
        return EXIT_USAGE

    result, findings, exit_code = run_check(args)
    if result is None:
        return exit_code

    if args.json:
        payload = json.dumps(result, indent=2, sort_keys=True, default=_json_scalar)
        if args.json == "-":
            print(payload)
        else:
            with open(args.json, "w", encoding="utf-8") as handle:
                handle.write(payload + "\n")
            print("JSON written to %s" % args.json)

    print_report(result, findings, args)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
