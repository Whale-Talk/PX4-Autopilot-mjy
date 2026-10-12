#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline checker for the raw-FIFO logging configuration.

It mirrors the behaviour of
  * src/modules/logger/params.c                (SDLOG_PROFILE / SDLOG_ACC_MASK / SDLOG_GYR_MASK)
  * src/modules/logger/logged_topics.cpp       (instance masks, logger_topics.txt priority)

Important semantics reproduced on purpose:
  * masks are only applied when the matching SDLOG_PROFILE FIFO bit is set;
  * an invalid mask value (< 0 or > 1023) makes the implementation warn once and use 1
    for this boot; the stored parameter is not rewritten;
  * bits above the build limit (ORB_MULTI_MAX_INSTANCES) are dropped with a warning,
    and if nothing remains no instance is selected (never "all instances");
  * logger_topics.txt replaces the profile topic list as soon as **any** line parses,
    even when every topic name is unknown (the original ``ntopics > 0`` test).

Usage examples:
  px4_vibration_config_check.py --params params.txt --topics-file logger_topics.txt
  px4_vibration_config_check.py --profile 8 --acc-mask 3 --orb-multi-max 4
"""

import argparse
import json
import os
import re
import sys

from vibration_common import (
    MASK_MAX_VALUE,
    MASK_MIN_VALUE,
    ORB_MULTI_MAX_INSTANCES_DEFAULT,
    PARAM_ACC_MASK,
    PARAM_GYR_MASK,
    PARAM_PROFILE,
    PROFILE_BIT_ACCEL_FIFO,
    PROFILE_BIT_GYRO_FIFO,
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

EXIT_OK = 0
EXIT_ERRORS = 1
EXIT_USAGE = 2

# real `param show` line: "<used> <saved> NAME [index,used] : VALUE"
# (src/systemcmds/param/param.cpp:680 prints "%c %c %s [%d,%d] : ")
_PARAM_SHOW_RE = re.compile(
    r"^\s*(?:\S+\s+)?(?:\S+\s+)?(?P<name>[A-Za-z0-9_]+)\s*\[\s*-?\d+\s*,\s*-?\d+\s*\]\s*:"
    r"\s*(?P<value>.*)$")
_INT_RE = re.compile(r"^[+-]?\d+$")
_DECIMAL_RE = re.compile(r"^[+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?$")
_UINT_PREFIX_RE = re.compile(r"^[+-]?\d+")

# firmware limits (src/modules/logger/logged_topics.cpp)
FGETS_LINE_BYTES = 80
TOPIC_NAME_BYTES = 80
UINT16_MAX_VALUE = 65535
UINT8_MAX_VALUE = 255


def _sscanf_uint(token):
    """Emulate sscanf('%u') on one token: return (value, rest, was_negative)."""
    match = _UINT_PREFIX_RE.match(token)

    if not match:
        return None, token, False

    text = match.group(0)
    negative = text.startswith("-")
    value = abs(int(text))

    if negative:
        value = (1 << 32) - value

    return value, token[match.end():], negative


def as_int(value):
    """Return the value only when it is a usable integer, else None."""
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def parse_param_value(text):
    """Return ('int', v) | ('float', v) | ('text', raw).

    A fractional value is kept as a float: it is never silently truncated, because
    ``int(float(x))`` would turn a malformed 1.5 into an accepted 1.
    """
    text = (text or "").strip()
    if _INT_RE.match(text):
        return "int", int(text)
    if _DECIMAL_RE.match(text):
        value = float(text)
        # a float parameter is printed with %4.4f, so an integral value is still an int
        return ("int", int(value)) if value.is_integer() else ("float", value)
    return "text", text


def parse_params_file(path, diagnostics=None):
    """Parse parameter files.

    Supported inputs:
      * real ``param show`` output: ``<used> <saved|default|modified> NAME [index,used] : VALUE``
        (see src/systemcmds/param/param.cpp:680, which prints "%c %c %s [%d,%d] : ");
      * simple ``NAME VALUE`` and ``NAME: VALUE`` files.

    Values are never truncated and never guessed: unsupported text is kept verbatim
    and reported by the caller (relevant parameters then become an ERROR).
    """
    params = {}
    unparsed = []

    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_no, raw in enumerate(handle, 1):
            line = raw.strip()

            if not line or line.startswith("#"):
                continue

            match = _PARAM_SHOW_RE.match(line)

            if match:
                name, value_text = match.group("name"), match.group("value")

            elif ":" in line:
                left, right = line.split(":", 1)
                left_bits = left.strip().split()
                name, value_text = (left_bits[-1] if left_bits else ""), right

            else:
                bits = line.split(None, 1)
                name = bits[0]
                value_text = bits[1] if len(bits) > 1 else ""

            name = name.strip().rstrip(",")

            if not name:
                unparsed.append((line_no, "", line))
                continue

            kind, value = parse_param_value(value_text)

            if kind == "text":
                params[name] = value
                unparsed.append((line_no, name, value))

            else:
                params[name] = value

    if diagnostics is not None:
        diagnostics.extend(unparsed)

    return params


def parse_topics_file(path):
    """Parse logger_topics.txt, flagging what cannot be emulated exactly.

    Returns (entries, ntopics). For ordinary input the firmware semantics are
    reproduced: ``ntopics`` counts **every** line whose first token parses, even
    when the topic name is unknown, because add_topic_multi() always returns true
    (logged_topics.cpp). No general sscanf emulation is attempted; lines that need
    it are reported as ``ambiguous`` instead of being silently interpreted:

      * 79+ byte lines: the firmware ``fgets`` buffer would split them;
      * numeric fields with trailing characters (sscanf stops at the numeric prefix);
      * negative values in the unsigned fields (wrap-around is modelled, not verified);
      * values beyond the uint16 interval / uint8 instance narrowing of add_topic();
      * topic names of 80+ bytes (firmware buffer);
      * more than three tokens (sscanf reads at most three).
    """
    entries = []

    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_no, raw in enumerate(handle, 1):

            # C code: skip if strlen(line) < 2 or line[0] == '#'
            if len(raw) < 2 or raw[0] == "#":
                continue

            token_line = raw.strip()

            if not token_line:
                continue

            ambiguous = []
            content = raw.rstrip("\n").rstrip("\r")

            if len(content) >= FGETS_LINE_BYTES - 1:
                ambiguous.append("%d byte line would be split by the firmware fgets (buffer %d)"
                                 % (len(content), FGETS_LINE_BYTES))

            tokens = token_line.split()
            name = tokens[0]

            if name.endswith(","):
                name = name[:-1]

            if len(name) >= TOPIC_NAME_BYTES:
                ambiguous.append("%d byte topic name exceeds the firmware %d byte buffer"
                                 % (len(name), TOPIC_NAME_BYTES))

            interval = None
            instance = None
            nfields = 1

            if len(tokens) >= 2:
                value, rest, negative = _sscanf_uint(tokens[1])

                if value is not None:
                    nfields = 2

                    if rest:
                        ambiguous.append("interval token %r has trailing characters: sscanf stops "
                                         "after the numeric prefix" % tokens[1])
                    if negative:
                        ambiguous.append("negative interval %r: unsigned wrap is modelled" % tokens[1])
                    if value > UINT16_MAX_VALUE:
                        ambiguous.append("interval %d exceeds the uint16 narrowing of add_topic()" % value)

                    interval = value & UINT16_MAX_VALUE

                    if len(tokens) >= 3:
                        value2, rest2, negative2 = _sscanf_uint(tokens[2])

                        if value2 is not None:
                            nfields = 3

                            if rest2:
                                ambiguous.append("instance token %r has trailing characters" % tokens[2])
                            if negative2:
                                ambiguous.append("negative instance %r: unsigned wrap is modelled" % tokens[2])
                            if value2 > UINT8_MAX_VALUE:
                                ambiguous.append("instance %d exceeds the uint8 narrowing of add_topic()" % value2)

                            instance = value2 & UINT8_MAX_VALUE

                        else:
                            # a failed third conversion leaves nfields at 2, not 1
                            ambiguous.append("instance token %r does not parse: sscanf leaves nfields=2"
                                             % tokens[2])

                if len(tokens) > 3:
                    ambiguous.append("%d tokens: sscanf reads at most three" % len(tokens))

            entries.append({"line": line_no, "name": name, "interval_ms": interval,
                            "instance": instance, "nfields": nfields, "ambiguous": ambiguous})

    return entries, len(entries)


def validate_mask(raw_value, orb_multi_max, findings, param_name, label):
    """Return (effective_mask, enabled_instances) after reproducing the C validation."""
    if raw_value is None:
        raw_value = 1
        findings.append(Finding(SEVERITY_INFO, "param/default",
                                "%s not given, using the firmware default 1 (%s instance 0 only)"
                                % (param_name, label), param_name))

    if raw_value < MASK_MIN_VALUE or raw_value > MASK_MAX_VALUE:
        findings.append(Finding(SEVERITY_ERROR, "param/mask-out-of-range",
                                "%s=%d is outside %d..%d: the implementation warns once and uses 1 "
                                "for this boot (the stored parameter is not rewritten)"
                                % (param_name, raw_value, MASK_MIN_VALUE, MASK_MAX_VALUE), param_name))
        return 1, [0]

    effective = raw_value
    supported_mask = (1 << orb_multi_max) - 1

    if effective & ~supported_mask:
        clamped = effective & supported_mask
        findings.append(Finding(SEVERITY_WARNING, "param/mask-clamped",
                                "%s=0x%03x selects instance bit(s) above the build limit %d; "
                                "the implementation drops them and uses 0x%03x"
                                % (param_name, effective, orb_multi_max, clamped), param_name))
        effective = clamped
        if effective == 0:
            findings.append(Finding(SEVERITY_WARNING, "param/mask-clamped-to-zero",
                                    "%s clamps to 0: no %s instance is selected. It is NOT expanded to "
                                    "all instances" % (param_name, label), param_name))

    return effective, instances_from_mask(effective)


def run_check(args):
    findings = []
    params = {}
    param_diagnostics = []
    if args.params:
        if not os.path.exists(args.params):
            return None, [Finding(SEVERITY_ERROR, "io/params-missing",
                                  "parameter file not found: %s" % args.params, args.params)], EXIT_USAGE
        params = parse_params_file(args.params, param_diagnostics)

    # Values read from a file are validated before use: a malformed or fractional value
    # must be an explicit ERROR (no silent default, no int(float()) truncation, no
    # bitwise operation on a string).
    raw_profile = params.get(PARAM_PROFILE)
    raw_acc = params.get(PARAM_ACC_MASK)
    raw_gyr = params.get(PARAM_GYR_MASK)
    raw_mode = params.get("SDLOG_MODE")

    for name, value in ((PARAM_PROFILE, raw_profile), (PARAM_ACC_MASK, raw_acc),
                        (PARAM_GYR_MASK, raw_gyr), ("SDLOG_MODE", raw_mode)):
        if value is None:
            continue

        if isinstance(value, float):
            findings.append(Finding(SEVERITY_ERROR, "param/not-integer",
                                    "%s is %s in the parameter file: a fractional value is not truncated "
                                    "and cannot be applied; re-check `param show` output" % (name, value), name))

        elif not isinstance(value, int):
            findings.append(Finding(SEVERITY_ERROR, "param/unparseable",
                                    "%s has the unparsed value %r in the parameter file: its effect cannot "
                                    "be judged; re-check `param show` output" % (name, value), name))

    for line_no, name, value in param_diagnostics:
        if name in (PARAM_PROFILE, PARAM_ACC_MASK, PARAM_GYR_MASK, "SDLOG_MODE"):
            continue  # already reported as an ERROR above

        findings.append(Finding(SEVERITY_INFO, "param/unparsed-line",
                                "parameter line %d (%s) could not be read as a number: %r "
                                "(not used by this check)" % (line_no, name or "<no name>", value),
                                "params file"))

    profile = args.profile if args.profile is not None else as_int(raw_profile)
    acc_raw = args.acc_mask if args.acc_mask is not None else as_int(raw_acc)
    gyr_raw = args.gyr_mask if args.gyr_mask is not None else as_int(raw_gyr)
    sdlog_mode = as_int(raw_mode)

    if profile is None:
        profile = 1

        if raw_profile is not None or args.profile is not None:
            findings.append(Finding(SEVERITY_INFO, "param/profile-unusable",
                                    "%s has no usable integer value: the profile-bit checks use the firmware "
                                    "default 1 for reference only" % PARAM_PROFILE, PARAM_PROFILE))
        else:
            findings.append(Finding(SEVERITY_INFO, "param/profile-default",
                                    "%s not given, assuming the firmware default 1 (default set only)"
                                    % PARAM_PROFILE, PARAM_PROFILE))

    gyro_enabled = bool(profile & PROFILE_MASK_GYRO_FIFO)
    accel_enabled = bool(profile & PROFILE_MASK_ACCEL_FIFO)

    result = {
        "inputs": {
            "params_file": args.params,
            "topics_file": args.topics_file,
            "orb_multi_max_instances": args.orb_multi_max,
        },
        "profile": profile,
        "profile_bits": {
            "gyro_fifo_bit_%d" % PROFILE_BIT_GYRO_FIFO: gyro_enabled,
            "accel_fifo_bit_%d" % PROFILE_BIT_ACCEL_FIFO: accel_enabled,
        },
        "masks": {},
        "topics_file": {},
        "findings": [],
    }

    if sdlog_mode is not None:
        result["sdlog_mode"] = {"value": sdlog_mode,
                                "meaning": SDLOG_MODE_VALUES.get(sdlog_mode, "unknown")}
        if sdlog_mode == -1:
            findings.append(Finding(SEVERITY_ERROR, "sdlog/mode-disabled",
                                    "SDLOG_MODE=-1 disables logging entirely; no raw FIFO data will be written",
                                    "SDLOG_MODE"))
        elif sdlog_mode == 0 and args.expect_logging_without_arming:
            findings.append(Finding(SEVERITY_WARNING, "sdlog/mode-needs-arming",
                                    "SDLOG_MODE=0 logs only while armed; for a bench capture use "
                                    "SDLOG_MODE=1 (boot until disarm) or 2 (boot until shutdown)", "SDLOG_MODE"))

    checks = ((PARAM_GYR_MASK, "gyro", gyr_raw, gyro_enabled),
              (PARAM_ACC_MASK, "accel", acc_raw, accel_enabled))

    for param_name, label, raw_value, enabled in checks:
        effective, instances = validate_mask(raw_value, args.orb_multi_max, findings, param_name, label)
        result["masks"][param_name] = {
            "raw": raw_value,
            "effective": effective,
            "effective_hex": "0x%03x" % effective,
            "instances": instances,
            "profile_bit_enabled": enabled,
            "applied": enabled,
        }
        if not enabled:
            if instances:
                findings.append(Finding(SEVERITY_WARNING, "mask/without-profile-bit",
                                        "%s selects instance(s) %s but SDLOG_PROFILE bit %d is not set: "
                                        "the mask has no effect until the bit is enabled"
                                        % (param_name, instances, PROFILE_BIT_GYRO_FIFO if label == "gyro"
                                           else PROFILE_BIT_ACCEL_FIFO), param_name))
        else:
            if not instances:
                findings.append(Finding(SEVERITY_WARNING, "mask/empty-selection",
                                        "%s selects no instance while SDLOG_PROFILE bit %d is set: the stock "
                                        "fast-capture behaviour (raised logger priority, 800 us log interval) "
                                        "still applies. Clear the profile bit and reboot to leave the experiment."
                                        % (param_name, PROFILE_BIT_GYRO_FIFO if label == "gyro"
                                           else PROFILE_BIT_ACCEL_FIFO), param_name))
            else:
                findings.append(Finding(SEVERITY_INFO, "mask/selection",
                                        "%s: raw %s FIFO logging will subscribe instance(s) %s at full rate"
                                        % (param_name, label, instances), param_name))

    # --- logger_topics.txt priority ---------------------------------------------
    replacement_active = False
    if args.topics_file:
        if not os.path.exists(args.topics_file):
            findings.append(Finding(SEVERITY_INFO, "topics-file/absent",
                                    "no %s on the SD card: the SDLOG_PROFILE topic list is used"
                                    % args.topics_file, args.topics_file))
        else:
            entries, ntopics = parse_topics_file(args.topics_file)
            known = None
            if args.known_topics:
                known = set()
                with open(args.known_topics, "r", encoding="utf-8", errors="replace") as handle:
                    for line in handle:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            known.add(line.split()[0])
            unknown = [e["name"] for e in entries if known is not None and e["name"] not in known]
            ambiguous = [e for e in entries if e["ambiguous"]]
            result["topics_file"] = {
                "path": args.topics_file,
                "parsed_lines": ntopics,
                "replacement_active": ntopics > 0,
                "entries": entries,
                "unknown_topics": unknown,
                "ambiguous_lines": [{"line": e["line"], "name": e["name"], "reasons": e["ambiguous"]}
                                    for e in ambiguous],
            }
            replacement_active = ntopics > 0

            if ambiguous:
                findings.append(Finding(SEVERITY_WARNING, "topics-file/ambiguous-line",
                                        "%d line(s) use syntax this checker does not emulate exactly: %s. "
                                        "Review them manually (the ntopics>0 replacement count above is only "
                                        "asserted for ordinary input)"
                                        % (len(ambiguous),
                                           "; ".join("line %d (%s): %s" % (e["line"], e["name"],
                                                                           ", ".join(e["ambiguous"]))
                                                     for e in ambiguous[:3])),
                                        args.topics_file))

            if ntopics > 0:
                findings.append(Finding(SEVERITY_INFO, "topics-file/replaces-profile",
                                        "%s parsed %d line(s): it replaces the SDLOG_PROFILE topic list "
                                        "(including when a line names an unknown topic, which is the original "
                                        "ntopics > 0 semantics)" % (args.topics_file, ntopics), args.topics_file))
                if unknown:
                    findings.append(Finding(SEVERITY_WARNING, "topics-file/unknown-topics",
                                            "%d parsed line(s) name topics not in the known list: %s. They still "
                                            "count as replacement, but they add no subscription."
                                            % (len(unknown), ", ".join(sorted(set(unknown))[:8])),
                                            args.topics_file))
                if known is not None and len(unknown) == len(entries):
                    findings.append(Finding(SEVERITY_WARNING, "topics-file/all-unknown",
                                            "every parsed line names an unknown topic: the replacement is active "
                                            "but the subscription set ends up empty, so initialize_logged_topics() "
                                            "returns false. This is NOT 'no replacement'", args.topics_file))
                if gyro_enabled or accel_enabled:
                    if (result["masks"][PARAM_ACC_MASK]["raw"] not in (None, 1)
                            or result["masks"][PARAM_GYR_MASK]["raw"] not in (None, 1)):
                        findings.append(Finding(SEVERITY_WARNING, "topics-file/masks-ignored",
                                                "a non-default instance mask is configured together with "
                                                "%s: the file replaces the profile, so the masks are ignored"
                                                % args.topics_file, args.topics_file))
                    findings.append(Finding(SEVERITY_WARNING, "topics-file/no-health-topics",
                                            "the file replace path does not add the logger_status health "
                                            "subscriptions: add them to the file if you need them", args.topics_file))
            else:
                findings.append(Finding(SEVERITY_INFO, "topics-file/no-parsed-lines",
                                        "%s exists but no line parses: the SDLOG_PROFILE topic list is used"
                                        % args.topics_file, args.topics_file))

    result["replacement_active"] = replacement_active
    result["findings"] = [f.as_dict() for f in findings]
    result["summary"] = {
        "errors": count_severity(findings, SEVERITY_ERROR),
        "warnings": count_severity(findings, SEVERITY_WARNING),
        "infos": count_severity(findings, SEVERITY_INFO),
    }
    exit_code = EXIT_ERRORS if result["summary"]["errors"] else EXIT_OK
    return result, findings, exit_code


def print_report(result, findings):
    inputs = result["inputs"]
    print("PX4 raw-FIFO logging configuration check")
    print("  params file      : %s" % (inputs["params_file"] or "<not given>"))
    print("  topics file      : %s" % (inputs["topics_file"] or "<not given>"))
    print("  ORB_MULTI_MAX_INSTANCES (build limit): %d" % inputs["orb_multi_max_instances"])
    print("  SDLOG_PROFILE    : %d  (gyro FIFO bit %d: %s, accel FIFO bit %d: %s)"
          % (result["profile"], PROFILE_BIT_GYRO_FIFO,
             "set" if result["profile_bits"]["gyro_fifo_bit_%d" % PROFILE_BIT_GYRO_FIFO] else "clear",
             PROFILE_BIT_ACCEL_FIFO,
             "set" if result["profile_bits"]["accel_fifo_bit_%d" % PROFILE_BIT_ACCEL_FIFO] else "clear"))
    if "sdlog_mode" in result:
        print("  SDLOG_MODE       : %s (%s)" % (result["sdlog_mode"]["value"], result["sdlog_mode"]["meaning"]))
    print()
    print("Effective instance selection")
    print("  %-16s %-8s %-10s %-9s %s" % ("parameter", "raw", "effective", "applied", "instances"))
    for name, entry in result["masks"].items():
        print("  %-16s %-8s %-10s %-9s %s"
              % (name, entry["raw"], entry["effective_hex"],
                 "yes" if entry["applied"] else "no", entry["instances"] or "-"))
    print()
    print("  logger_topics.txt replacement active: %s" % ("yes" if result["replacement_active"] else "no"))
    print()
    if findings:
        print("Findings")
        print(format_findings(findings))
    else:
        print("Findings: none")
    print()
    print("Summary: %d error(s), %d warning(s), %d info"
          % (result["summary"]["errors"], result["summary"]["warnings"], result["summary"]["infos"]))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Check SDLOG_PROFILE / SDLOG_ACC_MASK / SDLOG_GYR_MASK and logger_topics.txt consistency",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--params", help="parameter dump (one 'NAME VALUE' or 'NAME: VALUE' per line)")
    parser.add_argument("--profile", type=int, help="SDLOG_PROFILE value (overrides --params)")
    parser.add_argument("--acc-mask", type=int, help="SDLOG_ACC_MASK value (overrides --params)")
    parser.add_argument("--gyr-mask", type=int, help="SDLOG_GYR_MASK value (overrides --params)")
    parser.add_argument("--topics-file", help="path to the SD card logger_topics.txt to check")
    parser.add_argument("--known-topics", help="optional file with one known topic name per line "
                                              "(enables unknown-topic detection)")
    parser.add_argument("--orb-multi-max", type=int, default=ORB_MULTI_MAX_INSTANCES_DEFAULT,
                        help="build limit: 10 normally, 4 with CONSTRAINED_MEMORY (default 10)")
    parser.add_argument("--expect-logging-without-arming", action="store_true",
                        help="treat SDLOG_MODE=0 as a problem (bench capture without arming)")
    parser.add_argument("--json", help="write the full result as JSON to this path ('-' for stdout)")
    args = parser.parse_args(argv)

    if args.orb_multi_max < 1 or args.orb_multi_max > 10:
        print("error: --orb-multi-max must be within 1..10", file=sys.stderr)
        return EXIT_USAGE

    result, findings, exit_code = run_check(args)
    if result is None:
        print(format_findings(findings), file=sys.stderr)
        return exit_code

    if args.json:
        payload = json.dumps(result, indent=2, sort_keys=True)
        if args.json == "-":
            print(payload)
        else:
            with open(args.json, "w", encoding="utf-8") as handle:
                handle.write(payload + "\n")
            print("JSON written to %s" % args.json)

    print_report(result, findings)

    print()
    print("Note: this checker covers the configuration layer only. It cannot observe flight")
    print("      behaviour, CPU load or SD write performance; those need a device test.")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
