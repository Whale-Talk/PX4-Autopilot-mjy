#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Local tests for the raw-FIFO vibration tooling.

Everything here runs with the standard library only. Tests that need pyulog or a
real PX4 log skip with an explicit message instead of failing:

  * pyulog cross-check      -> skipped when pyulog is not installed
  * real-log cross-check    -> skipped unless PX4_VIBRATION_REAL_ULOG points at a ULog

Run:  python3 Tools/vibration/test_vibration_tools.py -v
"""

import contextlib
import io
import json
import math
import os
import shutil
import struct
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import make_synthetic_ulog as synth                      # noqa: E402
import px4_ulog_vibration_check as ulog_check             # noqa: E402
import px4_vibration_config_check as config_check         # noqa: E402
import ulog_reader                                        # noqa: E402
from vibration_analysis import analyze_fifo_dataset, analyze_health  # noqa: E402

REAL_ULOG = os.environ.get("PX4_VIBRATION_REAL_ULOG", "")


def run_cli(main_function, argv):
    """Invoke a CLI main() and capture stdout + exit code."""
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        code = main_function(argv)
    return code, buffer.getvalue()


def add_logged_records(path):
    """Return [(payload_len, name_bytes)] for every ADD_LOGGED message."""
    with open(path, "rb") as handle:
        buf = handle.read()
    records = []
    offset = 16
    while offset + 3 <= len(buf):
        payload_len, msg_type = struct.unpack_from("<HB", buf, offset)
        if payload_len < 1 or offset + 3 + payload_len > len(buf):
            break
        payload = buf[offset + 3:offset + 3 + payload_len]
        if msg_type == ulog_reader.MSG_ADD_LOGGED:
            records.append((payload_len, payload[3:]))
        offset += 3 + payload_len
    return records


def fifo_dataset(rows, name="sensor_gyro_fifo", multi_id=0):
    """Build a Dataset in memory for analysis-level tests."""
    dataset = ulog_reader.Dataset(name, multi_id, 1, {})
    for key in ("timestamp", "timestamp_sample", "device_id", "dt", "scale", "samples", "x", "y", "z"):
        dataset.fields[key] = []
    for row in rows:
        dataset.fields["timestamp"].append(row["timestamp"])
        dataset.fields["timestamp_sample"].append(row["anchor"])
        dataset.fields["device_id"].append(row["device_id"])
        dataset.fields["dt"].append(row.get("dt", 125.0))
        dataset.fields["scale"].append(row.get("scale", 0.001))
        count = row.get("samples", 20)
        dataset.fields["samples"].append(count)
        values = [100] * 32
        dataset.fields["x"].append(list(values))
        dataset.fields["y"].append(list(values))
        dataset.fields["z"].append(list(values))
        dataset.count += 1
    return dataset


def finding_codes(findings):
    return sorted({f.code for f in findings})


def data_payload_lengths(path):
    """Yield (topic, payload_len) for every data message of a ULog file.

    Independent of the writer: sizes come from the format strings stored in the
    file itself.
    """
    with open(path, "rb") as handle:
        buf = handle.read()
    formats = {}
    subs = {}
    out = []
    offset = 16
    while offset + 3 <= len(buf):
        payload_len, msg_type = struct.unpack_from("<HB", buf, offset)
        if payload_len < 1 or offset + 3 + payload_len > len(buf):
            break
        payload = buf[offset + 3:offset + 3 + payload_len]
        if msg_type == ulog_reader.MSG_FORMAT:
            name, fields = ulog_reader._parse_format(payload.decode("utf-8", "replace").rstrip("\x00"))
            if name:
                formats[name] = fields
        elif msg_type == ulog_reader.MSG_ADD_LOGGED:
            msg_id = struct.unpack_from("<H", payload, 1)[0]
            subs[msg_id] = payload[3:].split(b"\x00", 1)[0].decode("utf-8", "replace")
        elif msg_type == ulog_reader.MSG_DATA:
            msg_id = struct.unpack_from("<H", payload, 0)[0]
            name = subs.get(msg_id)
            fields = formats.get(name) or [] if name else []
            # topics with nested message fields cannot be sized here (the fallback reader
            # skips them as well); leave them out of this convention check
            sized = not any(not f.is_padding and f.type_name not in ulog_reader.BASE_TYPES
                            for f in fields)
            if name and sized:
                out.append((name, payload_len, fields))
        offset += 3 + payload_len
    return out


class SyntheticFileMixin(object):
    @classmethod
    def setUpClass(cls):
        cls.tmpdir = tempfile.mkdtemp(prefix="vibration-tools-")
        cls.paths = {}
        for name in synth.SCENARIOS:
            cls.paths[name] = synth.build(name, os.path.join(cls.tmpdir, "%s.ulg" % name))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmpdir, ignore_errors=True)


class TestSyntheticWriter(SyntheticFileMixin, unittest.TestCase):
    def test_native_reader_reads_all_scenarios(self):
        for name, path in self.paths.items():
            content = ulog_reader.load_ulog(path, reader="native")
            self.assertTrue(content.reader.startswith("native"), name)
            self.assertTrue(content.datasets, "%s: no datasets decoded" % name)

    def test_nominal_streams_and_values(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        self.assertEqual(sorted(set(n for n, _ in content.datasets)),
                         ["logger_status", "sensor_accel_fifo", "sensor_gyro_fifo", "vehicle_imu_status"])
        gyro0 = content.dataset("sensor_gyro_fifo", 0)
        self.assertEqual(gyro0.count, 20)
        # packet 0 uses 19 samples, packet 1 uses 20 (generator pattern)
        self.assertEqual(int(gyro0.get("samples")[0]), 19)
        self.assertEqual(int(gyro0.get("samples")[1]), 20)
        self.assertEqual(int(gyro0.get("device_id")[0]), 0x36001A)
        self.assertAlmostEqual(float(gyro0.get("dt")[0]), 125.0, places=6)
        self.assertEqual(content.dataset("sensor_gyro_fifo", 1).count, 20)
        self.assertEqual(content.dataset("sensor_accel_fifo", 0).count, 20)

    def test_padding_fields_declared_and_not_written(self):
        """Format strings must declare _padding* and data records must omit them."""
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        _c, _s, formats = ulog_reader.scan_structure(self.paths["nominal"], decode=False)
        self.assertTrue(any(field.is_padding for fields in formats.values() for field in fields),
                        "no _padding field declared: not mimicking real PX4 serialization")

        checked = 0
        for topic, payload_len, fields in data_payload_lengths(self.paths["nominal"]):
            self.assertTrue(fields, topic)
            expected = 2 + sum(ulog_reader.BASE_TYPES[f.type_name][1] * f.count
                               for f in fields if not f.is_padding)
            self.assertEqual(payload_len, expected,
                             "%s: data record length %d != fields-without-padding+msg_id %d"
                             % (topic, payload_len, expected))
            checked += 1
        self.assertGreater(checked, 40)

    def test_params_are_stored(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        self.assertEqual(content.params.get("SDLOG_PROFILE"), 769)
        self.assertEqual(content.params.get("SDLOG_ACC_MASK"), 3)
        self.assertEqual(content.params.get("SDLOG_GYR_MASK"), 3)

    def test_add_logged_has_no_nul_terminator(self):
        """pyulog decodes data[3:] verbatim: a NUL would end up inside the topic name."""
        records = add_logged_records(self.paths["nominal"])
        self.assertGreater(len(records), 0)
        for payload_len, name_bytes in records:
            self.assertFalse(name_bytes.endswith(b"\x00"), "ADD_LOGGED name has a NUL terminator")
            self.assertEqual(payload_len, 3 + len(name_bytes),
                             "ADD_LOGGED payload length must be 3 + name length")


class TestNominalAnalysis(SyntheticFileMixin, unittest.TestCase):
    def test_no_error_findings(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        for topic, multi_id in (("sensor_gyro_fifo", 0), ("sensor_gyro_fifo", 1),
                                ("sensor_accel_fifo", 0), ("sensor_accel_fifo", 1)):
            _report, findings = analyze_fifo_dataset(content.dataset(topic, multi_id))
            errors = [f for f in findings if f.severity == "ERROR"]
            self.assertEqual(errors, [], "%s[%d]: %s" % (topic, multi_id, errors))

    def test_endpoint_rate_matches_independent_expectation(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        for multi_id, dt in ((0, 125.0), (1, 136.5333)):
            dataset = content.dataset("sensor_gyro_fifo", multi_id)
            report, _findings = analyze_fifo_dataset(dataset)
            expected = 1e6 / dt
            self.assertAlmostEqual(report["endpoint_rate_hz"] / expected, 1.0, delta=0.005)

    def test_samples_and_physical_scaling(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        dataset = content.dataset("sensor_gyro_fifo", 0)
        report, _findings = analyze_fifo_dataset(dataset)
        samples = [int(v) for v in dataset.get("samples")]
        expected_total = sum(samples)
        self.assertEqual(report["samples_total"], expected_total)
        # 20 packets, every third packet uses 19 samples
        self.assertEqual(expected_total, 20 * 20 - 7)
        scale = float(dataset.get("scale")[0])
        max_raw = 0
        for packet, count in enumerate(samples):
            for value in dataset.get("x")[packet][:count]:
                max_raw = max(max_raw, abs(value))
        self.assertAlmostEqual(report["physical_abs_max"], max_raw * scale, places=9)
        self.assertEqual(report["padding_tail_nonzero_packets"], 1)

    def test_health_summary_reads_type_field(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        health, findings = analyze_health(content)
        self.assertEqual(len(health["logger_status"]), 2)
        kinds = {entry["multi_id"]: entry["log_kinds"] for entry in health["logger_status"]}
        self.assertEqual(kinds[0], ["full"])
        self.assertEqual(kinds[1], ["mission"])
        self.assertEqual(finding_codes(findings).count("health/no-logger-status"), 0)
        self.assertIn("health/imu-clipping", finding_codes(findings))
        self.assertNotIn("health/logger-status-inactive", finding_codes(findings))


class TestErrorDetection(SyntheticFileMixin, unittest.TestCase):
    def _codes(self):
        content = ulog_reader.load_ulog(self.paths["errors"], reader="native")
        report, findings = analyze_fifo_dataset(content.dataset("sensor_gyro_fifo", 0))
        return content, report, finding_codes(findings), findings

    def test_all_anomalies_are_reported(self):
        content, report, codes, findings = self._codes()
        for expected in ("fifo/invalid-dt", "fifo/invalid-scale", "fifo/invalid-samples",
                         "fifo/timestamp-regression", "fifo/duplicate-packets",
                         "fifo/residual-beyond-tolerance", "fifo/device-change",
                         "fifo/raw-at-int16-limit", "fifo/raw-near-int16-limit"):
            self.assertIn(expected, codes, "missing finding %s (got %s)" % (expected, codes))
        self.assertEqual(len(content.dropouts), 1)
        self.assertGreaterEqual(len(report["device_segments"]), 2)
        self.assertGreater(report["raw"]["int16_limit_samples"], 0)
        self.assertGreater(report["raw"]["near_int16_limit_samples"], 0)

    def test_int16_limit_wording_points_to_driver_counters(self):
        _content, _report, _codes, findings = self._codes()
        limit = [f for f in findings if f.code == "fifo/raw-at-int16-limit"]
        self.assertTrue(limit)
        self.assertIn("confirmed from the driver counters", limit[0].message)
        self.assertNotIn("true clipping", limit[0].message.lower().replace(
            "true sensor clipping must be", ""))

    def test_residual_wording_does_not_claim_drop_count(self):
        _content, _report, _codes, findings = self._codes()
        residual = [f for f in findings if f.code == "fifo/residual-beyond-tolerance"]
        self.assertTrue(residual)
        self.assertIn("does not measure an exact number of dropped samples", residual[0].message)


class TestHealthEvidence(SyntheticFileMixin, unittest.TestCase):
    def test_no_fifo_data_is_reported(self):
        content = ulog_reader.load_ulog(self.paths["no_fifo"], reader="native")
        health, findings = analyze_health(content)
        codes = finding_codes(findings)
        self.assertIn("health/no-vehicle-imu-status", codes)
        code, output = run_cli(ulog_check.main, [self.paths["no_fifo"], "--reader", "native"])
        self.assertIn("fifo/no-data", output)
        self.assertEqual(code, 0, "a normal log without FIFO data is a warning, not an error")

    def test_missing_health_topics_are_missing_evidence(self):
        code, output = run_cli(ulog_check.main, [self.paths["health_missing"], "--reader", "native"])
        self.assertIn("health/no-logger-status", output)
        self.assertIn("health/no-vehicle-imu-status", output)
        self.assertIn("missing evidence, not a pass", output)
        self.assertEqual(code, 0)

    def test_inactive_mission_logger_is_not_a_fault(self):
        content = ulog_reader.load_ulog(self.paths["mission_inactive"], reader="native")
        health, findings = analyze_health(content)
        codes = finding_codes(findings)
        self.assertIn("health/logger-status-inactive", codes)
        self.assertNotIn("health/logger-buffer-pressure", codes)
        self.assertNotIn("health/logger-dropouts", codes)
        mission = [e for e in health["logger_status"] if e["multi_id"] == 1][0]
        self.assertFalse(mission["is_logging_any"])
        self.assertEqual(mission["buffer_size_max"], 0)
        full = [e for e in health["logger_status"] if e["multi_id"] == 0][0]
        self.assertTrue(full["is_logging_any"])

    def test_message_gaps_wording(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        # inject a message_gaps value through the analysis of a crafted row set
        dataset = content.dataset("logger_status", 0)
        dataset.fields["message_gaps"] = [3] * dataset.count
        _health, findings = analyze_health(content)
        gaps = [f for f in findings if f.code == "health/logger-message-gaps"]
        self.assertTrue(gaps)
        self.assertIn("not missing samples", gaps[0].message)


class TestExpectedSelection(SyntheticFileMixin, unittest.TestCase):
    def test_matching_expectation_has_no_selection_findings(self):
        code, output = run_cli(ulog_check.main,
                               [self.paths["nominal"], "--reader", "native",
                                "--expect-gyr-mask", "3", "--expect-acc-mask", "3"])
        self.assertNotIn("expect/", output)
        self.assertEqual(code, 0)

    def test_extra_instance_is_reported(self):
        _code, output = run_cli(ulog_check.main,
                                [self.paths["nominal"], "--reader", "native", "--expect-gyr-mask", "1"])
        self.assertIn("expect/instance-extra", output)

    def test_missing_instance_is_an_error(self):
        code, output = run_cli(ulog_check.main,
                               [self.paths["mask_gyro_instance1"], "--reader", "native",
                                "--expect-gyr-mask", "1"])
        self.assertIn("expect/instance-missing", output)
        self.assertEqual(code, 1)


class TestConfigChecker(SyntheticFileMixin, unittest.TestCase):
    def setUp(self):
        self.tmpdir_cfg = tempfile.mkdtemp(prefix="vibration-cfg-")
        self.addCleanup(shutil.rmtree, self.tmpdir_cfg, True)

    def _write(self, name, text):
        path = os.path.join(self.tmpdir_cfg, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        return path

    def test_valid_single_instance_selection(self):
        code, output = run_cli(config_check.main, ["--profile", "264", "--gyr-mask", "1", "--acc-mask", "1"])
        self.assertEqual(code, 0)
        self.assertIn("mask/selection", output)
        self.assertIn("gyro", output)

    def test_mask_without_profile_bit_warns(self):
        _code, output = run_cli(config_check.main, ["--profile", "1", "--gyr-mask", "3"])
        self.assertIn("mask/without-profile-bit", output)

    def test_out_of_range_mask_is_an_error(self):
        for value in ("-1", "1024"):
            code, output = run_cli(config_check.main,
                                   ["--profile", "256", "--gyr-mask", value, "--acc-mask", "1"])
            self.assertEqual(code, 1, "mask %s must be an error" % value)
            self.assertIn("param/mask-out-of-range", output)

    def test_clamping_and_clamp_to_zero(self):
        _code, output = run_cli(config_check.main,
                                ["--profile", "264", "--gyr-mask", "32", "--acc-mask", "32",
                                 "--orb-multi-max", "4"])
        self.assertIn("param/mask-clamped", output)
        self.assertIn("param/mask-clamped-to-zero", output)
        self.assertIn("NOT expanded to", output)

    def test_zero_mask_with_profile_bit(self):
        _code, output = run_cli(config_check.main, ["--profile", "256", "--gyr-mask", "0", "--acc-mask", "1"])
        self.assertIn("mask/empty-selection", output)
        self.assertIn("Clear the profile bit and reboot", output)

    def test_topics_file_valid_replaces_profile(self):
        path = self._write("logger_topics.txt", "sensor_gyro 0 0\nsensor_accel 0 0\n")
        code, output = run_cli(config_check.main,
                               ["--profile", "256", "--gyr-mask", "1", "--acc-mask", "1",
                                "--topics-file", path])
        self.assertEqual(code, 0)
        self.assertIn("topics-file/replaces-profile", output)
        self.assertIn("topics-file/no-health-topics", output)

    def test_topics_file_all_unknown_is_still_a_replacement(self):
        topics = self._write("logger_topics.txt", "not_a_topic_one\nnot_a_topic_two\n")
        known = self._write("known_topics.txt", "sensor_gyro\nsensor_accel\n")
        _code, output = run_cli(config_check.main,
                                ["--profile", "256", "--gyr-mask", "1", "--acc-mask", "1",
                                 "--topics-file", topics, "--known-topics", known])
        self.assertIn("topics-file/replaces-profile", output)
        self.assertIn("topics-file/all-unknown", output)
        self.assertIn("This is NOT 'no replacement'", output)
        self.assertNotIn("topics-file/absent", output)

    def test_topics_file_empty_or_comments_falls_back_to_profile(self):
        path = self._write("logger_topics.txt", "\n# comment\n   \n")
        _code, output = run_cli(config_check.main, ["--profile", "256", "--topics-file", path])
        self.assertIn("topics-file/no-parsed-lines", output)
        self.assertIn("replacement active: no", output)

    def test_topics_file_missing_is_informational(self):
        _code, output = run_cli(config_check.main,
                                ["--profile", "256", "--topics-file",
                                 os.path.join(self.tmpdir_cfg, "does_not_exist.txt")])
        self.assertIn("topics-file/absent", output)

    def test_unknown_lines_do_count_like_the_original_parser(self):
        # a 1-character topic name still parses (strlen(line) >= 2 including newline)
        path = self._write("logger_topics.txt", "x\n")
        entries, ntopics = config_check.parse_topics_file(path)
        self.assertEqual(ntopics, 1)
        self.assertEqual(entries[0]["name"], "x")
        # a whitespace-only line does not parse
        path2 = self._write("logger_topics2.txt", "   \n")
        _entries2, ntopics2 = config_check.parse_topics_file(path2)
        self.assertEqual(ntopics2, 0)

    def test_sdlog_mode_disabled_is_an_error(self):
        params = self._write("params.txt", "SDLOG_MODE -1\nSDLOG_PROFILE 256\n")
        code, output = run_cli(config_check.main, ["--params", params, "--expect-logging-without-arming"])
        self.assertEqual(code, 1)
        self.assertIn("sdlog/mode-disabled", output)

    def test_params_file_parsing(self):
        params = self._write("params.txt", "SDLOG_PROFILE: 769\nSDLOG_ACC_MASK 3\nSDLOG_GYR_MASK 2\n")
        parsed = config_check.parse_params_file(params)
        self.assertEqual(parsed["SDLOG_PROFILE"], 769)
        self.assertEqual(parsed["SDLOG_ACC_MASK"], 3)
        self.assertEqual(parsed["SDLOG_GYR_MASK"], 2)


class TestConfigCheckerJson(SyntheticFileMixin, unittest.TestCase):
    def test_json_output(self):
        path = os.path.join(self.tmpdir, "config.json")
        code, _output = run_cli(config_check.main,
                                ["--profile", "264", "--gyr-mask", "3", "--acc-mask", "1", "--json", path])
        self.assertEqual(code, 0)
        import json
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.assertEqual(payload["masks"]["SDLOG_GYR_MASK"]["instances"], [0, 1])
        self.assertEqual(payload["summary"]["errors"], 0)


class TestRealLogOptional(unittest.TestCase):
    @unittest.skipUnless(REAL_ULOG and os.path.exists(REAL_ULOG),
                         "set PX4_VIBRATION_REAL_ULOG to a real PX4 ULog to enable this test")
    def test_real_log_is_readable_and_padding_convention_holds(self):
        content = ulog_reader.load_ulog(REAL_ULOG, reader="native")
        self.assertTrue(content.datasets)
        self.assertTrue(content.datasets_named("vehicle_imu_status"))
        self.assertEqual(content.datasets_named("sensor_gyro_fifo"), [])
        checked = 0
        for topic, payload_len, fields in data_payload_lengths(REAL_ULOG):
            if not fields:
                continue
            expected = 2 + sum(ulog_reader.BASE_TYPES[f.type_name][1] * f.count
                               for f in fields if not f.is_padding)
            self.assertEqual(payload_len, expected,
                             "%s: %d != %d" % (topic, payload_len, expected))
            checked += 1
        self.assertGreater(checked, 100, "expected a substantial real log")

        # the synthetic writer must match real PX4: no NUL terminator in ADD_LOGGED names
        records = add_logged_records(REAL_ULOG)
        self.assertGreater(len(records), 50)
        for payload_len, name_bytes in records:
            self.assertEqual(payload_len, 3 + len(name_bytes),
                             "real ADD_LOGGED payload_len must equal 3 + name length")
            self.assertFalse(name_bytes.endswith(b"\x00"), "real ADD_LOGGED names carry no NUL")


class TestPyulogCrossCheck(SyntheticFileMixin, unittest.TestCase):
    @unittest.skipUnless(ulog_reader.pyulog_available(),
                         "pyulog is not installed in this interpreter: run this test in the "
                         "repository environment (Tools/setup/requirements.txt)")
    def test_pyulog_reads_synthetic_files(self):
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="pyulog")
        self.assertEqual(content.reader, "pyulog")
        native = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        self.assertEqual(sorted(set(n for n, _ in content.datasets)),
                         sorted(set(n for n, _ in native.datasets)))
        for key, dataset in native.datasets.items():
            self.assertEqual(content.datasets[key].count, dataset.count, str(key))

    @unittest.skipUnless(ulog_reader.pyulog_available(), "pyulog is not installed")
    def test_pyulog_reads_errors_scenario(self):
        content = ulog_reader.load_ulog(self.paths["errors"], reader="pyulog")
        dataset = content.dataset("sensor_gyro_fifo", 0)
        self.assertIsNotNone(dataset)
        self.assertGreater(dataset.count, 5)


class TestDeviceSegmentAnalysis(unittest.TestCase):
    def test_residual_stops_at_device_boundary(self):
        rows = [
            {"timestamp": 1_000_000, "anchor": 1_000_000, "device_id": 0x36001A},
            {"timestamp": 1_002_750, "anchor": 1_002_500, "device_id": 0x36001A},
            {"timestamp": 5_000_000, "anchor": 5_000_000, "device_id": 0x99001A},
            {"timestamp": 5_002_750, "anchor": 5_002_500, "device_id": 0x99001A},
        ]
        report, findings = analyze_fifo_dataset(fifo_dataset(rows))

        # one residual per segment, never across the device change (would be 3 otherwise)
        self.assertEqual(report["residual_us"]["count"], 2)
        self.assertEqual(len(report["device_segments"]), 2)
        self.assertIn("fifo/device-change", finding_codes(findings))
        # no global span across different devices -> no global rate
        self.assertIsNone(report["endpoint_rate_hz"])
        for segment in report["device_segments"]:
            self.assertAlmostEqual(segment["endpoint_rate_hz"], 8000.0, delta=1.0)
            self.assertEqual(segment["residual_us"]["count"], 1)

    def test_device_id_zero_is_missing_mapping_evidence(self):
        rows = [{"timestamp": 1_000_000, "anchor": 1_000_000, "device_id": 0},
                {"timestamp": 1_002_750, "anchor": 1_002_500, "device_id": 0}]
        report, findings = analyze_fifo_dataset(fifo_dataset(rows))
        self.assertEqual(report["device_id_zero_packets"], 1)
        self.assertIn("fifo/device-id-unset", finding_codes(findings))


class TestHealthRowGating(SyntheticFileMixin, unittest.TestCase):
    def test_mixed_rows_are_gated_individually(self):
        content = ulog_reader.load_ulog(self.paths["health_rows_mixed"], reader="native")
        health, findings = analyze_health(content)
        codes = finding_codes(findings)
        entry = health["logger_status"][0]

        self.assertEqual(entry["active_full_file_rows"], 3)
        self.assertEqual(entry["full_log_zero_buffer_rows"], 1)
        self.assertIn("health/full-log-zero-buffer", codes)
        self.assertIn("health/logger-buffer-pressure", codes)
        # the ratio comes only from the rows that actually have a buffer
        self.assertAlmostEqual(entry["buffer_fill_ratio_max"], 7000 / 8192.0, places=6)

    def test_no_active_full_file_row_is_missing_evidence(self):
        content = ulog_reader.load_ulog(self.paths["health_inactive_only"], reader="native")
        health, findings = analyze_health(content)
        self.assertIn("health/no-active-full-file-row", finding_codes(findings))
        self.assertEqual(health["logger_status"][0]["active_full_file_rows"], 0)


class TestSummaryWording(SyntheticFileMixin, unittest.TestCase):
    def test_clean_run_uses_no_anomaly_wording(self):
        code, output = run_cli(ulog_check.main,
                               [self.paths["nominal"], "--reader", "native",
                                "--expect-gyr-mask", "3", "--expect-acc-mask", "3"])
        self.assertEqual(code, 0)
        self.assertIn("no anomaly found at the checked level", output)
        self.assertNotIn("need review", output)

    def test_warning_run_asks_for_review(self):
        code, output = run_cli(ulog_check.main, [self.paths["health_rows_mixed"], "--reader", "native"])
        self.assertEqual(code, 0)
        self.assertIn("need review", output)
        self.assertNotIn("no anomaly found at the checked level", output)


class TestReaderPolicy(SyntheticFileMixin, unittest.TestCase):
    def test_native_reader_is_marked_diagnostic(self):
        code, output = run_cli(ulog_check.main, [self.paths["nominal"], "--reader", "native"])
        self.assertEqual(code, 0)
        self.assertIn("reader/native-diagnostic", output)

    def test_auto_without_pyulog_returns_marked_diagnostic_reader(self):
        if ulog_reader.pyulog_available():
            self.skipTest("pyulog is installed: the auto path uses pyulog here")
        content = ulog_reader.load_ulog(self.paths["nominal"], reader="auto")
        self.assertIn("diagnostic", content.reader)
        self.assertIn("pyulog not installed", content.reader)

    def test_pyulog_reader_without_pyulog_fails_instead_of_downgrading(self):
        if ulog_reader.pyulog_available():
            self.skipTest("pyulog is installed")
        with self.assertRaises(RuntimeError):
            ulog_reader.load_ulog(self.paths["nominal"], reader="pyulog")

    def test_skipped_topics_are_reported_as_limited_evidence(self):
        # hand-crafted ULog whose topic has a nested (unsupported) field type, like the real
        # position_setpoint_triplet topic in flight logs
        path = os.path.join(self.tmpdir, "nested.ulg")

        with open(path, "wb") as handle:
            handle.write(synth.ULOG_HEADER)
            handle.write(struct.pack("<Q", 1_000_000))

            def message(msg_type, payload):
                handle.write(struct.pack("<HB", len(payload), msg_type))
                handle.write(payload)

            message(synth.MSG_FORMAT,
                    b"position_setpoint_triplet:uint64_t timestamp;position_setpoint_s current;")
            message(synth.MSG_ADD_LOGGED, struct.pack("<BH", 0, 0) + b"position_setpoint_triplet")
            message(synth.MSG_DATA, struct.pack("<H", 0) + struct.pack("<Q", 1_000_000) + b"\x00" * 40)

        content = ulog_reader.load_ulog(path, reader="native")
        self.assertEqual(content.unsupported_topics, ["position_setpoint_triplet"])
        _code, output = run_cli(ulog_check.main, [path, "--reader", "native"])
        self.assertIn("reader/native-diagnostic", output)
        self.assertIn("reader/skipped-topics", output)
        self.assertIn("limited evidence", output)


class TestCliInputValidation(SyntheticFileMixin, unittest.TestCase):
    def test_invalid_numeric_options_are_rejected(self):
        cases = (["--gap-tolerance-us", "-1"], ["--gap-tolerance-us", "nan"],
                 ["--gap-tolerance-us", "inf"], ["--latency-limit-us", "0"],
                 ["--latency-limit-us", "-5"], ["--full-scale-fraction", "1.5"],
                 ["--full-scale-fraction", "nan"], ["--orb-multi-max", "11"],
                 ["--orb-multi-max", "0"])
        for extra in cases:
            code, _output = run_cli(ulog_check.main,
                                    [self.paths["nominal"], "--reader", "native"] + extra)
            self.assertEqual(code, 2, "expected a usage error for %s" % extra)

    def test_orb_multi_max_limits_expected_instances(self):
        _code, output = run_cli(ulog_check.main,
                                [self.paths["mask_gyro_instance1"], "--reader", "native",
                                 "--expect-gyr-mask", "1023", "--orb-multi-max", "4"])
        # a 4-instance build must not demand instances 4..9
        self.assertIn("selected=[0, 1, 2, 3]", output)
        self.assertNotIn("[0, 1, 2, 3, 4", output)


class TestReaderDowngradePolicy(SyntheticFileMixin, unittest.TestCase):
    def test_auto_does_not_downgrade_when_pyulog_fails(self):
        """A broken pyulog decode must fail the run, not silently succeed natively."""
        import types

        fake_pyulog = types.ModuleType("pyulog")

        class Boom(Exception):
            pass

        def _raise(*_args, **_kwargs):
            raise Boom("simulated corrupt file")

        fake_pyulog.ULog = _raise

        original_available = ulog_reader.pyulog_available
        original_module = sys.modules.get("pyulog")
        sys.modules["pyulog"] = fake_pyulog
        ulog_reader.pyulog_available = lambda: True

        try:
            with self.assertRaises(RuntimeError) as context:
                ulog_reader.load_ulog(self.paths["nominal"], reader="auto")
            self.assertIn("not silently downgraded", str(context.exception))

            # explicit diagnostic read is still available
            content = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
            self.assertTrue(content.reader.startswith("native"))
        finally:
            ulog_reader.pyulog_available = original_available

            if original_module is not None:
                sys.modules["pyulog"] = original_module

            else:
                sys.modules.pop("pyulog", None)


class TestPyulogFieldNormalization(unittest.TestCase):
    def test_arrays_are_rebuilt_from_indexed_keys(self):
        fields = {
            "timestamp": [1, 2],
            "samples": [3, 4],
            "is_logging": [True, False],
            "x[0]": [10, 20], "x[1]": [11, 21], "x[2]": [12, 22],
            "accel_clipping[0]": [0, 1], "accel_clipping[1]": [0, 2], "accel_clipping[2]": [5, 3],
        }
        normalized = ulog_reader.normalize_pyulog_fields(fields)
        self.assertEqual(normalized["x"], [[10, 11, 12], [20, 21, 22]])
        self.assertEqual(normalized["accel_clipping"], [[0, 0, 5], [1, 2, 3]])
        self.assertEqual(normalized["samples"], [3, 4])
        self.assertNotIn("x[0]", normalized)
        self.assertNotIn("accel_clipping[0]", normalized)


class TestPyulogValueCrossCheck(SyntheticFileMixin, unittest.TestCase):
    @unittest.skipUnless(ulog_reader.pyulog_available(),
                         "pyulog is not installed: run this suite in the repository environment")
    def test_pyulog_matches_native_values_and_arrays(self):
        py = ulog_reader.load_ulog(self.paths["nominal"], reader="pyulog")
        native = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        self.assertEqual(sorted(set(n for n, _ in py.datasets)),
                         sorted(set(n for n, _ in native.datasets)))
        for key, dataset in native.datasets.items():
            other = py.datasets[key]
            self.assertEqual(other.count, dataset.count, str(key))
            for field in ("samples", "dt", "scale", "device_id", "timestamp_sample"):
                self.assertEqual(other.get(field), dataset.get(field), "%s %s" % (key, field))
            self.assertEqual(other.get("x"), dataset.get("x"), "%s x" % (key,))

    @unittest.skipUnless(ulog_reader.pyulog_available(), "pyulog is not installed")
    def test_pyulog_health_values_and_summary_match(self):
        py = ulog_reader.load_ulog(self.paths["nominal"], reader="pyulog")
        native = ulog_reader.load_ulog(self.paths["nominal"], reader="native")
        for key, dataset in native.datasets.items():
            if key[0] != "logger_status":
                continue
            for field in ("type", "is_logging", "backend", "buffer_size_bytes", "buffer_used_bytes",
                          "dropouts", "message_gaps"):
                self.assertEqual(py.datasets[key].get(field), dataset.get(field),
                                 "%s %s" % (key, field))
        py_health, _f = analyze_health(py)
        native_health, _f2 = analyze_health(native)
        self.assertEqual([e["active_full_file_rows"] for e in py_health["logger_status"]],
                         [e["active_full_file_rows"] for e in native_health["logger_status"]])

    @unittest.skipUnless(ulog_reader.pyulog_available(), "pyulog is not installed")
    def test_pyulog_reads_the_errors_scenario_and_finds_the_same_anomalies(self):
        py = ulog_reader.load_ulog(self.paths["errors"], reader="pyulog")
        native = ulog_reader.load_ulog(self.paths["errors"], reader="native")
        py_report, py_findings = analyze_fifo_dataset(py.dataset("sensor_gyro_fifo", 0))
        native_report, native_findings = analyze_fifo_dataset(native.dataset("sensor_gyro_fifo", 0))
        self.assertEqual(sorted(f.code for f in py_findings), sorted(f.code for f in native_findings))
        self.assertEqual(py_report["device_segments"], native_report["device_segments"])

    @unittest.skipUnless(ulog_reader.pyulog_available(), "pyulog is not installed")
    def test_pyulog_reads_all_synthetic_scenarios(self):
        for name, path in self.paths.items():
            content = ulog_reader.load_ulog(path, reader="pyulog")
            self.assertEqual(content.reader, "pyulog", name)


class TempFileMixin(object):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="vibration-tmp-")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def write(self, name, text):
        path = os.path.join(self.tmp, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        return path


class TestParamShowParsing(TempFileMixin, unittest.TestCase):
    def test_real_param_show_lines(self):
        """Format from src/systemcmds/param/param.cpp:680: '%c %c %s [%d,%d] : '."""
        path = self.write("params.txt", "\n".join([
            "x + SDLOG_PROFILE [3,45] : 769",
            "   SDLOG_ACC_MASK [0,12] : 2",
            " * SDLOG_GYR_MASK [1,7] : 5",
            "x   SDLOG_MODE [0,1] : 1.0000",
            "x SDLOG_CALIB_FLOAT [0,1] : 1.5000",
        ]) + "\n")
        params = config_check.parse_params_file(path)
        self.assertEqual(params["SDLOG_PROFILE"], 769)
        self.assertEqual(params["SDLOG_ACC_MASK"], 2)
        self.assertEqual(params["SDLOG_GYR_MASK"], 5)
        self.assertEqual(params["SDLOG_MODE"], 1)             # integral %4.4f value stays an int
        self.assertEqual(params["SDLOG_CALIB_FLOAT"], 1.5)    # never truncated to 1

    def test_simple_name_value_and_colon_forms(self):
        path = self.write("params.txt", "SDLOG_PROFILE 768\nSDLOG_ACC_MASK: 2\nSDLOG_GYR_MASK 5\n")
        params = config_check.parse_params_file(path)
        self.assertEqual(params["SDLOG_PROFILE"], 768)
        self.assertEqual(params["SDLOG_ACC_MASK"], 2)
        self.assertEqual(params["SDLOG_GYR_MASK"], 5)

    def test_fractional_relevant_value_is_an_error_not_truncation(self):
        path = self.write("params.txt", "x SDLOG_PROFILE [3,45] : 769.5\n")
        code, output = run_cli(config_check.main, ["--params", path])
        self.assertEqual(code, 1)
        self.assertIn("param/not-integer", output)
        self.assertIn("769.5", output)
        self.assertIn("param/profile-unusable", output)

    def test_unparseable_relevant_values_error_instead_of_crashing(self):
        cases = ("x SDLOG_GYR_MASK [1,7] : abc\n",
                 "x SDLOG_ACC_MASK [0,12] : <error fetching parameter 42>\n",
                 "SDLOG_MODE nope\n")
        for text in cases:
            path = self.write("params.txt", text)
            code, output = run_cli(config_check.main, ["--params", path, "--profile", "768"])
            self.assertEqual(code, 1, "expected ERROR exit for %r" % text)
            self.assertIn("param/unparseable", output)

    def test_missing_relevant_parameter_stays_missing_evidence(self):
        path = self.write("params.txt", "x SDLOG_MODE [0,1] : 1\n")
        code, output = run_cli(config_check.main, ["--params", path])
        self.assertEqual(code, 0)
        # profile and masks are absent -> reported as defaults, not silently assumed as configured
        self.assertIn("param/profile-default", output)
        self.assertIn("param/default", output)
        self.assertIn("mask/without-profile-bit", output)

    def test_integration_configuration_is_parsed_end_to_end(self):
        """The configuration used by functional-LoggerFifoIntegration (768 / acc 2 / gyr 5)."""
        path = self.write("params.txt", "\n".join([
            "x + SDLOG_PROFILE [3,45] : 768",
            "   SDLOG_ACC_MASK [0,12] : 2",
            "   SDLOG_GYR_MASK [1,7] : 5",
        ]) + "\n")
        json_path = os.path.join(self.tmp, "out.json")
        code, _output = run_cli(config_check.main, ["--params", path, "--json", json_path])
        self.assertEqual(code, 0)
        with open(json_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.assertEqual(payload["masks"]["SDLOG_ACC_MASK"]["instances"], [1])
        self.assertEqual(payload["masks"]["SDLOG_GYR_MASK"]["instances"], [0, 2])


class TestTopicsFileAmbiguity(TempFileMixin, unittest.TestCase):
    def test_numeric_prefix_and_third_token_semantics(self):
        path = self.write("t1.txt", "sensor_gyro 12abc 3\nsensor_accel 100 abc\n")
        entries, ntopics = config_check.parse_topics_file(path)
        self.assertEqual(ntopics, 2)
        self.assertEqual(entries[0]["interval_ms"], 12)
        self.assertEqual(entries[0]["instance"], 3)
        self.assertTrue(any("trailing characters" in reason for reason in entries[0]["ambiguous"]))
        self.assertEqual(entries[1]["nfields"], 2)       # a failed third conversion keeps nfields=2
        self.assertIsNone(entries[1]["instance"])
        self.assertTrue(any("nfields=2" in reason for reason in entries[1]["ambiguous"]))

    def test_overlong_line_is_flagged_not_silently_emulated(self):
        path = self.write("t2.txt", "sensor_gyro 0 0" + " " * 70 + "\n")
        entries, ntopics = config_check.parse_topics_file(path)
        self.assertEqual(ntopics, 1)
        self.assertTrue(any("fgets" in reason for reason in entries[0]["ambiguous"]))

    def test_narrowing_and_unsigned_wrap_are_flagged(self):
        path = self.write("t3.txt", "sensor_gyro 70000 300\nsensor_gyro -5 0\n")
        entries, _ntopics = config_check.parse_topics_file(path)
        self.assertEqual(entries[0]["interval_ms"], 70000 & 0xFFFF)
        self.assertEqual(entries[0]["instance"], 300 & 0xFF)
        self.assertTrue(any("uint16" in reason for reason in entries[0]["ambiguous"]))
        self.assertTrue(any("uint8" in reason for reason in entries[0]["ambiguous"]))
        self.assertTrue(any("unsigned wrap" in reason for reason in entries[1]["ambiguous"]))

    def test_cli_reports_ambiguous_lines_and_keeps_replacement_semantics(self):
        path = self.write("t4.txt", "sensor_gyro 12abc 3\n")
        code, output = run_cli(config_check.main,
                               ["--profile", "256", "--gyr-mask", "1", "--topics-file", path])
        self.assertEqual(code, 0)
        self.assertIn("topics-file/ambiguous-line", output)
        self.assertIn("topics-file/replaces-profile", output)
        self.assertIn("ordinary input", output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
