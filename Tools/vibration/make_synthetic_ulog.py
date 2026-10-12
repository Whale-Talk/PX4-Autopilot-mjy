#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synthetic ULog writer for the vibration tooling tests.

The files are real ULog serialisations (same header, message framing, format
strings, subscription registration and parameter encoding as PX4 writes), so they
can be read both by the built-in fallback reader and by pyulog.

Verified against a real PX4 v1.16.1 log:
  * header = b"ULog\\x01\\x125\\x01", uint64 start timestamp at offset 8;
  * messages are [uint16 payload_len][uint8 type][payload];
  * format strings list generated ``_padding*`` fields, data records do not
    contain them (137/137 records in the reference log match this);
  * the sync payload is the constant byte sequence PX4 writes.

Scenarios: nominal, mask_gyro_instance1, no_fifo, errors, health_missing,
mission_inactive.
"""

import argparse
import math
import os
import struct
import sys

MSG_FORMAT = 0x46
MSG_DATA = 0x44
MSG_INFO = 0x49
MSG_PARAM = 0x50
MSG_ADD_LOGGED = 0x41
MSG_SYNC = 0x53
MSG_DROPOUT = 0x4F

ULOG_HEADER = b"ULog\x01\x125\x01"
SYNC_PAYLOAD = bytes([0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12])

# type name -> (struct code, size, alignment)
BASE_TYPES = {
    "uint8_t": ("B", 1, 1),
    "int8_t": ("b", 1, 1),
    "bool": ("B", 1, 1),
    "char": ("c", 1, 1),
    "uint16_t": ("H", 2, 2),
    "int16_t": ("h", 2, 2),
    "uint32_t": ("I", 4, 4),
    "int32_t": ("i", 4, 4),
    "float": ("f", 4, 4),
    "uint64_t": ("Q", 8, 8),
    "int64_t": ("q", 8, 8),
    "double": ("d", 8, 8),
}

FIFO_FIELDS = [
    ("uint64_t", "timestamp", 1),
    ("uint64_t", "timestamp_sample", 1),
    ("uint32_t", "device_id", 1),
    ("float", "dt", 1),
    ("float", "scale", 1),
    ("uint8_t", "samples", 1),
    ("int16_t", "x", 32),
    ("int16_t", "y", 32),
    ("int16_t", "z", 32),
]

LOGGER_STATUS_FIELDS = [
    ("uint64_t", "timestamp", 1),
    ("uint8_t", "type", 1),
    ("uint8_t", "backend", 1),
    ("bool", "is_logging", 1),
    ("float", "total_written_kb", 1),
    ("float", "write_rate_kb_s", 1),
    ("uint32_t", "dropouts", 1),
    ("uint32_t", "message_gaps", 1),
    ("uint32_t", "buffer_used_bytes", 1),
    ("uint32_t", "buffer_size_bytes", 1),
    ("uint8_t", "num_messages", 1),
]

VEHICLE_IMU_STATUS_FIELDS = [
    ("uint64_t", "timestamp", 1),
    ("uint32_t", "accel_device_id", 1),
    ("uint32_t", "gyro_device_id", 1),
    ("uint32_t", "accel_clipping", 3),
    ("uint32_t", "gyro_clipping", 3),
    ("uint32_t", "accel_error_count", 1),
    ("uint32_t", "gyro_error_count", 1),
    ("float", "accel_rate_hz", 1),
    ("float", "gyro_rate_hz", 1),
    ("float", "accel_raw_rate_hz", 1),
    ("float", "gyro_raw_rate_hz", 1),
    ("float", "accel_vibration_metric", 1),
    ("float", "gyro_vibration_metric", 1),
    ("float", "delta_angle_coning_metric", 1),
    ("float", "mean_accel", 3),
    ("float", "mean_gyro", 3),
    ("float", "var_accel", 3),
    ("float", "var_gyro", 3),
    ("float", "temperature_accel", 1),
    ("float", "temperature_gyro", 1),
]


def _layout(fields):
    """Insert `_padding*` fields exactly as the PX4 generator would."""
    offset = 0
    align_max = 1
    padded = []
    pad_index = 0
    # PX4 sorts fields by base type size before adding alignment padding.
    # Keeping declaration order would introduce internal padding that pyulog
    # expects in data records, while this writer omits padding from the payload.
    ordered_fields = sorted(fields, key=lambda field: BASE_TYPES[field[0]][1], reverse=True)
    for type_name, name, count in ordered_fields:
        _code, size, align = BASE_TYPES[type_name]
        align_max = max(align_max, align)
        gap = (align - (offset % align)) % align
        if gap:
            padded.append(("uint8_t", "_padding%d" % pad_index, gap))
            pad_index += 1
            offset += gap
        padded.append((type_name, name, count))
        offset += size * count
    tail = (align_max - (offset % align_max)) % align_max
    if tail:
        padded.append(("uint8_t", "_padding%d" % pad_index, tail))
    return padded


def _format_string(name, padded):
    parts = []
    for type_name, field_name, count in padded:
        if count > 1:
            parts.append("%s[%d] %s" % (type_name, count, field_name))
        else:
            parts.append("%s %s" % (type_name, field_name))
    return "%s:%s;" % (name, ";".join(parts))


def _pack_data(fields, values):
    chunks = []
    for type_name, name, count in fields:
        if name.startswith("_padding"):
            continue  # declared in the format string, not written into data records
        code, size, _align = BASE_TYPES[type_name]
        value = values[name]
        if type_name == "char":
            chunks.append(value if isinstance(value, bytes) else value.encode("utf-8"))
        elif code in ("B", "H", "I", "Q", "b", "h", "i", "q"):
            if count == 1:
                chunks.append(struct.pack("<" + code, int(round(value))))
            else:
                chunks.append(struct.pack("<%d%s" % (count, code),
                                          *[int(round(v)) for v in value]))
        else:
            if count == 1:
                chunks.append(struct.pack("<" + code, float(value)))
            else:
                chunks.append(struct.pack("<%d%s" % (count, code), *[float(v) for v in value]))
    return b"".join(chunks)


def _pack_param_value(type_name, count, value):
    if type_name == "char":
        return value.encode("utf-8") if isinstance(value, str) else value
    code, _size, _align = BASE_TYPES[type_name]
    if count > 1:
        return struct.pack("<%d%s" % (count, code), *value)
    return struct.pack("<" + code, value)


class UlogWriter(object):
    """Minimal ULog writer (format strings, subscriptions, params, data)."""

    def __init__(self, path, start_timestamp_us=1_000_000):
        self.path = path
        self.start_timestamp_us = start_timestamp_us
        self._handle = open(path, "wb")
        self._handle.write(ULOG_HEADER)
        self._handle.write(struct.pack("<Q", start_timestamp_us))
        self._formats = {}          # name -> padded field list
        self._written_formats = set()
        self._subscriptions = {}    # (name, multi_id) -> msg_id
        self._next_msg_id = 0

        # ULog definitions precede the first ADD_LOGGED record. pyulog switches
        # to the data section at that record and does not accept later formats.
        # Declare all four topics used by these synthetic scenarios up front.
        for name, fields in (("sensor_accel_fifo", FIFO_FIELDS),
                             ("sensor_gyro_fifo", FIFO_FIELDS),
                             ("logger_status", LOGGER_STATUS_FIELDS),
                             ("vehicle_imu_status", VEHICLE_IMU_STATUS_FIELDS)):
            self.register_format(name, fields)

    # -- low level --------------------------------------------------------------
    def _write_message(self, msg_type, payload):
        self._handle.write(struct.pack("<HB", len(payload), msg_type))
        self._handle.write(payload)

    # -- formats and subscriptions ---------------------------------------------
    def register_format(self, name, fields):
        padded = _layout(fields)
        self._formats[name] = padded
        if name not in self._written_formats:
            self._write_message(MSG_FORMAT, _format_string(name, padded).encode("utf-8"))
            self._written_formats.add(name)
        return padded

    def register_subscription(self, name, fields, multi_id):
        if name not in self._formats:
            self.register_format(name, fields)
        key = (name, multi_id)
        if key in self._subscriptions:
            return self._subscriptions[key]
        msg_id = self._next_msg_id
        self._next_msg_id += 1
        # Real PX4 writes [uint8 multi_id][uint16 msg_id][name] with NO NUL terminator:
        # the payload length defines the name exactly (verified against a real v1.16.1 log,
        # 162/162 ADD_LOGGED records have payload_len == 3 + len(name)). pyulog decodes
        # data[3:] verbatim, so a terminator would end up inside the topic name.
        payload = struct.pack("<BH", multi_id, msg_id) + name.encode("utf-8")
        self._write_message(MSG_ADD_LOGGED, payload)
        self._subscriptions[key] = msg_id
        return msg_id

    # -- metadata ---------------------------------------------------------------
    def write_param(self, name, value, type_name="int32_t", count=1):
        key = "%s %s" % (type_name, name)
        if type_name == "char":
            key = "char[%d] %s" % (len(value) + 1, name)
        key_bytes = key.encode("utf-8")
        payload = struct.pack("<B", len(key_bytes)) + key_bytes + _pack_param_value(type_name, count, value)
        self._write_message(MSG_PARAM, payload)

    def write_info(self, name, value, type_name="char"):
        key = "char[%d] %s" % (len(value) + 1, name) if type_name == "char" else "%s %s" % (type_name, name)
        key_bytes = key.encode("utf-8")
        payload = struct.pack("<B", len(key_bytes)) + key_bytes + _pack_param_value(type_name, 1, value)
        self._write_message(MSG_INFO, payload)

    def write_sync(self):
        self._write_message(MSG_SYNC, SYNC_PAYLOAD)

    def write_dropout(self, duration_ms):
        self._write_message(MSG_DROPOUT, struct.pack("<H", duration_ms))

    # -- data -------------------------------------------------------------------
    def write_data(self, name, multi_id, values):
        msg_id = self._subscriptions[(name, multi_id)]
        padded = self._formats[name]
        payload = struct.pack("<H", msg_id) + _pack_data(padded, values)
        self._write_message(MSG_DATA, payload)

    def close(self):
        self._handle.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# ---------------------------------------------------------------------------
# scenarios
# ---------------------------------------------------------------------------

def _fifo_packet(timestamp, anchor, device_id, dt, scale, samples, amplitude, tail_nonzero=False,
                 saturated=False, limit=32767):
    values = []
    for index in range(32):
        if index < samples:
            base = amplitude * math.sin(index * 0.25)
            value = int(max(-32768, min(32767, base)))
            if saturated and index == 0:
                value = limit
        else:
            value = 4321 if (tail_nonzero and index == samples) else 0
        values.append(value)
    return {"timestamp": timestamp, "timestamp_sample": anchor, "device_id": device_id,
            "dt": dt, "scale": scale, "samples": samples,
            "x": values, "y": list(values), "z": list(values)}


def _logger_status_rows(start_us, count, kind, backend, is_logging, buffer_size,
                        buffer_used=0, dropouts=0, gaps=0):
    rows = []
    for index in range(count):
        rows.append({"timestamp": start_us + index * 1_000_000,
                     "type": kind, "backend": backend, "is_logging": is_logging,
                     "total_written_kb": 100.0 * (index + 1), "write_rate_kb_s": 50.0,
                     "dropouts": dropouts, "message_gaps": gaps,
                     "buffer_used_bytes": buffer_used, "buffer_size_bytes": buffer_size,
                     "num_messages": 120})
    return rows


def _vehicle_imu_status_row(timestamp, accel_id, gyro_id, clipping=(0, 0, 0), errors=(0, 0)):
    return {"timestamp": timestamp, "accel_device_id": accel_id, "gyro_device_id": gyro_id,
            "accel_clipping": list(clipping), "gyro_clipping": [0, 0, 0],
            "accel_error_count": errors[0], "gyro_error_count": errors[1],
            "accel_rate_hz": 800.0, "gyro_rate_hz": 800.0,
            "accel_raw_rate_hz": 8000.0, "gyro_raw_rate_hz": 8000.0,
            "accel_vibration_metric": 12.5, "gyro_vibration_metric": 0.4,
            "delta_angle_coning_metric": 0.0,
            "mean_accel": [0.1, 0.2, 9.81], "mean_gyro": [0.0, 0.0, 0.0],
            "var_accel": [0.5, 0.5, 0.5], "var_gyro": [0.01, 0.01, 0.01],
            "temperature_accel": 45.0, "temperature_gyro": 45.0}


def _write_params(writer, profile, acc_mask, gyr_mask, sdlog_mode=2, imu_gyro_ratemax=400):
    writer.write_param("SDLOG_PROFILE", profile)
    writer.write_param("SDLOG_ACC_MASK", acc_mask)
    writer.write_param("SDLOG_GYR_MASK", gyr_mask)
    writer.write_param("SDLOG_MODE", sdlog_mode)
    writer.write_param("IMU_GYRO_RATEMAX", imu_gyro_ratemax)


def scenario_nominal(writer):
    """Two accel and two gyro instances, varying packets, health topics present."""
    _write_params(writer, profile=769, acc_mask=3, gyr_mask=3)
    writer.write_info("sys_name", "PX4")
    writer.write_info("ver_sw", "synthetic")

    accel_ids = {0: 0x3A001A, 1: 0x380012}
    gyro_ids = {0: 0x36001A, 1: 0x380013}
    start_us = 2_000_000

    for multi_id in (0, 1):
        writer.register_subscription("sensor_accel_fifo", FIFO_FIELDS, multi_id)
        writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, multi_id)

    for multi_id in (0, 1):
        anchor = start_us
        for packet in range(20):
            samples = 20 if packet % 3 else 19          # intentionally varying packet fill
            dt = 125.0 if multi_id == 0 else 136.5333   # clean 8 kHz vs ICM-style chip delta
            anchor += samples * dt
            writer.write_data("sensor_gyro_fifo", multi_id,
                              _fifo_packet(anchor + 250, anchor, gyro_ids[multi_id], dt,
                                           0.001065, samples, 30000,
                                           tail_nonzero=(packet == 0)))
            writer.write_data("sensor_accel_fifo", multi_id,
                              _fifo_packet(anchor + 250, anchor, accel_ids[multi_id], dt,
                                           0.00479, samples, 20000,
                                           tail_nonzero=(packet == 0)))

    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 0)
    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 1)
    for row in _logger_status_rows(start_us, 3, 0, 3, True, 8192, buffer_used=2048):
        writer.write_data("logger_status", 0, row)
    for row in _logger_status_rows(start_us, 3, 1, 1, True, 12288, buffer_used=1024):
        writer.write_data("logger_status", 1, row)

    writer.register_subscription("vehicle_imu_status", VEHICLE_IMU_STATUS_FIELDS, 0)
    writer.register_subscription("vehicle_imu_status", VEHICLE_IMU_STATUS_FIELDS, 1)
    writer.write_data("vehicle_imu_status", 0,
                      _vehicle_imu_status_row(start_us, accel_ids[0], gyro_ids[0], clipping=(0, 0, 12)))
    writer.write_data("vehicle_imu_status", 1,
                      _vehicle_imu_status_row(start_us, accel_ids[1], gyro_ids[1]))

    writer.write_sync()


def scenario_mask_gyro_instance1(writer):
    """Only gyro instance 1 publishes (mask 2 expected)."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=2)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 1)
    anchor = 3_000_000
    for _packet in range(10):
        anchor += 20 * 125.0
        writer.write_data("sensor_gyro_fifo", 1,
                          _fifo_packet(anchor + 250, anchor, 0x380013, 125.0, 0.001065, 20, 15000))


def scenario_no_fifo(writer):
    """A normal log without any raw FIFO topic."""
    _write_params(writer, profile=1, acc_mask=1, gyr_mask=1, sdlog_mode=0)
    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 0)
    for row in _logger_status_rows(4_000_000, 2, 0, 3, True, 8192, buffer_used=512):
        writer.write_data("logger_status", 0, row)


def scenario_errors(writer):
    """Anomalies that the checker must detect."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=1)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 0)

    writer.write_data("sensor_gyro_fifo", 0,
                      {**_fifo_packet(5_000_000, 5_000_000, 0x36001A, 0.0, 0.001, 0, 0)})
    writer.write_data("sensor_gyro_fifo", 0,
                      {**_fifo_packet(5_000_500, 5_000_500, 0x36001A, 125.0, 0.0, 20, 100)})
    writer.write_data("sensor_gyro_fifo", 0,
                      {**_fifo_packet(5_001_000, 5_001_000, 0x36001A, 125.0, float("inf"), 20, 100)})

    anchor = 5_002_000
    saturated_packet = _fifo_packet(anchor + 200, anchor, 0x36001A, 125.0, 0.001, 20, 100,
                                    saturated=True)
    writer.write_data("sensor_gyro_fifo", 0, saturated_packet)
    # identical repeat of the previous packet -> duplicate
    writer.write_data("sensor_gyro_fifo", 0, saturated_packet)
    # near full scale but below saturation
    writer.write_data("sensor_gyro_fifo", 0,
                      _fifo_packet(anchor + 250, anchor + 50, 0x36001A, 125.0, 0.001, 20, 31500))
    # non-monotonic timestamp
    writer.write_data("sensor_gyro_fifo", 0,
                      _fifo_packet(anchor - 100, anchor + 50, 0x36001A, 125.0, 0.001, 20, 100))
    # timing anomaly: anchor jumps far beyond samples * dt
    writer.write_data("sensor_gyro_fifo", 0,
                      _fifo_packet(anchor + 300 + 400_000, anchor + 300 + 400_000, 0x36001A,
                                   125.0, 0.001, 20, 100))
    # device change mid stream
    writer.write_data("sensor_gyro_fifo", 0,
                      _fifo_packet(anchor + 301 + 400_000, anchor + 301 + 400_000, 0x99001A,
                                   125.0, 0.001, 20, 100))
    writer.write_dropout(12)


def scenario_health_missing(writer):
    """FIFO data without any health topic: missing evidence, not healthy."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=1)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 0)
    anchor = 6_000_000
    for _packet in range(5):
        anchor += 20 * 125.0
        writer.write_data("sensor_gyro_fifo", 0,
                          _fifo_packet(anchor + 250, anchor, 0x36001A, 125.0, 0.001065, 20, 8000))


def scenario_mission_inactive(writer):
    """Mission logger instance inactive (buffer 0) must not be reported as a fault."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=1)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 0)
    anchor = 7_000_000
    for _packet in range(5):
        anchor += 20 * 125.0
        writer.write_data("sensor_gyro_fifo", 0,
                          _fifo_packet(anchor + 250, anchor, 0x36001A, 125.0, 0.001065, 20, 8000))

    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 0)
    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 1)
    for row in _logger_status_rows(anchor, 2, 0, 3, True, 8192, buffer_used=4096):
        writer.write_data("logger_status", 0, row)
    for row in _logger_status_rows(anchor, 2, 1, 1, False, 0, buffer_used=0):
        writer.write_data("logger_status", 1, row)


def scenario_health_rows_mixed(writer):
    """logger_status rows where only some rows are active full-file records."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=1)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 0)
    anchor = 8_000_000
    for _packet in range(3):
        anchor += 20 * 125.0
        writer.write_data("sensor_gyro_fifo", 0,
                          _fifo_packet(anchor + 250, anchor, 0x36001A, 125.0, 0.001065, 20, 8000))

    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 0)
    # row 0: active full log with buffer_size_bytes = 0 -> inconsistent, must be reported
    writer.write_data("logger_status", 0, _logger_status_rows(anchor, 1, 0, 3, True, 0, buffer_used=0)[0])
    # rows 1..2: active full file with 85% buffer fill -> pressure warning
    for row in _logger_status_rows(anchor + 1_000_000, 2, 0, 3, True, 8192, buffer_used=7000):
        writer.write_data("logger_status", 0, row)


def scenario_health_inactive_only(writer):
    """No active full-file logger_status row: acceptance evidence is missing."""
    _write_params(writer, profile=256, acc_mask=1, gyr_mask=1)
    writer.register_subscription("sensor_gyro_fifo", FIFO_FIELDS, 0)
    anchor = 9_000_000
    for _packet in range(3):
        anchor += 20 * 125.0
        writer.write_data("sensor_gyro_fifo", 0,
                          _fifo_packet(anchor + 250, anchor, 0x36001A, 125.0, 0.001065, 20, 8000))

    writer.register_subscription("logger_status", LOGGER_STATUS_FIELDS, 1)
    for row in _logger_status_rows(anchor, 2, 1, 1, False, 0, buffer_used=0):
        writer.write_data("logger_status", 1, row)


SCENARIOS = {
    "nominal": scenario_nominal,
    "mask_gyro_instance1": scenario_mask_gyro_instance1,
    "no_fifo": scenario_no_fifo,
    "errors": scenario_errors,
    "health_missing": scenario_health_missing,
    "mission_inactive": scenario_mission_inactive,
    "health_rows_mixed": scenario_health_rows_mixed,
    "health_inactive_only": scenario_health_inactive_only,
}


def build(scenario, path, start_timestamp_us=1_000_000):
    if scenario not in SCENARIOS:
        raise ValueError("unknown scenario %r (known: %s)" % (scenario, ", ".join(sorted(SCENARIOS))))
    with UlogWriter(path, start_timestamp_us=start_timestamp_us) as writer:
        SCENARIOS[scenario](writer)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate synthetic ULog files for the vibration tools")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), help="scenario to write")
    parser.add_argument("--out", help="output .ulg path (single scenario)")
    parser.add_argument("--out-dir", help="write every scenario into this directory")
    parser.add_argument("--list", action="store_true", help="list scenarios and exit")
    args = parser.parse_args(argv)

    if args.list or (not args.scenario and not args.out_dir):
        print("Scenarios:")
        for name in sorted(SCENARIOS):
            print("  %-20s %s" % (name, SCENARIOS[name].__doc__.strip().splitlines()[0]))
        return 0

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        for name in sorted(SCENARIOS):
            path = os.path.join(args.out_dir, "%s.ulg" % name)
            build(name, path)
            print("wrote %s (%d bytes)" % (path, os.path.getsize(path)))
        return 0

    if not args.out:
        print("error: --out is required with --scenario", file=sys.stderr)
        return 2
    build(args.scenario, args.out)
    print("wrote %s (%d bytes)" % (args.out, os.path.getsize(args.out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
