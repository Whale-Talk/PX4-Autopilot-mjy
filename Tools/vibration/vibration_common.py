# -*- coding: utf-8 -*-
"""
Shared constants and small helpers for the PX4 raw-FIFO vibration tooling.

Semantics encoded here follow the PX4 v1.16.1 sources that produce the data:
  * msg/SensorAccelFifo.msg, msg/SensorGyroFifo.msg, msg/LoggerStatus.msg,
    msg/VehicleImuStatus.msg  (field names, units)
  * src/modules/logger/logged_topics.cpp  (profile bits and instance masks)
  * src/modules/logger/params.c           (parameter names, ranges, defaults)
"""

# --- logger topic names ---------------------------------------------------------
TOPIC_ACCEL_FIFO = "sensor_accel_fifo"
TOPIC_GYRO_FIFO = "sensor_gyro_fifo"
TOPIC_LOGGER_STATUS = "logger_status"
TOPIC_VEHICLE_IMU_STATUS = "vehicle_imu_status"

FIFO_TOPICS = (TOPIC_ACCEL_FIFO, TOPIC_GYRO_FIFO)

# --- SDLOG_PROFILE bits (logged_topics.h SDLogProfileMask) ----------------------
PROFILE_BIT_GYRO_FIFO = 8        # RAW_IMU_GYRO_FIFO  = 1 << 8  = 256
PROFILE_BIT_ACCEL_FIFO = 9       # RAW_IMU_ACCEL_FIFO = 1 << 9  = 512
PROFILE_MASK_GYRO_FIFO = 1 << PROFILE_BIT_GYRO_FIFO
PROFILE_MASK_ACCEL_FIFO = 1 << PROFILE_BIT_ACCEL_FIFO
SDLOG_PROFILE_MIN = 0
SDLOG_PROFILE_MAX = 2047

# --- instance masks (params.c: SDLOG_ACC_MASK / SDLOG_GYR_MASK) -----------------
PARAM_ACC_MASK = "SDLOG_ACC_MASK"
PARAM_GYR_MASK = "SDLOG_GYR_MASK"
PARAM_PROFILE = "SDLOG_PROFILE"
MASK_MAX_VALUE = 1023            # documented @max (bit 0..9)
MASK_MIN_VALUE = 0
MASK_INVALID_FALLBACK = 1        # implementation falls back to instance 0

# --- build limits ---------------------------------------------------------------
# platforms/common/uORB/uORB.h: 4 with CONSTRAINED_MEMORY, otherwise 10
ORB_MULTI_MAX_INSTANCES_DEFAULT = 10
ORB_MULTI_MAX_INSTANCES_CONSTRAINED = 4

# --- FIFO sample arrays ---------------------------------------------------------
FIFO_SAMPLES_MAX = 32            # int16 x/y/z arrays are 32 entries long
FIFO_DT_QUANTIZATION_ICM42688P_US = 1e6 / 58593.75  # ≈17.0667 us, chip clock quantum
INT16_MAX = 32767
INT16_MIN = -32768

# --- SDLOG_MODE (params.c) ------------------------------------------------------
SDLOG_MODE_VALUES = {
    -1: "disabled",
    0: "when armed until disarm (default)",
    1: "from boot until disarm",
    2: "from boot until shutdown",
    3: "while manual input AUX1 > 30%",
    4: "from 1st armed until shutdown",
}

# --- severity -------------------------------------------------------------------
SEVERITY_ERROR = "ERROR"
SEVERITY_WARNING = "WARNING"
SEVERITY_INFO = "INFO"
_SEVERITY_ORDER = {SEVERITY_ERROR: 0, SEVERITY_WARNING: 1, SEVERITY_INFO: 2}


class Finding(object):
    """One observation produced by a check.

    ``evidence`` must be traceable to the input (parameter name, topic/instance,
    field or topic line) so that a reader can re-check the claim.
    """

    __slots__ = ("severity", "code", "message", "evidence")

    def __init__(self, severity, code, message, evidence=""):
        self.severity = severity
        self.code = code
        self.message = message
        self.evidence = evidence

    def as_dict(self):
        return {
            "severity": self.severity,
            "code": self.code,
            "message": self.message,
            "evidence": self.evidence,
        }

    def __repr__(self):
        return "Finding(%s, %s, %s)" % (self.severity, self.code, self.message)


def count_severity(findings, severity):
    return sum(1 for f in findings if f.severity == severity)


def sort_findings(findings):
    return sorted(findings, key=lambda f: (_SEVERITY_ORDER.get(f.severity, 9), f.code))


def format_findings(findings, indent="  "):
    lines = []
    for f in sort_findings(findings):
        line = "%s[%s] %s: %s" % (indent, f.severity, f.code, f.message)
        if f.evidence:
            line += "  (%s)" % f.evidence
        lines.append(line)
    return "\n".join(lines)


def instances_from_mask(mask):
    """Return the sorted list of instance numbers selected by ``mask``."""
    return [i for i in range(32) if mask & (1 << i)]


def mask_from_instances(instances):
    mask = 0
    for i in instances:
        mask |= 1 << i
    return mask
