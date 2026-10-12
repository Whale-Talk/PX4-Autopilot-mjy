# Raw FIFO vibration tooling (CUAV X7+ / H1)

Offline tooling for the raw IMU FIFO logging feature (C1, `SDLOG_ACC_MASK` /
`SDLOG_GYR_MASK`). It has two jobs:

1. **Configuration check** before a capture run (no hardware needed).
2. **ULog acceptance check** on the recorded file, plus synthetic files and the
   host-side functional integration test so the logging chain itself is exercised.

Nothing here changes firmware behaviour. No dependency beyond the repository's own
`Tools/setup/requirements.txt` (pyulog, numpy) is required; the checks themselves
run with the standard library only when pyulog is absent (the report always states
which reader was used).

## Files

| File | Purpose |
|---|---|
| `px4_vibration_config_check.py` | offline check of `SDLOG_PROFILE` bits 8/9, both masks, build instance limit, `logger_topics.txt` replacement conflicts |
| `px4_ulog_vibration_check.py` | offline acceptance check of a recorded ULog (FIFO streams + health evidence) |
| `ulog_reader.py` | acceptance reader (pyulog, filtered to the four relevant topics) plus a clearly labelled diagnostic fallback |
| `vibration_analysis.py` | data semantics: valid prefix, scaling, `dt` anchors, per-segment residuals/rates, health interpretation |
| `make_synthetic_ulog.py` | writes real ULog test files (8 scenarios) |
| `test_vibration_tools.py` | local test suite (`python3 Tools/vibration/test_vibration_tools.py -v`) |
| `logger_topics.example.txt` | example + warnings for the SD-card topic list (advanced path) |

## Quick start

```bash
# 1) configuration (params from `param show`, optional SD-card topic list)
python3 Tools/vibration/px4_vibration_config_check.py --params params.txt \
        --topics-file /media/sdcard/etc/logging/logger_topics.txt

# 2) after the capture
python3 Tools/vibration/px4_ulog_vibration_check.py log_XX.ulg \
        --expect-gyr-mask 1 --expect-acc-mask 1 --json report.json

# 3) local tests
cd Tools/vibration && python3 test_vibration_tools.py -v

# 4) real uORB -> logger -> ULog integration (host functional test, see below)
export PX4_VIBRATION_INTEGRATION_DIR="$(mktemp -d)"
make tests TESTFILTER=LoggerFifoIntegration
```

Exit codes: `0` no error, `1` at least one ERROR finding, `2` usage/IO failure.
Warnings never fail the run.

`--params` accepts real `param show` output (status markers and `[index,used]`
brackets are handled; see `src/systemcmds/param/param.cpp:680`) as well as plain
`NAME VALUE` / `NAME: VALUE` files. Values are never truncated: a malformed or
fractional `SDLOG_*` value is an explicit ERROR instead of a silent default.
`logger_topics.txt` parsing reproduces the firmware `ntopics > 0` replacement rule
for ordinary input and reports lines whose syntax it cannot emulate (79+ byte lines,
numeric-prefix or out-of-range fields) as `topics-file/ambiguous-line` instead of
claiming exact sscanf equivalence.

## Data semantics implemented (from the v1.16.1 sources)

| Topic field | Meaning |
|---|---|
| `samples` | number of valid entries at the start of `x/y/z` (1..32); entries at index >= `samples` are padding and are excluded |
| `x/y/z` | raw int16, FRD board frame, driver rotation applied; physical value = `raw * scale` (acc m/s², gyro rad/s); **no full calibration** |
| `timestamp_sample` | nominal anchor of the **last** sample of the batch; `t_i = T - (N - 1 - i) * dt` |
| `dt` | microseconds |
| cross-packet residual | `T[k] - T[k-1] - N[k] * dt[k]`, compared against `--gap-tolerance-us`; computed **only inside one `device_id` segment** |
| endpoint rate | `sum(N[k] for k >= 1) / (T_last - T_first) * 1e6`, computed **per `device_id` segment** (a span across devices would be a false rate) |
| `device_id` | change segments the stream; accel and gyro instances are mapped separately; `device_id = 0` is reported as missing driver mapping evidence |
| publication latency | `timestamp - timestamp_sample`, i.e. the **driver publication timestamp** minus the batch anchor — not the SD-card write time |
| raw values | values reaching the int16 numeric limit are reported as a numeric-limit observation only; **true clipping is judged from the driver counters** (`vehicle_imu_status.accel_clipping` / `gyro_clipping`) |

Chip caveats that must not become hard assertions: ICM42688P `dt` is the last chip
timestamp delta of the batch (quantised ≈17.0667 µs, may vary within a batch);
ICM20689 accel phase can differ from the gyro by ~125 µs, so the two streams are
not strictly sample synchronous.

Raw saturation (int16 full scale) is reported separately from "near full scale":
a large but non-saturated value is not clipping.

## What a passing result does and does not mean

* A run with no ERROR means **no anomaly was found at the checked level**; if there
  are warnings the CLI says that they **need review** instead of claiming a clean run.
* A timing residual above tolerance is reported as a timing observation: it can come
  from chip timestamp quantisation, publication jitter or real loss. The tools do
  **not** convert it into a claimed number of dropped samples.
* Health is only judged from actual evidence: ULog dropout records, `logger_status`
  and `vehicle_imu_status`. `logger_status` is evaluated **row by row**: only rows
  with `type = FULL`, `is_logging` and a file backend count as acceptance evidence,
  and the buffer ratio is taken from those rows. An inactive mission log
  (`is_logging = False`, `buffer_size_bytes = 0`) is not a fault; if no active
  full-file row exists at all, the run reports **missing acceptance evidence**.
  Missing health topics are reported as **missing evidence**, never as healthy.
  `message_gaps` counts generation jump events, not samples.
* `--orb-multi-max` (1..10) is **user supplied**: the build's instance limit cannot
  be proven from log metadata. It only affects how masks are interpreted.
* Buffer size / Flash numbers are not device performance results. Nothing here
  measures CPU load, SD write behaviour or flight behaviour.

## Synthetic scenarios

| Scenario | Content | Expected outcome |
|---|---|---|
| `nominal` | 2 accel + 2 gyro instances, varying packet fill, padding tail non-zero, health topics | no ERROR; prefix-only analysis |
| `mask_gyro_instance1` | only gyro instance 1 | selection check per `--expect-*` |
| `no_fifo` | normal log without FIFO topics | `fifo/no-data` warning |
| `errors` | zero dt, zero/infinite scale, `samples=0`, non-monotonic timestamp, duplicate packet, residual jump, device change, saturation, near-full-scale, one dropout | all reported |
| `health_missing` | FIFO data, no health topics | missing-evidence warnings |
| `mission_inactive` | mission logger inactive (`buffer_size_bytes = 0`) | not flagged as a fault |
| `health_rows_mixed` | full-log rows where one row has `buffer_size_bytes = 0` and others are 85% full | per-row gating: zero-buffer row + buffer-pressure warning |
| `health_inactive_only` | FIFO data, only inactive mission rows | `health/no-active-full-file-row` (missing evidence) |

## Host integration test (`functional-LoggerFifoIntegration`)

The uORB → logger → ULog chain is exercised by the firmware-side functional test
`src/modules/logger/LoggerFifoIntegrationTest.cpp` (built by the logger
CMakeLists; this tooling does not modify it). It publishes synthetic
`sensor_accel_fifo` / `sensor_gyro_fifo` batches on the host and lets the real
logger module write a ULog.

```bash
export PX4_VIBRATION_INTEGRATION_DIR="$(mktemp -d)"   # parent dir for the output
make tests TESTFILTER=LoggerFifoIntegration           # or: make functional-LoggerFifoIntegration
```

The test writes `logger_fifo_expected.json` into that directory; it records the
produced `ulog` path, `sdlog_profile` 768, `sdlog_acc_mask` 2, `sdlog_gyr_mask` 5
(accel instance 1, gyro instances 0 and 2), the published/unselected instances,
`batches`, `samples_per_batch` 32, `dt_us`, `scale` and the expected value formula
per axis. Run the conformance check on the ULog named in that manifest:

```bash
python3 Tools/vibration/px4_ulog_vibration_check.py \
        "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["ulog"])' \
           "$PX4_VIBRATION_INTEGRATION_DIR/logger_fifo_expected.json")" \
        --expect-acc-mask 2 --expect-gyr-mask 5 --orb-multi-max 10
```

Coverage boundary: this is the **host synthetic publisher** path with the real
logger write chain (32 samples per batch, profile 768, masks 2/5). It proves the
selection/scaling/serialisation path, and it is **not** a measurement of X7+ CPU
load, SD-card bandwidth or flight behaviour.

## Remaining gaps

* The multi-instance + accel-FIFO chain is covered by the host functional test
  above; the same selection has not yet been accepted on the X7+ hardware (that is
  the device-campaign step, see `docs/vibration/operation-guide.md`).
* The synthetic files exercise the analysis/health logic and are cross-checked
  against pyulog when it is installed. In this environment pyulog is absent, so six
  pyulog-backed tests **skip**; run them once with `Tools/setup/requirements.txt`
  installed (the pyulog acceptance run and the real-log check are the repository
  environment's responsibility).
