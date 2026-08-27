# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# SITL (Software-in-the-Loop) — most common dev target
make px4_sitl_default            # Build SITL (POSIX/Linux)
make px4_sitl_default sitl_gazebo-classic  # Build + run Gazebo SITL

# Hardware targets
make px4_fmu-v6x_default         # Build for Pixhawk FMUv6x
make px4_fmu-v5x_default         # Build for Pixhawk FMUv5x
make <vendor>_<board>_default    # General pattern (omit _default shorthand works)
make list_config_targets         # List all available board targets

# Testing
make tests                       # Build + run unit tests (SITL test config)
make tests TESTFILTER=<regex>    # Run specific tests via ctest -R filter
make tests_integration           # Build + run MAVSDK integration tests (requires Gazebo)
make tests_coverage              # Tests with lcov coverage report

# Code quality
make check_format                # Check formatting with astyle (CI gate)
make format                      # Auto-format with astyle
make clang-tidy                  # Run clang-tidy static analysis (requires clang build)
make cppcheck                    # Run cppcheck
make check                       # Full gate: build SITL+NuttX targets + tests + format

# Metadata / docs
make px4_metadata                # Generate parameters & airframes metadata
make parameters_metadata         # Parameters XML generation
make airframe_metadata           # Airframe reference generation
make doxygen                     # Build Doxygen API docs

# Cleanup
make clean                       # Clean all build directories
make submodulesupdate            # Update git submodules
```

Build outputs land in `build/<config_target>/`. With Ninja available, `-j` controls parallelism. Build uses CMake internally — the Makefile is a wrapper around `cmake --build`. C++14, C11.

## Architecture Overview

### uORB Messaging (Inter-Module Communication)

PX4 uses an asynchronous publish-subscribe IPC system called uORB. Message definitions are `.msg` files in `msg/`. The build system generates C++ headers from these. Modules publish topics and subscribe to topics they depend on — this is the primary data flow between modules. Parameters are also defined declaratively and published as uORB topics.

### Module System

Every functional unit is a **module** — a static library linked into the PX4 binary (NuttX) or a shared library (POSIX dynamic loading). Modules are defined via `px4_add_module()` in their `CMakeLists.txt`:

```cmake
px4_add_module(
    MODULE modules__mc_att_control
    MAIN mc_att_control
    SRCS mc_att_control_main.cpp mc_att_control.hpp
    DEPENDS AttitudeControl mathlib px4_work_queue
)
```

Each module's entry point is `<MAIN>_app_main()`. Module parameters and metadata are declared in a `module.yaml` file. The YAML is used to auto-generate parameter documentation and C++ parameter accessors.

### Directory Layout

```
src/
  modules/       — Flight stack: estimator (EKF2), controllers (mc/fw/vtol), commander, navigator, logger, mavlink
  drivers/       — Hardware drivers: IMU, barometer, GPS, magnetometer, actuators, distance sensors, optical flow
  lib/           — Shared libraries: mathlib, matrix, parameters, pid, controllib, mixer_module, modes, geo, perf
  include/       — Global headers
  systemcmds/    — System command-line tools
  examples/      — Example/skeleton modules
  templates/     — Template for creating new modules
msg/             — uORB message definitions (*.msg)
platforms/
  nuttx/         — NuttX RTOS (flight controller hardware)
  posix/         — Linux/POSIX (SITL)
  common/        — Shared platform-independent code
  qurt/          — Qualcomm Hexagon DSP
  ros2/          — ROS 2 integration
boards/          — Board configs per manufacturer/board: .px4board files, NuttX defconfig, init scripts, bootloader
ROMFS/
  px4fmu_common/init.d/  — Shell init scripts that start modules at boot time
    rc.sensors    — Sensor driver startup
    rc.mc_apps    — Multicopter controller startup
    rc.fw_apps    — Fixed-wing controller startup
    rc.vehicle_setup — Routes to vehicle-type-specific startup
Tools/           — Build helpers, CI scripts, sim models, parameter tools
cmake/           — CMake modules (px4_add_module, px4_add_library, px4_config, etc.)
test/            — Integration test scripts (ROS, MAVSDK)
test_data/       — Test data files
```

### Boot Sequence

1. NuttX boots → starts `init` → runs `rcS` shell script
2. `rcS` detects airframe from autostart ID, sets `VEHICLE_TYPE`
3. Platform init runs, then `rc.vehicle_setup` sources vehicle-specific scripts (e.g., `rc.mc_apps`)
4. Vehicle scripts `start` modules in order: sensors → control allocator → rate controller → attitude controller → position controller → land detector
5. `commander` module performs pre-arm checks, then arms the vehicle

### Key Flight Stack Modules (Control Pipeline)

```
Sensors → EKF2 (estimator) → Flight Mode Manager → Position Controller → Attitude Controller → Rate Controller → Control Allocator → Actuators
```

- **ekf2** — Extended Kalman Filter for state estimation (attitude, position, velocity)
- **flight_mode_manager** — Translates flight mode (Manual, Altitude, Position, Mission, etc.) into setpoints
- **mc_pos_control / fw_pos_control** — Position control (setpoint → thrust/attitude setpoint)
- **mc_att_control / fw_att_control** — Attitude control (attitude setpoint → rate setpoint)
- **mc_rate_control / fw_rate_control** — Rate control (rate setpoint → actuator torque setpoint)
- **control_allocator** — Maps torque/thrust setpoints to individual actuator outputs (mixer)
- **commander** — State machine: arm/disarm, flight mode switching, failsafes, pre-arm checks
- **navigator** — Mission state machine (waypoint sequencing, RTL, takeoff, land)
- **mavlink** — MAVLink protocol communication with GCS/offboard
- **logger** — uORB topic logging to SD card (ulog format)

### Coding Conventions

- **Tabs** for indentation (tab width 8), no spaces
- **No trailing whitespace**, files end with newline
- **AStyle** formatting via `Tools/astyle/astylerc`
- **C++14** (`CMAKE_CXX_STANDARD 14`), C11 for C files
- **Parameter definitions** in `module.yaml`, never hardcoded literals
- **Error handling**: `PX4_WARN()`, `PX4_ERR()`, `PX4_INFO()`, `PX4_DEBUG()` macros, no exceptions
- Header guard style: `#pragma once`
- Module entry point naming: `<name>_app_main()` which is aliased via `PX4_MAIN=<name>_app_main`
- Include paths: use module-relative paths (modules depend on exported interface targets from `DEPENDS`)

### Development Environment

Recommended: VS Code with devcontainer (`px4io/px4-dev-nuttx-focal` Docker image). Settings are pre-configured in `.vscode/settings.json`. The devcontainer includes all cross-compilers for NuttX targets.

For local setup, use the PX4 toolchain installer scripts or the Docker-based build:
```bash
./Tools/docker_run.sh make px4_sitl_default
```
