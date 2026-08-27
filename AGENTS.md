# Repository Guidelines

## Project Structure & Module Organization

PX4 is a CMake-based autopilot with Makefile wrappers. Core flight code lives in `src/`: `src/modules/` contains flight stack applications, `src/drivers/` hardware drivers, `src/lib/` shared libraries, and `src/include/` common headers. uORB message definitions are in `msg/`. Board targets and NuttX configs are under `boards/`, platform code under `platforms/`, startup scripts under `ROMFS/px4fmu_common/init.d/`, build and simulation helpers under `Tools/`, and integration tests under `test/`. Build artifacts are generated in `build/<config_target>/`.

## Build, Test, and Development Commands

- `make px4_sitl_default`: build the default SITL target for local Linux/POSIX development.
- `make px4_sitl_default sitl_gazebo-classic`: build and run Gazebo Classic SITL.
- `make px4_fmu-v6x_default`: build a representative NuttX hardware target.
- `make list_config_targets`: list supported board configuration targets.
- `make tests`: build and run unit tests.
- `make tests TESTFILTER=<regex>`: run a focused CTest subset.
- `make tests_integration`: run MAVSDK integration tests; requires simulator dependencies.
- `make check_format` and `make format`: check or apply AStyle formatting.
- `make check`: broad pre-submit gate including builds, tests, and format checks.

## Coding Style & Naming Conventions

Use tabs for indentation, tab width 8, no trailing whitespace, and end files with a newline. C++ code targets C++14; C code targets C11. Run the repository AStyle configuration in `Tools/astyle/astylerc` before submitting. Prefer `#pragma once` for headers, PX4 logging macros such as `PX4_WARN()` and `PX4_ERR()`, and module entry points named `<module>_app_main()`. Add modules through `px4_add_module()` in the local `CMakeLists.txt`, with parameter metadata in `module.yaml` when applicable.

## Testing Guidelines

Place unit tests near the code they exercise, commonly in `test/` subdirectories with names like `*Test.cpp` or `test_*.cpp`. Use `make tests TESTFILTER=<module-or-case>` while iterating, then run the wider target relevant to the change. For flight behavior, include SITL results, bench results, or flight logs when opening a PR.

## Commit & Pull Request Guidelines

Recent history uses concise, scoped subjects such as `drivers/gps: prioritize non-blocking reads over injection`. Keep the first line imperative and specific, and reference issues with `Fixes #123` or `Relates #123` when relevant. Pull requests should describe the behavior change, affected targets, test commands run, and include logs or screenshots for flight, UI, or simulation-visible changes.

## Agent-Specific Instructions

Do not edit generated build output in `build/`. Treat vendored trees and submodules under `platforms/nuttx/NuttX/`, `src/modules/*/mavlink/`, and similar third-party directories carefully; prefer upstream-compatible patches and avoid unrelated formatting churn.
