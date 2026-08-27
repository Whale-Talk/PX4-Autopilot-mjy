<!-- Parent: ../../AGENTS.md -->
<!-- Generated: 2026-06-27 | Updated: 2026-06-27 -->

# PX4 Board Configurations

## Purpose

Contains board-level configurations for all supported PX4 flight controller hardware targets. Each subdirectory defines the board-specific build config (`.px4board`), NuttX RTOS configuration, hardware pin definitions (`board_config.h`), and board initialization scripts.

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `fmu-v2/` | Legacy FMUv2 (Pixhawk 1, STM32F4) |
| `fmu-v3/` | FMUv3 (Pixhawk 1 2MB flash) |
| `fmu-v4/` | FMUv4 (Pixracer, STM32F4) |
| `fmu-v4pro/` | FMUv4 Pro variant |
| `fmu-v5/` | FMUv5 (Pixhawk 4, STM32F7) |
| `fmu-v5x/` | FMUv5x variant |
| `fmu-v6c/` | FMUv6C (cost-optimized) |
| `fmu-v6u/` | FMUv6U (ultra) |
| `fmu-v6x/` | **FMUv6x (Pixhawk 6X, STM32H7)** — 标准版 (see `fmu-v6x/AGENTS.md`) |
| `fmu-v6x_seeex25d/` | **FMUv6x SEEEX25D 定制版（H1 飞控）** — 基于 fmu-v6x 派生，独立传感器组合 (see `fmu-v6x_seeex25d/AGENTS.md`) |
| `fmu-v6xrt/` | FMUv6x RT (Real-Time variant) |
| `io-v2/` | PX4IO co-processor v2 firmware |
| `raspberrypi/` | Raspberry Pi board config |
| `ros2/` | ROS2 board config |
| `sitl/` | Software-in-the-Loop (simulation) |

## Board Configuration Pattern

Each board directory follows this structure:

```
<board-name>/
  default.px4board          # Kconfig-based build config (required)
  <variant>.px4board        # Optional variant configs (multicopter, rover, etc.)
  bootloader.px4board       # Bootloader build config
  firmware.prototype        # Output binary naming pattern
  src/                      # Board-specific C/C++ sources
    board_config.h           # GPIO, peripheral, and pin definitions
    hw_config.h             # Hardware configuration constants
    init.cpp                # Board initialization
    i2c.cpp, spi.cpp        # Bus initialization
    usb.c, can.c, led.c     # Peripheral setup
    timer_config.cpp        # Timer/channel configuration
  nuttx-config/             # NuttX RTOS configuration
    nsh/defconfig            # NuttX kernel config
    bootloader/defconfig     # Bootloader kernel config
    include/                 # Board headers
    scripts/                 # Linker scripts
  init/                     # Board-specific init scripts
    rc.board_defaults        # Default parameters
    rc.board_mavlink         # MAVLink config
    rc.board_sensors         # Sensor startup sequence
  cmake/                    # Build helpers (upload scripts)
  extras/                   # Prebuilt binaries (bootloader, IO firmware)
```

## For AI Agents

### Working In This Directory
- **FMUv6x** 和 **FMUv6x_seeex25d（H1 飞控）** 是当前关注重点，后者基于前者派生（早期曾用名 mjy，现已统一为 seeex25d）
- 创建新的板卡变体时，从最近接的现有板卡复制，而非从头开始
- `.px4board` 文件是 Kconfig 片段，使用 `CONFIG_*` 变量选择驱动和功能模块
- `board_config.h` 中的 GPIO 宏必须与原理图严格对应
- NuttX defconfig 通过 `make menuconfig` 或 `boards/<board>/nuttx-config/nsh/defconfig` 手动维护

### SEEEX25D vs Original fmu-v6x 差异速查
- `fmu-v6x_seeex25d` 是 `fmu-v6x` 的派生版本，适配独立传感器组合
- **传感器配置**: SPI1=ICM20602, SPI2=ICM42688P, SPI3=BMI088(A+G), I2C4=BMP388(内,0x77), I2C2=BMP388(外,0x76), I2C4=QMC5883P
- **HW 版本**: 电阻 33.4kΩ+57.4kΩ → ordinal 7 → V6X007
- **USB**: 与原版 fmu-v6x 完全相同 (硬件已解决)
- NuttX 配置仅路径不同，其余配置功能等价

### Testing Requirements
- 每个板卡变体须独立编译测试: `make px4_<board>_default`
- 硬件板卡必须在真实硬件上验证传感器初始化流程
- 派生板卡需回归测试与原版的差异点

### Common Patterns
- `ver hwbasecmp` / `ver hwtypecmp` 在 init 脚本中用于硬件版本条件分支
- 板卡宏命名: `BOARD_HAS_*` (功能检测), `GPIO_*` (引脚定义)
- nEN 前缀表示负逻辑使能

## Dependencies

### Internal
- `ROMFS/px4fmu_common/init.d/` — 共享的 PX4 系统初始化脚本
- `platforms/nuttx/` — NuttX RTOS 平台代码
- `cmake/` — PX4 CMake 模块 (px4_add_board 等)
- `src/drivers/` — 板卡引用的所有硬件驱动

### External
- arm-none-eabi toolchain (for STM32 targets)
- NuttX RTOS (submodule in `platforms/nuttx/NuttX/`)

<!-- MANUAL: -->
