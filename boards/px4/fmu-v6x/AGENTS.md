<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-06-27 | Updated: 2026-06-27 -->

# FMUv6x (Pixhawk 6X)

## Purpose

Standard FMUv6x board configuration for Pixhawk 6X flight controller hardware. Based on STM32H7 (Cortex-M7), this is the reference implementation supporting dual IMU, triple power domains, Ethernet, CAN, and PX4IO co-processor.

## Key Files

| File | Description |
|------|-------------|
| `default.px4board` | Kconfig overrides: toolchain, serial port mappings, HW drivers (BMI088, ICM20649, ICM42688P, BMP388, etc.), modules |
| `multicopter.px4board` | Multicopter-specific build config extending default |
| `rover.px4board` | Rover-specific build config |
| `bootloader.px4board` | Bootloader build configuration |
| `flash-analysis.px4board` | Special build for flash memory analysis |
| `zenoh.px4board` | Zenoh (DDS) debug/config build |
| `performance-test.px4board` | Performance testing build config |
| `firmware.prototype` | Firmware binary pattern/naming template |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `src/` | Board support sources: GPIO config, init, bus setup, bootloader |
| `nuttx-config/` | NuttX RTOS config: defconfigs, linker scripts, board headers, Kconfig |
| `init/` | Board-specific init scripts (rc.board_defaults, rc.board_mavlink, rc.board_sensors) |
| `cmake/` | CMake helpers for upload/flash |
| `extras/` | Prebuilt binaries: bootloader, IO firmware |

## Hardware Configuration Summary

### MCU: STM32H743IIK6 (Cortex-M7, 480MHz)

### Key Pin Assignments (board_config.h)
- **USB OTG FS VBUS**: PA9 (input with pulldown)
- **nARMED LED**: PE6 (open-drain output, goes LOW when armed)
- **Status LEDs**: PE3 (RED), PE4 (GREEN), PE5 (BLUE) — shared with TRACE pins
- **Buzzer/Tone Alarm**: Timer 14, CH1 on PF9
- **HRT (High Resolution Timer)**: Timer 8, CH3
- **RC Input**: USART6 `/dev/ttyS5` (half-duplex, single-wire, RX/TX swapped)
- **PX4IO Serial**: USART6 `/dev/ttyS5`, 1.5Mbps
- **Spektrum Power**: PH2
- **SD Card**: SDIO slot 0, power via PC13
- **Ethernet**: Power enable on PG15
- **Heater**: PB10 (Timer 2 CH3)
- **NFC**: PC0 (EXTI input)
- **Safety Switch**: Input PF5, LED output PD10
- **ADC**: 9 channels (VDD sensors, 5V, HW ver/rev sense)
- **SPI External Reset**: PF10

### Power Domains
- 2x Brick valid inputs (PG1, PG2) via LTC44XX
- USB valid input (PG3)
- 5V Peripheral enable (PG4)
- 5V High-power enable (PG10)
- 3.3V Sensors4 enable (PG8)
- 3.3V Spektrum enable (PH2)
- 3.3V SD Card enable (PC13)

### Sensor Bus Configuration (SPI)
- `BOARD_NUM_SPI_CFG_HW_VERSIONS 7` — supports 7 hardware revisions (V6X_0 through V6X_16)
- HW version detection via ADC3 on PH3/PH4 with `BOARD_HAS_HW_SPLIT_VERSIONING`

### I2C
- 2 EEPROMs (MTD bus 4,5)
- SE050 secure element on address 0x48
- BMP388 DRDY on I2C2 (PG5)

### DMA Pool: 5120 bytes

## For AI Agents

### Working In This Directory
- **Do NOT modify** pin definitions in `src/board_config.h` unless the hardware design changes — these must match the schematic exactly
- When adding new sensor drivers, update `default.px4board` with `CONFIG_DRIVERS_*` entries
- Board init scripts in `init/` use NuttX shell syntax — test with `ver hwbasecmp` and `ver hwtypecmp` for hardware-conditional startup
- NuttX config changes in `nuttx-config/nsh/defconfig` affect kernel behavior — review with NuttX documentation

### Testing Requirements
- Build target: `make px4_fmu-v6x_default`
- Bootloader builds separately via bootloader.px4board configuration
- Flash via `make px4_fmu-v6x_default upload`
- All sensor startup paths must be tested on physical hardware for each HW revision

### Common Patterns
- `ver hwbasecmp` is used in init scripts to branch on hardware base revision
- `ver hwtypecmp` branches on sensor set type
- Board config macros use negative logic for enables (e.g., `GPIO_VDD_5V_PERIPH_nEN`) — the `VDD_5V_PERIPH_EN()` macro inverts automatically

## Dependencies

### Internal
- `boards/px4/fmu-v6x_seeex25d/` — SEEEX25D 定制版（H1 飞控），基于本板派生，独立传感器组合（早期曾用名 mjy，现已统一）(see `fmu-v6x_seeex25d/AGENTS.md` for full diff)
- `ROMFS/px4fmu_common/init.d/rc.board_defaults` — shared board defaults
- `src/drivers/` — All hardware drivers referenced in default.px4board

### External
- STM32H7 HAL (NuttX arch layer)
- arm-none-eabi toolchain (Cortex-M7)

<!-- MANUAL: -->
