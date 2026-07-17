<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-06-27 | Updated: 2026-06-27 -->

# FMUv6x SEEEX25D (定制版)

## Purpose

SEEEX25D 定制版 FMUv6x 飞控板卡配置。基于原版 `fmu-v6x` (Pixhawk 6X) 派生，硬件上解决了 USB VBUS 问题（与原版一致），并适配了独立的传感器组合。

## 传感器配置

| 传感器 | 总线 | CS/地址 | DRDY/备注 |
|--------|------|---------|-----------|
| BMI088 (Accel) | SPI3 | PI4 | DRDY: PI6 |
| BMI088 (Gyro) | SPI3 | PI8 | DRDY: PI7 |
| ICM42688P | SPI2 | PH5 | DRDY: PA10 |
| ICM20602 | SPI1 | PI9 | DRDY: PF2 |
| BMP388 #1 (内) | I2C2 | 0x77 | 内部气压计 |
| BMP388 #2 (外) | I2C2 | 0x76 | 外部气压计，与 #1 同总线 |
| QMC5883P | I2C4 | 0x0D | 外部磁力计 |

## HW 版本检测

- **版本电阻**: 33.4kΩ (R1 上拉至 VDD) + 57.4kΩ (R2 下拉至 GND)
- **检测引脚**: PG0 驱动, PH3 (VER) / PH4 (REV) ADC3 检测
- **对应 ordinal**: 7 (ADC DN ≈ 2588, 范围 2520~2919) → FMUM ID 0x007 → HW 类型字符串 `V6X007`
- **内部标识**: `V6X_SEEEX25D = HW_FMUM_ID(0x7)`

## 与原版 fmu-v6x 的差异

| 差异文件 | 变更内容 |
|----------|----------|
| `src/board_config.h` | 新增 `V6X_SEEEX25D = HW_FMUM_ID(0x7)` (ordinal 7, 电阻 33.4k+57.4k) |
| `src/spi.cpp` | 新增 V6X_SEEEX25D SPI 配置条目: SPI1=ICM20602, SPI2=ICM42688P, SPI3=BMI088(A+G) |
| `init/rc.board_sensors` | 新增 V6X004 条件分支: 启动 SEEEX25D 传感器集 (BMI088+ICM42688P+ICM20602+BMP388×2+QMC5883P) |
| `default.px4board` | 新增 `CONFIG_DRIVERS_MAGNETOMETER_QMC5883L=y` |
| `nuttx-config/nsh/defconfig` | CUSTOM_DIR 路径指向 `fmu-v6x_seeex25d` |
| `nuttx-config/bootloader/defconfig` | CUSTOM_DIR 路径指向 `fmu-v6x_seeex25d` |

USB 相关代码 (`usb.c`, USB VBUS 配置) 已恢复为与原版一致——SEEEX25D 硬件已解决 USB 问题。

## 关键文件

| File | Description |
|------|-------------|
| `default.px4board` | 构建配置 (新增 QMC5883L 驱动) |
| `src/board_config.h` | V6X_SEEEX25D 版本定义 (ordinal 4) |
| `src/spi.cpp` | SEEEX25D SPI 传感器配置 |
| `src/usb.c` | USB 初始化 (**与原版相同**) |
| `init/rc.board_sensors` | SEEEX25D 传感器启动脚本 |
| `nuttx-config/nsh/defconfig` | NuttX 配置 (路径已更新) |

## 子目录

| Directory | Purpose |
|-----------|---------|
| `src/` | 板级支持源码 |
| `nuttx-config/` | NuttX RTOS 配置 |
| `init/` | 板级初始化脚本 (rc.board_sensors 已适配) |
| `cmake/` | CMake 上传脚本 (**与原版相同**) |
| `extras/` | 预编译固件 (**与原版相同**) |

## For AI Agents

### Working In This Directory
- 构建目标: `make px4_fmu-v6x_seeex25d_default`
- HW 版本通过电阻分压检测，返回 ordinal 4 → `V6X004`
- 两个 BMP388 在同一 I2C2 总线上，必须使用不同地址 (0x77 和 0x76)
- QMC5883P 在 I2C4 上，使用 qmc5883l 驱动
- 所有 `-R` 旋转参数需根据实际 PCB 物理方向调整，当前为占位值 6

### HW 版本检测故障排查
如果 `ver hw` 返回 `FMUM ID: 0x000`:
1. 检查 33.4kΩ 电阻是否连接在 PG0 与 VDD 之间
2. 检查 57.4kΩ 电阻是否连接在 PG0 与 GND 之间
3. 确认 PH3 (VER) 和 PH4 (REV) 连通到 PG0
4. 确认 VDD_3V3_SENSORS4 电源域已使能
5. 上电后测 PG0 电压 (驱动LOW时): 应为 ~2.09V (3.3×57.4/(33.4+57.4))

### 合并原版更新策略
1. `usb.c`、`cmake/`、`extras/` 可直接从原版复制
2. `board_config.h`、`spi.cpp`、`rc.board_sensors` 需手动合并
3. `default.px4board` 需保留 QMC5883L 配置项

## Dependencies

### Internal
- `boards/px4/fmu-v6x/` — 原版 FMUv6x，派生基础
- `ROMFS/px4fmu_common/init.d/` — 共享系统初始化脚本
- `src/drivers/magnetometer/qmc5883l/` — QMC5883L 磁力计驱动
- `src/drivers/barometer/bmp388/` — BMP388 气压计驱动
- `src/drivers/imu/` — IMU 驱动 (ICM20602, ICM42688P, BMI088)

### External
- STM32H7 HAL (NuttX arch layer)
- arm-none-eabi toolchain (Cortex-M7)

<!-- MANUAL: -->
