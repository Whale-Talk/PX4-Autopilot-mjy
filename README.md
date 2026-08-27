# PX4 Autopilot — SEEEX25D H1 飞控定制版

本仓库是 [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) 的定制分叉（fork），基于 **PX4 v1.16.1**（上游基线提交 `94cb201279`，2025-11-10）派生，服务于 **SEEEX25D H1 飞控**（定制版 FMUv6x，STM32H743 + NuttX）。

> **状态：已打板，实际飞行验证通过** ✅
>
> H1 飞控已完成 PCB 打板、固件烧录与真实飞行验证（传感器组合、SBUS 遥控链路、PX4IO 协处理器均已在硬件上验证）。

## 关于 H1 飞控（fmu-v6x_seeex25d）

H1 飞控基于标准 FMUv6x（Pixhawk 6X 架构）定制，硬件型号 SEEEX25D，早期开发代号 mjy，现已统一命名。板级目标：`px4_fmu-v6x_seeex25d_default`。

### 定制传感器组合

| 传感器 | 总线 | CS/地址 | 说明 |
|--------|------|---------|------|
| BMI088（加速度计）| SPI3 | PI4 | 与陀螺仪分体 |
| BMI088（陀螺仪）| SPI3 | PI8 | |
| ICM42688P | SPI2 | PH5 | |
| ICM20602 | SPI1 | PI9 | 主 IMU |
| BMP388 #1（内部）| I2C4 | 0x77 | |
| BMP388 #2（外部）| I2C2 | 0x76 | |
| QMC5883P | I2C4 | 0x0D | 外部磁力计 |

### 硬件版本检测

- 版本电阻分压：33.4kΩ（上拉）+ 57.4kΩ（下拉）→ ordinal 7 → `V6X007`（HW FMUM ID 0x7）

### 架构

- **FMU**：STM32H743（Cortex-M7），运行 NuttX
- **IO 协处理器**：PX4IO v2（STM32F100），负责 RC 输入与 MAIN PWM 输出，经 USART6（`/dev/ttyS5`，1.5Mbps）与 FMU 通信
- **RC 输入**：SBUS 经 IO 板硬件反相器进入 IO 板 USART3（PB11）；DSM/Spektrum 走 USART1（PA10）

## 相对上游 PX4 的主要修改

| 修改 | 位置 | 说明 |
|------|------|------|
| **H1 飞控板级** | `boards/px4/fmu-v6x_seeex25d/` | 定制板级：HW 版本检测（V6X007）、独立 SPI/I2C 传感器配置、启动脚本 |
| **QMC5883P 磁力计驱动** | `src/drivers/magnetometer/qmc5883p/` | 新增驱动（H1 外部磁力计） |
| **DETA40 INS 驱动** | `src/drivers/ins/deta40/` | 外接 DETA40 IMU/GNSS 模组（FDILink 协议，UART 921600bps）|
| **磁力计标定点数** | `src/modules/commander/mag_calibration.cpp` | 标定采样点 240 → 600（提高球拟合精度）|
| **bootloader 更新** | `boards/px4/fmu-v6x/extras/` | 同步升级至 PX4BL v1.16.1（上游仓库仍为 v1.15.0）|
| 诊断工具 | `_h1_diag/` | H1 飞控 MAVLink 串口诊断/参数工具（零第三方依赖）|

## 构建

```bash
# H1 飞控固件（本仓库主目标）
make px4_fmu-v6x_seeex25d_default

# 产物
build/px4_fmu-v6x_seeex25d_default/px4_fmu-v6x_seeex25d_default.px4
```

构建依赖 arm-none-eabi 工具链（GCC 14.2.1）、CMake、Ninja。板级定制说明见 `boards/px4/fmu-v6x_seeex25d/AGENTS.md`。

## 分支结构

| 分支 | 内容 |
|------|------|
| `seeex25d_H1` | **主开发分支**：H1 飞控（SEEEX25D）全部定制 |
| `upstream` | 跟踪上游 PX4（基线 `94cb201279`，v1.16.1）|
| `lyra_dev` | RK3506 Lyra 板（Linux 目标，独立维护）|

## 上游信息

- 上游项目：[PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)（BSD 3-clause，见 [LICENSE](LICENSE)）
- 官方文档：https://docs.px4.io
- 本定制基于 PX4 v1.16.1（提交 `94cb201279`），飞行栈与中间件沿用上游，未改动核心控制逻辑
