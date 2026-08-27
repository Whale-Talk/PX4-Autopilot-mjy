/****************************************************************************
 *
 * RK3506 Lyra board configuration (Linux POSIX target)
 *
 ****************************************************************************/

#pragma once

#define BOARD_OVERRIDE_UUID "RKLYRA0000000000"
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_RPI
#define HW_INFO_INIT_PREFIX "RK3506"

#define BOARD_HAS_CONTROL_STATUS_LEDS 1

// I2C
#define CONFIG_I2C 1
#define PX4_NUMBER_I2C_BUSES    4

// SPI
#define CONFIG_SPI 1
#define PX4_NUMBER_SPI_BUSES    3

// ADC
#define ADC_BATTERY_VOLTAGE_CHANNEL  0
#define ADC_BATTERY_CURRENT_CHANNEL -1

#include <system_config.h>
#include <px4_platform_common/board_common.h>
