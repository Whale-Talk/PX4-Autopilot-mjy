/****************************************************************************
 *
 * RK3506 Lyra I2C configuration (Linux POSIX)
 *
 ****************************************************************************/

#include <px4_arch/i2c_hw_description.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(0),   // /dev/i2c-0 → BMP388 ×2 (0x77, 0x76)
	initI2CBusInternal(1),   // /dev/i2c-1 → QMC5883P (0x2C)
	initI2CBusExternal(2),   // /dev/i2c-2 → external I2C (external mag)
};
