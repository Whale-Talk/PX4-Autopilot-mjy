/****************************************************************************
 *
 * RK3506 Lyra SPI configuration (Linux POSIX)
 *
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(1, {
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, 0),    // SPI1 CS0 → BMI088 Accel
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, 1),    // SPI1 CS1 → BMI088 Gyro
	}),
	initSPIBus(2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, 0), // SPI2 CS0 → ICM42688P
	}),
	initSPIBus(3, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, 0),  // SPI3 CS0 → external ICM20602
	}),
};
