/****************************************************************************
 *
 *   Driver for QMC5883P / QMC5883P connected via I2C.
 *
 ****************************************************************************/

#pragma once

#include "QST_QMC5883P_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace QST_QMC5883P;

class QMC5883P : public device::I2C, public I2CSPIDriver<QMC5883P>
{
public:
	QMC5883P(const I2CSPIDriverConfig &config);
	~QMC5883P() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	bool Reset();

	bool Configure();

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	PX4Magnetometer _px4_mag;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	hrt_abstime _reset_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};
};
