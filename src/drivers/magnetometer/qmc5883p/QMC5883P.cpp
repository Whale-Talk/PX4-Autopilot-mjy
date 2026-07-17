/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * QMC5883P / QMC5883P I2C magnetometer driver
 *
 ****************************************************************************/

#include "QMC5883P.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

QMC5883P::QMC5883P(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

QMC5883P::~QMC5883P()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int QMC5883P::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool QMC5883P::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void QMC5883P::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int QMC5883P::probe()
{
	_retries = 1;

	for (int i = 0; i < 3; i++) {
		// Try to read register 0x01 (X_LSB) via combined I2C transaction
		const uint8_t cmd = static_cast<uint8_t>(Register::X_LSB);
		uint8_t buffer{};

		if (transfer(&cmd, 1, &buffer, 1) == PX4_OK) {
			return PX4_OK;
		}

		px4_usleep(2000);
	}

	return PX4_ERROR;
}

bool QMC5883P::Configure()
{
	// QMC5883P datasheet init sequence (QST-QMC5883P Rev A):
	// 1. Register 0x29 ← 0x06 (Define X/Y/Z axis sign)
	// 2. Register 0x0B ← 0x08 (Set/Reset mode, Field Range 8 Gauss)
	// 3. Register 0x0A ← 0xCD (Normal mode, ODR=200Hz)
	// Sensitivity: 3750 LSB/G for ±8G range

	uint8_t buf[2];

	// Register 0x29 ← 0x06
	buf[0] = 0x29;
	buf[1] = 0x06;

	if (transfer(buf, 2, nullptr, 0) != PX4_OK) {
		return false;
	}

	px4_usleep(1000);

	// Register 0x0B ← 0x08
	buf[0] = 0x0B;
	buf[1] = 0x08;

	if (transfer(buf, 2, nullptr, 0) != PX4_OK) {
		return false;
	}

	px4_usleep(1000);

	// Register 0x0A ← 0xCD
	buf[0] = 0x0A;
	buf[1] = 0xCD;

	if (transfer(buf, 2, nullptr, 0) != PX4_OK) {
		return false;
	}

	// Set scale: ±8G range, 3750 LSB/G → 1/3750 = 0.0002667 Gauss/LSB
	_px4_mag.set_scale(1.0f / 3750.0f);

	return true;
}

void QMC5883P::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// QMC5883P: soft reset not needed, just configure
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::CONFIGURE;
		perf_count(_reset_perf);
		ScheduleDelayed(10_ms);
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			_state = STATE::READ;
			ScheduleOnInterval(20_ms, 20_ms);

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_ERR("QMC5883P: configure failed, retrying reset");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("QMC5883P: configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			// Read X/Y/Z data starting from register 0x01 (6 bytes)
			const uint8_t cmd = static_cast<uint8_t>(Register::X_LSB);
			uint8_t data[6] {};

			if (transfer(&cmd, 1, data, 6) != PX4_OK) {
				perf_count(_bad_transfer_perf);
				_failure_count++;
				break;
			}

			// QMC5883P: X=01-02, Y=03-04, Z=05-06
			int16_t x = combine(data[1], data[0]); // 02:MSB, 01:LSB
			int16_t y = combine(data[3], data[2]); // 04:MSB, 03:LSB
			int16_t z = combine(data[5], data[4]); // 06:MSB, 05:LSB

			_px4_mag.update(now, x, y, z);

			if (_failure_count > 0) {
				_failure_count--;
			}
		}
		break;
	}
}

uint8_t QMC5883P::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void QMC5883P::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void QMC5883P::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
