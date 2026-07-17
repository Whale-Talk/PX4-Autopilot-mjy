/****************************************************************************
 *
 *   QMC5883P registers (adapted from QMC5883P for P-variant)
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace QST_QMC5883P
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_SPEED = 400 * 1000;
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b0101100; // QMC5883P: 0x2C (7-bit) = 0x58 (8-bit)

static constexpr uint8_t Chip_ID = 0xFF; // Probe check (may not exist on QMC5883P)

// QMC5883P register map (different from QMC5883P!)
enum class Register : uint8_t {
	X_LSB      = 0x01, // X-axis LSB
	X_MSB      = 0x02, // X-axis MSB
	Y_LSB      = 0x03, // Y-axis LSB
	Y_MSB      = 0x04, // Y-axis MSB
	Z_LSB      = 0x05, // Z-axis LSB
	Z_MSB      = 0x06, // Z-axis MSB

	CNTL1      = 0x06, // Control 1 (write 0x29)
	CNTL2      = 0x08, // Control 2 (write 0x0B)
	CNTL3      = 0x0A, // Control 3 (write 0xCD)

	CHIP_ID    = 0x00, // May not have a CHIP_ID register
};

} // namespace QST_QMC5883P
