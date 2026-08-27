/****************************************************************************
 * Driver for DETA40 FDILink IMU/GNSS/INS module
 ****************************************************************************/

#include "DETA40.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <math.h>

using namespace time_literals;

DETA40::DETA40(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	if (port) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	}
}

DETA40::~DETA40()
{
	if (_fd >= 0) { ::close(_fd); _fd = -1; }
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

bool DETA40::openPort()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed: %d", _port, errno);
		return false;
	}

	struct termios t {};
	tcgetattr(_fd, &t);

	/* DETA40 default: 921600 8N1 */
	cfsetspeed(&t, B921600);
	t.c_cflag |= (CLOCAL | CREAD);
	t.c_cflag &= ~CSIZE;
	t.c_cflag |= CS8;
	t.c_cflag &= ~PARENB;
	t.c_cflag &= ~CSTOPB;
	t.c_cflag &= ~CRTSCTS;
	t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	t.c_iflag &= ~(IXON | IXOFF | IXANY);
	t.c_oflag &= ~OPOST;

	tcsetattr(_fd, TCSANOW, &t);
	tcflush(_fd, TCIOFLUSH);

	PX4_INFO("DETA40 opened on %s @ 921600", _port);
	return true;
}

void DETA40::closePort()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int DETA40::readFrame()
{
	if (_fd < 0) { return 0; }

	uint8_t byte;
	ssize_t n = ::read(_fd, &byte, 1);

	if (n <= 0) { return 0; }

	switch (_state) {
	case STATE_IDLE:
		if (byte == FDILINK_FRAME_START) {
			_bufpos = 0;
			_rxbuf[_bufpos++] = byte;
			_state = STATE_HEADER;
		}
		break;

	case STATE_HEADER:
		_rxbuf[_bufpos++] = byte;
		if (_bufpos >= FDILINK_HEADER_SIZE) {
			/* validate CRC8 over bytes 0-3 (start, id, len, seq) */
			uint8_t crc = FDILinkCRC8(_rxbuf, 4);
			if (crc != _rxbuf[4]) {
				_state = STATE_IDLE;
				_error_count++;
				return 0;
			}
			_pkt_id = _rxbuf[1];
			_pkt_len = _rxbuf[2];
			_pkt_seq = _rxbuf[3];

			if (_pkt_len == 0) {
				/* no payload, check CRC16 wraps */
				_state = STATE_COMPLETE;
			} else {
				_state = STATE_PAYLOAD;
				_bufpos = 0; /* reset to fill payload */
			}
		}
		break;

	case STATE_PAYLOAD:
		_rxbuf[_bufpos++] = byte;
		if (_bufpos >= _pkt_len) {
			_state = STATE_CRC16;
			_bufpos = 0;
		}
		break;

	case STATE_CRC16:
		_rxbuf[_pkt_len + _bufpos] = byte;
		_bufpos++;
		if (_bufpos >= FDILINK_CRC16_SIZE) {
			uint16_t crc16_expect = (_rxbuf[_pkt_len + 0] << 8) | _rxbuf[_pkt_len + 1];
			uint16_t crc16_actual = FDILinkCRC16(_rxbuf, _pkt_len);
			if (crc16_expect == crc16_actual) {
				_state = STATE_COMPLETE;
			} else {
				_state = STATE_IDLE;
				_error_count++;
			}
		}
		break;

	case STATE_COMPLETE:
	case STATE_ERROR:
		_state = STATE_IDLE;
		break;
	}

	if (_state == STATE_COMPLETE) {
		_state = STATE_IDLE;
		return _pkt_id;
	}

	return 0;
}

void DETA40::parseIMU(const uint8_t *payload)
{
	const FDILinkIMU *imu = reinterpret_cast<const FDILinkIMU *>(payload);

	sensor_accel_s accel{};
	accel.timestamp = hrt_absolute_time();
	accel.device_id = 0xFD40; /* DETA40 IMU */
	accel.x = imu->accel_x;
	accel.y = imu->accel_y;
	accel.z = imu->accel_z;
	accel.temperature = imu->imu_temp;
	accel.samples = 1;
	_sensor_accel_pub.publish(accel);

	sensor_gyro_s gyro{};
	gyro.timestamp = accel.timestamp;
	gyro.device_id = 0xFD40;
	gyro.x = imu->gyro_x;
	gyro.y = imu->gyro_y;
	gyro.z = imu->gyro_z;
	gyro.temperature = imu->imu_temp;
	gyro.samples = 1;
	_sensor_gyro_pub.publish(gyro);

	sensor_mag_s mag{};
	mag.timestamp = accel.timestamp;
	mag.device_id = 0xFD40;
	mag.x = imu->mag_x / 1000.0f; /* mG -> Gs */
	mag.y = imu->mag_y / 1000.0f;
	mag.z = imu->mag_z / 1000.0f;
	mag.temperature = imu->imu_temp;
	_sensor_mag_pub.publish(mag);

	sensor_baro_s baro{};
	baro.timestamp = accel.timestamp;
	baro.device_id = 0xFD40;
	baro.pressure = imu->pressure / 100.0f; /* Pa -> hPa */
	baro.temperature = imu->pressure_temp;
	_sensor_baro_pub.publish(baro);

	_imu_count++;
	_last_imu_us = accel.timestamp;
}

void DETA40::parseSysState(const uint8_t *payload)
{
	const FDILinkSysState *ss = reinterpret_cast<const FDILinkSysState *>(payload);

	if (ss->system_status & 0x01) { return; } /* system failure */
	if (!(ss->filter_status & 0x01)) { return; } /* filter not initialized */

	vehicle_attitude_s att{};
	att.timestamp = hrt_absolute_time();
	/* FDILink AHRS: q1,q2,q3,q4 → PX4: q[0]=w, q[1]=x, q[2]=y, q[3]=z */
	/* Using euler from MSG_SYS_STATE: roll, pitch, heading (rad) */
	float cp = cosf(ss->pitch * 0.5f);
	float sp = sinf(ss->pitch * 0.5f);
	float cr = cosf(ss->roll * 0.5f);
	float sr = sinf(ss->roll * 0.5f);
	float ch = cosf(ss->heading * 0.5f);
	float sh = sinf(ss->heading * 0.5f);

	att.q[0] = cr * cp * ch + sr * sp * sh; /* w */
	att.q[1] = sr * cp * ch - cr * sp * sh; /* x */
	att.q[2] = cr * sp * ch + sr * cp * sh; /* y */
	att.q[3] = cr * cp * sh - sr * sp * ch; /* z */
	_attitude_pub.publish(att);

	sensor_gps_s gps{};
	gps.timestamp = att.timestamp;
	gps.latitude_deg = ss->latitude * 180.0 / M_PI;
	gps.longitude_deg = ss->longitude * 180.0 / M_PI;
	gps.altitude_msl_m = ss->height;
	gps.altitude_ellipsoid_m = ss->height;
	gps.vel_n_m_s = ss->vel_north;
	gps.vel_e_m_s = ss->vel_east;
	gps.vel_d_m_s = ss->vel_down;
	gps.eph = ss->lat_std_dev;
	gps.epv = ss->height_std_dev;
	gps.fix_type = ((ss->filter_status >> 4) & 0xF) > 1 ? 3 : 1;
	_gps_pub.publish(gps);

	_gps_count++;
}

void DETA40::Run()
{
	if (_fd < 0) {
		if (!openPort()) {
			ScheduleDelayed(1_s);
			return;
		}
	}

	perf_begin(_sample_perf);

	/* Read all available frames */
	for (int i = 0; i < 64; i++) {
		int pkt_id = readFrame();
		if (pkt_id == 0) { break; }

		_frame_count++;
		_last_data_us = hrt_absolute_time();

		switch (pkt_id) {
		case MSG_IMU:
			parseIMU(_rxbuf);
			break;
		case MSG_SYS_STATE:
			parseSysState(_rxbuf);
			break;
		case MSG_HEARTBEAT:
			break;
		default:
			break;
		}
	}

	perf_end(_sample_perf);

	/* Check for data timeout */
	hrt_abstime now = hrt_absolute_time();
	if (_last_data_us != 0 && (now - _last_data_us) > 2_s) {
		PX4_ERR("data timeout");
		closePort();
		perf_count(_comms_errors);
	}

	ScheduleDelayed(2_ms);
}

int DETA40::print_status()
{
	PX4_INFO("DETA40 on %s @ 921600", _port);
	PX4_INFO("frames: %" PRIu32 " imu: %" PRIu32 " gps: %" PRIu32 " err: %" PRIu32,
		 _frame_count, _imu_count, _gps_count, _error_count);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	return 0;
}

int DETA40::task_spawn(int argc, char *argv[])
{
	const char *port = nullptr;

	if (argc > 1) {
		port = argv[1];
	} else {
		PX4_ERR("usage: deta40 start <device>");
		return PX4_ERROR;
	}

	DETA40 *instance = new DETA40(port);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);

	if (!instance->openPort()) {
		PX4_ERR("port open failed");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	instance->ScheduleDelayed(10_ms);
	return PX4_OK;
}

int DETA40::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DETA40::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION("DETA40 FDILink IMU/GNSS Driver");
	PRINT_MODULE_USAGE_NAME("deta40", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int deta40_main(int argc, char *argv[])
{
	return DETA40::main(argc, argv);
}
