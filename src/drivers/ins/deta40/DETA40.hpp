/****************************************************************************
 * Driver for DETA40 FDILink IMU/GNSS/INS module (UART binary protocol)
 ****************************************************************************/

#pragma once

#include "fdilink_protocol.h"

#include <stdint.h>
#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>

class DETA40 : public ModuleBase<DETA40>, public px4::ScheduledWorkItem
{
public:
	DETA40(const char *port);
	~DETA40() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	bool openPort();
	void closePort();
	int readFrame();
	void parseIMU(const uint8_t *payload);
	void parseSysState(const uint8_t *payload);

	char _port[32] {};
	int _fd{-1};

	uint8_t _rxbuf[FDILINK_MAX_PAYLOAD + 16];
	uint8_t _bufpos{0};
	uint8_t _pkt_id{0}, _pkt_len{0}, _pkt_seq{0};
	FDILinkParseState _state{STATE_IDLE};

	hrt_abstime _last_data_us{0};
	hrt_abstime _last_imu_us{0};
	uint32_t _frame_count{0};
	uint32_t _error_count{0};
	uint32_t _imu_count{0};
	uint32_t _gps_count{0};

	uORB::PublicationMulti<sensor_accel_s>  _sensor_accel_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_gyro_s>   _sensor_gyro_pub{ORB_ID(sensor_gyro)};
	uORB::PublicationMulti<sensor_mag_s>    _sensor_mag_pub{ORB_ID(sensor_mag)};
	uORB::PublicationMulti<sensor_baro_s>   _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<vehicle_attitude_s>     _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::PublicationMulti<sensor_gps_s>           _gps_pub{ORB_ID(sensor_gps)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
