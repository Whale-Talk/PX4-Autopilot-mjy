/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file LoggerFifoIntegrationTest.cpp
 *
 * Host-only integration test for the real uORB FIFO topic path into the PX4
 * logger and a flushed ULog file. This intentionally avoids IMU drivers and
 * controller modules; the production path under test is uORB subscription,
 * logger topic selection, logger thread lifecycle, and file writer shutdown.
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/workqueue.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

namespace
{

constexpr int32_t kRawFifoProfile = (1 << 8) | (1 << 9);
constexpr int32_t kAccelMask = 1 << 1;
constexpr int32_t kGyroMask = (1 << 0) | (1 << 2);
constexpr int kInstanceCount = 3;
constexpr int kPublishBatches = 300;
constexpr int kSamplesPerBatch = 32;
constexpr float kFifoScale = 1.0f;
constexpr float kFifoDtUs = 125.0f;

bool mkdir_one(const std::string &path)
{
	if (path.empty()) {
		return false;
	}

	if (::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST) {
		return true;
	}

	return false;
}

bool mkdir_p(const std::string &path)
{
	if (path.empty()) {
		return false;
	}

	std::string partial;

	for (char c : path) {
		partial.push_back(c);

		if (c == '/') {
			if (partial.size() > 1 && !mkdir_one(partial)) {
				return false;
			}
		}
	}

	return mkdir_one(partial);
}

std::string current_working_directory()
{
	std::vector<char> cwd(1024);

	while (::getcwd(cwd.data(), cwd.size()) == nullptr && errno == ERANGE) {
		cwd.resize(cwd.size() * 2);
	}

	return cwd.data();
}

bool is_regular_file(const std::string &path, off_t *size = nullptr)
{
	struct stat st {};

	if (::stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
		return false;
	}

	if (size) {
		*size = st.st_size;
	}

	return true;
}

void collect_ulg_files(const std::string &dir, std::vector<std::string> &files)
{
	DIR *dp = ::opendir(dir.c_str());

	if (!dp) {
		return;
	}

	while (dirent *entry = ::readdir(dp)) {
		if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
			continue;
		}

		const std::string path = dir + "/" + entry->d_name;
		struct stat st {};

		if (::stat(path.c_str(), &st) != 0) {
			continue;
		}

		if (S_ISDIR(st.st_mode)) {
			collect_ulg_files(path, files);

		} else if (S_ISREG(st.st_mode)) {
			const std::string name(entry->d_name);

			if (name.size() > 4 && name.compare(name.size() - 4, 4, ".ulg") == 0) {
				files.push_back(path);
			}
		}
	}

	::closedir(dp);
}

bool set_int_param(const char *name, int32_t value)
{
	const param_t handle = param_find(name);

	if (handle == PARAM_INVALID) {
		return false;
	}

	return param_set_no_notification(handle, &value) == PX4_OK;
}

int call_logger(std::initializer_list<const char *> args)
{
	std::vector<char *> argv;
	std::vector<std::vector<char>> storage;
	storage.reserve(args.size());
	argv.reserve(args.size() + 1);

	for (const char *arg : args) {
		storage.emplace_back(arg, arg + strlen(arg) + 1);
		argv.push_back(storage.back().data());
	}

	argv.push_back(nullptr);
	return logger_main(static_cast<int>(argv.size() - 1), argv.data());
}

sensor_accel_fifo_s make_accel_fifo(int instance, int batch)
{
	sensor_accel_fifo_s msg {};
	msg.timestamp_sample = hrt_absolute_time();
	msg.device_id = 0xACCE1000u + static_cast<uint32_t>(instance);
	msg.dt = kFifoDtUs;
	msg.scale = kFifoScale;
	msg.samples = kSamplesPerBatch;

	for (int n = 0; n < kSamplesPerBatch; n++) {
		msg.x[n] = static_cast<int16_t>(4000 + instance * 100 + batch);
		msg.y[n] = static_cast<int16_t>(5000 + instance * 100 + n);
		msg.z[n] = static_cast<int16_t>(6000 + instance * 100 + (batch % 97));
	}

	msg.timestamp = hrt_absolute_time();
	return msg;
}

sensor_gyro_fifo_s make_gyro_fifo(int instance, int batch)
{
	sensor_gyro_fifo_s msg {};
	msg.timestamp_sample = hrt_absolute_time();
	msg.device_id = 0x60001000u + static_cast<uint32_t>(instance);
	msg.dt = kFifoDtUs;
	msg.scale = kFifoScale;
	msg.samples = kSamplesPerBatch;

	for (int n = 0; n < kSamplesPerBatch; n++) {
		msg.x[n] = static_cast<int16_t>(1000 + instance * 100 + batch);
		msg.y[n] = static_cast<int16_t>(2000 + instance * 100 + n);
		msg.z[n] = static_cast<int16_t>(3000 + instance * 100 + (batch % 97));
	}

	msg.timestamp = hrt_absolute_time();
	return msg;
}

bool write_manifest(const std::string &path, const std::string &rootfs, const std::string &ulog, off_t ulog_size)
{
	FILE *fp = ::fopen(path.c_str(), "w");

	if (!fp) {
		return false;
	}

	const int written = ::fprintf(fp,
				      "{\n"
				      "  \"rootfs\": \"%s\",\n"
				      "  \"ulog\": \"%s\",\n"
				      "  \"ulog_size_bytes\": %lld,\n"
				      "  \"sdlog_profile\": %d,\n"
				      "  \"sdlog_acc_mask\": %d,\n"
				      "  \"sdlog_gyr_mask\": %d,\n"
				      "  \"selected_accel_instances\": [1],\n"
				      "  \"selected_gyro_instances\": [0, 2],\n"
				      "  \"published_accel_instances\": [0, 1, 2],\n"
				      "  \"published_gyro_instances\": [0, 1, 2],\n"
				      "  \"unselected_accel_instances\": [0, 2],\n"
				      "  \"unselected_gyro_instances\": [1],\n"
				      "  \"batches\": %d,\n"
				      "  \"warmup_batch_before_logger_start\": 0,\n"
				      "  \"samples_per_batch\": %d,\n"
				      "  \"dt_us\": %.1f,\n"
				      "  \"scale\": %.1f,\n"
				      "  \"expected_value_formula\": {\n"
				      "    \"gyro_x\": \"1000 + instance * 100 + batch\",\n"
				      "    \"gyro_y\": \"2000 + instance * 100 + sample_index\",\n"
				      "    \"gyro_z\": \"3000 + instance * 100 + (batch %% 97)\",\n"
				      "    \"accel_x\": \"4000 + instance * 100 + batch\",\n"
				      "    \"accel_y\": \"5000 + instance * 100 + sample_index\",\n"
				      "    \"accel_z\": \"6000 + instance * 100 + (batch %% 97)\"\n"
				      "  }\n"
				      "}\n",
				      rootfs.c_str(), ulog.c_str(), static_cast<long long>(ulog_size), kRawFifoProfile, kAccelMask,
				      kGyroMask, kPublishBatches, kSamplesPerBatch, (double)kFifoDtUs, (double)kFifoScale);

	const int close_ret = ::fclose(fp);
	return written > 0 && close_ret == 0;
}

class ScopedCwd
{
public:
	explicit ScopedCwd(const std::string &path) :
		_original(current_working_directory())
	{
		_changed = (::chdir(path.c_str()) == 0);
	}

	~ScopedCwd()
	{
		if (_changed) {
			const int ret = ::chdir(_original.c_str());
			(void)ret;
		}
	}

	bool changed() const { return _changed; }

private:
	std::string _original;
	bool _changed{false};
};

class WorkQueueRuntime
{
public:
	WorkQueueRuntime()
	{
		work_queues_init();
		hrt_work_queue_init();
		hrt_init();
		_wq_manager_started = (px4::WorkQueueManagerStart() == PX4_OK);
		px4_log_initialize();
	}

	~WorkQueueRuntime()
	{
		if (_wq_manager_started) {
			px4::WorkQueueManagerStop();
		}
	}

private:
	bool _wq_manager_started{false};
};

class LoggerStopGuard
{
public:
	int start()
	{
		const int ret = call_logger({"logger", "start", "-e", "-t", "-b", "128", "-m", "file"});
		_started = (ret == PX4_OK);
		return ret;
	}

	int stop()
	{
		if (!_started) {
			return PX4_OK;
		}

		const int ret = call_logger({"logger", "stop"});

		if (ret == PX4_OK) {
			_started = false;
		}

		return ret;
	}

	~LoggerStopGuard()
	{
		if (_started) {
			(void)stop();
		}
	}

private:
	bool _started{false};
};

} // namespace

TEST(LoggerFifoIntegrationTest, LogsSelectedFifoInstancesToUlog)
{
	const char *output_parent_env = ::getenv("PX4_VIBRATION_INTEGRATION_DIR");
	const std::string output_parent = output_parent_env ? output_parent_env : "/tmp/px4-vibration-integration";
	ASSERT_TRUE(mkdir_p(output_parent));

	std::string rootfs_template = output_parent + "/logger_fifo_rootfs_XXXXXX";
	std::vector<char> rootfs_buffer(rootfs_template.begin(), rootfs_template.end());
	rootfs_buffer.push_back('\0');
	char *rootfs_path = ::mkdtemp(rootfs_buffer.data());
	ASSERT_NE(rootfs_path, nullptr) << strerror(errno);

	const std::string rootfs(rootfs_path);
	ASSERT_TRUE(mkdir_p(rootfs + "/log"));
	ASSERT_TRUE(mkdir_p(rootfs + "/mission_log"));

	ScopedCwd cwd(rootfs);
	ASSERT_TRUE(cwd.changed()) << strerror(errno);

	WorkQueueRuntime runtime;
	param_control_autosave(false);
	ASSERT_TRUE(set_int_param("SDLOG_MODE", 2));
	ASSERT_TRUE(set_int_param("SDLOG_PROFILE", kRawFifoProfile));
	ASSERT_TRUE(set_int_param("SDLOG_ACC_MASK", kAccelMask));
	ASSERT_TRUE(set_int_param("SDLOG_GYR_MASK", kGyroMask));

	uORB::PublicationMulti<sensor_accel_fifo_s> accel_pubs[kInstanceCount] {
		ORB_ID(sensor_accel_fifo), ORB_ID(sensor_accel_fifo), ORB_ID(sensor_accel_fifo)
	};
	uORB::PublicationMulti<sensor_gyro_fifo_s> gyro_pubs[kInstanceCount] {
		ORB_ID(sensor_gyro_fifo), ORB_ID(sensor_gyro_fifo), ORB_ID(sensor_gyro_fifo)
	};

	for (int i = 0; i < kInstanceCount; i++) {
		ASSERT_TRUE(accel_pubs[i].advertise());
		ASSERT_EQ(accel_pubs[i].get_instance(), i);
		ASSERT_TRUE(gyro_pubs[i].advertise());
		ASSERT_EQ(gyro_pubs[i].get_instance(), i);
		ASSERT_TRUE(accel_pubs[i].publish(make_accel_fifo(i, 0)));
		ASSERT_TRUE(gyro_pubs[i].publish(make_gyro_fifo(i, 0)));
	}

	LoggerStopGuard logger;
	ASSERT_EQ(logger.start(), PX4_OK);
	px4_usleep(200000);

	for (int batch = 0; batch < kPublishBatches; batch++) {
		for (int instance = 0; instance < kInstanceCount; instance++) {
			ASSERT_TRUE(accel_pubs[instance].publish(make_accel_fifo(instance, batch)));
			ASSERT_TRUE(gyro_pubs[instance].publish(make_gyro_fifo(instance, batch)));
		}

		px4_usleep(8000);
	}

	px4_usleep(250000);
	ASSERT_EQ(logger.stop(), PX4_OK);

	std::vector<std::string> ulogs;
	collect_ulg_files(rootfs + "/log", ulogs);
	ASSERT_EQ(ulogs.size(), 1u);

	off_t ulog_size = 0;
	ASSERT_TRUE(is_regular_file(ulogs.front(), &ulog_size));
	ASSERT_GT(ulog_size, 0);

	const std::string manifest = output_parent + "/logger_fifo_expected.json";
	ASSERT_TRUE(write_manifest(manifest, rootfs, ulogs.front(), ulog_size));
}
