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
 * @file LoggedTopicsTest.cpp
 *
 * Functional tests for the logger topic selection (LoggedTopics).
 *
 * Covers the raw FIFO instance masks SDLOG_ACC_MASK / SDLOG_GYR_MASK, the
 * logger_status health subscriptions added for FIFO captures, and the
 * logger_topics.txt replacement semantics. The tests inspect the real
 * subscription list produced by LoggedTopics::initialize_logged_topics()
 * instead of re-implementing the bit arithmetic.
 */

#include <gtest/gtest.h>

#include <px4_platform_common/defines.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <parameters/param.h>

#include "logged_topics.h"

using namespace px4::logger;

namespace
{

/** Same path expression as the production code in logged_topics.cpp. */
constexpr const char *kTopicsFile = PX4_STORAGEDIR "/etc/logging/logger_topics.txt";

/** Documented @max of SDLOG_ACC_MASK / SDLOG_GYR_MASK. */
constexpr int32_t kMaskAllInstanceBits = 1023;

struct Subscription {
	std::string topic;
	uint8_t instance;
	uint16_t interval_ms;
};

SDLogProfileMask profile_mask(std::initializer_list<SDLogProfileMask> masks)
{
	int32_t value = 0;

	for (const auto mask : masks) {
		value |= static_cast<int32_t>(mask);
	}

	return static_cast<SDLogProfileMask>(value);
}

const char *topic_name(ORB_ID id)
{
	const orb_metadata *const *topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (static_cast<ORB_ID>(topics[i]->o_id) == id) {
			return topics[i]->o_name;
		}
	}

	return "<unknown>";
}

std::vector<Subscription> collect(const LoggedTopics &logged_topics)
{
	std::vector<Subscription> result;
	const LoggedTopics::RequestedSubscriptionArray &subs = logged_topics.subscriptions();

	for (int i = 0; i < subs.count; i++) {
		result.push_back({topic_name(subs.sub[i].id), subs.sub[i].instance, subs.sub[i].interval_ms});
	}

	return result;
}

int count_of(const std::vector<Subscription> &subs, const char *topic)
{
	int count = 0;

	for (const auto &sub : subs) {
		if (sub.topic == topic) {
			count++;
		}
	}

	return count;
}

std::vector<uint8_t> instances_of(const std::vector<Subscription> &subs, const char *topic)
{
	std::vector<uint8_t> instances;

	for (const auto &sub : subs) {
		if (sub.topic == topic) {
			instances.push_back(sub.instance);
		}
	}

	return instances;
}

bool has_subscription(const std::vector<Subscription> &subs, const char *topic, uint8_t instance, uint16_t interval_ms)
{
	for (const auto &sub : subs) {
		if (sub.topic == topic && sub.instance == instance && sub.interval_ms == interval_ms) {
			return true;
		}
	}

	return false;
}

} // namespace

class LoggedTopicsTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		save_topics_file();
		remove_topics_file();
	}

	void TearDown() override
	{
		remove_topics_file();

		if (file_existed_) {
			write_raw_topics_file(backup_);
		}

		// remove the directories this test created (fails silently if not empty)
		for (auto it = created_dirs_.rbegin(); it != created_dirs_.rend(); ++it) {
			::rmdir(it->c_str());
		}

		created_dirs_.clear();
	}

	// --- topics file fixture -----------------------------------------------------

	bool write_topics_file(const std::string &content)
	{
		return ensure_parent_dir() && write_raw_topics_file(content);
	}

	void remove_topics_file()
	{
		::remove(kTopicsFile);
	}

	void save_topics_file()
	{
		file_existed_ = false;
		backup_.clear();

		FILE *f = ::fopen(kTopicsFile, "r");

		if (!f) {
			return;
		}

		file_existed_ = true;

		char buffer[512];

		while (::fgets(buffer, sizeof(buffer), f)) {
			backup_ += buffer;
		}

		::fclose(f);
	}

	bool write_raw_topics_file(const std::string &content)
	{
		FILE *f = ::fopen(kTopicsFile, "w");

		if (!f) {
			return false;
		}

		const bool ok = content.empty() || (::fwrite(content.data(), 1, content.size(), f) == content.size());
		::fclose(f);
		return ok;
	}

	bool ensure_parent_dir()
	{
		const std::string dir(PX4_STORAGEDIR "/etc/logging");
		std::string partial;

		for (size_t start = 0; start <= dir.size();) {
			const size_t slash = dir.find('/', start);
			const std::string component = dir.substr(start, slash == std::string::npos
						      ? std::string::npos : slash - start);

			if (!component.empty()) {
				if (partial.empty()) {
					partial = (dir[0] == '/') ? "/" + component : component;

				} else {
					partial += "/" + component;
				}

				struct stat st {};

				if (::stat(partial.c_str(), &st) != 0) {
					if (::mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST) {
						return false;
					}

					created_dirs_.push_back(partial);
				}
			}

			if (slash == std::string::npos) {
				break;
			}

			start = slash + 1;
		}

		return true;
	}

	bool file_existed_{false};
	std::string backup_;
	std::vector<std::string> created_dirs_;
};

// --- SDLOG_PROFILE FIFO bits and instance masks ---------------------------------

TEST_F(LoggedTopicsTest, FifoProfileOffAddsNoFifoOrHealthSubscriptions)
{
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(SDLogProfileMask::DEFAULT, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(count_of(subs, "sensor_accel_fifo"), 0);
	EXPECT_EQ(count_of(subs, "sensor_gyro_fifo"), 0);
	EXPECT_EQ(count_of(subs, "logger_status"), 0);
	// the default profile itself is still added
	EXPECT_GT(count_of(subs, "sensor_combined"), 0);
}

TEST_F(LoggedTopicsTest, FifoProfileDefaultMaskLogsInstanceZero)
{
	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO,
					   SDLogProfileMask::RAW_IMU_ACCEL_FIFO
					  });
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(instances_of(subs, "sensor_gyro_fifo"), (std::vector<uint8_t> {0}));
	EXPECT_EQ(instances_of(subs, "sensor_accel_fifo"), (std::vector<uint8_t> {0}));
	// full publication rate
	EXPECT_TRUE(has_subscription(subs, "sensor_gyro_fifo", 0, 0));
	EXPECT_TRUE(has_subscription(subs, "sensor_accel_fifo", 0, 0));
}

TEST_F(LoggedTopicsTest, FifoProfileDifferentMasksForAccelAndGyro)
{
	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO,
					   SDLogProfileMask::RAW_IMU_ACCEL_FIFO
					  });
	LoggedTopics logged_topics;
	// accel instance 1, gyro instance 2
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1 << 1, 1 << 2));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(instances_of(subs, "sensor_accel_fifo"), (std::vector<uint8_t> {1}));
	EXPECT_EQ(instances_of(subs, "sensor_gyro_fifo"), (std::vector<uint8_t> {2}));
}

TEST_F(LoggedTopicsTest, SparseMaskIncludesZeroAndHighestSupportedInstance)
{
	const uint8_t highest = ORB_MULTI_MAX_INSTANCES - 1;
	const int32_t sparse = (1 << 0) | (1 << highest);
	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, sparse, sparse));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(instances_of(subs, "sensor_accel_fifo"), (std::vector<uint8_t> {0, highest}));
	EXPECT_EQ(instances_of(subs, "sensor_gyro_fifo"), (std::vector<uint8_t> {0, highest}));
}

TEST_F(LoggedTopicsTest, ZeroMaskSelectsNoInstanceButKeepsHealthSubscription)
{
	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 0, 0));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(count_of(subs, "sensor_accel_fifo"), 0);
	EXPECT_EQ(count_of(subs, "sensor_gyro_fifo"), 0);
	// the profile bits are not undone by a zero mask
	EXPECT_EQ(count_of(subs, "logger_status"), 2);
}

TEST_F(LoggedTopicsTest, InvalidMaskValuesFallBackToInstanceZero)
{
	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	LoggedTopics logged_topics;
	// negative and above the documented maximum both fall back to instance 0 only
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, -1, kMaskAllInstanceBits + 1));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(instances_of(subs, "sensor_accel_fifo"), (std::vector<uint8_t> {0}));
	EXPECT_EQ(instances_of(subs, "sensor_gyro_fifo"), (std::vector<uint8_t> {0}));
}

TEST_F(LoggedTopicsTest, FullDocumentedMaskSelectsAllSupportedInstances)
{
	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, kMaskAllInstanceBits, kMaskAllInstanceBits));

	const auto accel = instances_of(collect(logged_topics), "sensor_accel_fifo");
	ASSERT_EQ(accel.size(), static_cast<size_t>(ORB_MULTI_MAX_INSTANCES));

	for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
		EXPECT_EQ(accel[instance], instance);
	}
}

#if (ORB_MULTI_MAX_INSTANCES < 10)
TEST_F(LoggedTopicsTest, UnsupportedInstanceBitsAreDropped)
{
	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	// supported instances 0 and 1 plus one bit above the build limit
	const int32_t mixed = (1 << 0) | (1 << 1) | (1 << ORB_MULTI_MAX_INSTANCES);
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, mixed, mixed));

	const auto subs = collect(logged_topics);
	EXPECT_EQ(instances_of(subs, "sensor_accel_fifo"), (std::vector<uint8_t> {0, 1}));
	EXPECT_EQ(instances_of(subs, "sensor_gyro_fifo"), (std::vector<uint8_t> {0, 1}));

	// only unsupported bits: clamped to nothing, never "all instances"
	LoggedTopics clamped;
	ASSERT_TRUE(clamped.initialize_logged_topics(profile, 1 << ORB_MULTI_MAX_INSTANCES, 1 << ORB_MULTI_MAX_INSTANCES));
	const auto clamped_subs = collect(clamped);
	EXPECT_EQ(count_of(clamped_subs, "sensor_accel_fifo"), 0);
	EXPECT_EQ(count_of(clamped_subs, "sensor_gyro_fifo"), 0);
}
#else
TEST_F(LoggedTopicsTest, UnsupportedInstanceBitsAreDropped)
{
	// With ORB_MULTI_MAX_INSTANCES == 10 every documented mask bit is supported, so the
	// clamping path cannot be reached with a valid parameter value in this build.
	GTEST_SKIP() << "ORB_MULTI_MAX_INSTANCES=" << ORB_MULTI_MAX_INSTANCES
		     << ": all documented mask bits are supported";
}
#endif

TEST_F(LoggedTopicsTest, HealthSubscriptionsFollowFifoProfileBits)
{
	const auto fifo_only = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(fifo_only, 1, 1));

	const auto subs = collect(logged_topics);

	// both logger_status instances, about 1 Hz, regardless of the selected FIFO instances
	EXPECT_TRUE(has_subscription(subs, "logger_status", 0, 1000));
	EXPECT_TRUE(has_subscription(subs, "logger_status", 1, 1000));
	EXPECT_EQ(count_of(subs, "logger_status"), 2);
	// the DEFAULT profile is not enabled in this test
	EXPECT_EQ(count_of(subs, "sensor_combined"), 0);
}

// --- logger_topics.txt priority -------------------------------------------------

TEST_F(LoggedTopicsTest, MissingTopicsFileUsesProfile)
{
	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO,
					   SDLogProfileMask::RAW_IMU_ACCEL_FIFO
					  });
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_TRUE(has_subscription(subs, "sensor_gyro_fifo", 0, 0));
	EXPECT_TRUE(has_subscription(subs, "sensor_accel_fifo", 0, 0));
	EXPECT_GT(count_of(subs, "sensor_combined"), 0);
	EXPECT_EQ(count_of(subs, "logger_status"), 2);
}

TEST_F(LoggedTopicsTest, EmptyOrCommentOnlyTopicsFileUsesProfile)
{
	ASSERT_TRUE(write_topics_file("\n\n# only a comment\n   \n"));

	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_TRUE(has_subscription(subs, "sensor_gyro_fifo", 0, 0));
	EXPECT_TRUE(has_subscription(subs, "sensor_accel_fifo", 0, 0));
	EXPECT_EQ(count_of(subs, "logger_status"), 2);
}

TEST_F(LoggedTopicsTest, ValidTopicsFileReplacesProfileAndSkipsMasks)
{
	ASSERT_TRUE(write_topics_file("sensor_gyro\n"));

	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO,
					   SDLogProfileMask::RAW_IMU_ACCEL_FIFO
					  });
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	// the file replaces the profile: all instances of the listed topic at full rate
	EXPECT_EQ(count_of(subs, "sensor_gyro"), ORB_MULTI_MAX_INSTANCES);
	// profile topics (default set, FIFO masks, logger_status) are not added on top
	EXPECT_EQ(count_of(subs, "sensor_combined"), 0);
	EXPECT_EQ(count_of(subs, "sensor_gyro_fifo"), 0);
	EXPECT_EQ(count_of(subs, "sensor_accel_fifo"), 0);
	EXPECT_EQ(count_of(subs, "logger_status"), 0);
}

TEST_F(LoggedTopicsTest, UnknownLinesStillReplaceTheProfile)
{
	// Original parser behaviour: any parsed line counts, even for an unknown topic.
	ASSERT_TRUE(write_topics_file("sensor_gyro 0 0\nnot_a_topic\n"));

	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_TRUE(has_subscription(subs, "sensor_gyro", 0, 0));
	// replacement is active even though one line is unknown
	EXPECT_EQ(count_of(subs, "sensor_combined"), 0);
	EXPECT_EQ(count_of(subs, "sensor_gyro_fifo"), 0);
}

TEST_F(LoggedTopicsTest, AllUnknownLinesLeaveNoSubscriptions)
{
	ASSERT_TRUE(write_topics_file("not_a_topic_one\nnot_a_topic_two\n"));

	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::RAW_IMU_GYRO_FIFO});
	LoggedTopics logged_topics;

	// ntopics > 0 selects the replacement path, but no valid subscription is created
	EXPECT_FALSE(logged_topics.initialize_logged_topics(profile, 1, 1));
	EXPECT_EQ(logged_topics.subscriptions().count, 0);
}

TEST_F(LoggedTopicsTest, DuplicateLinesDeduplicateAndUpdateInterval)
{
	ASSERT_TRUE(write_topics_file("sensor_gyro 0 0\nsensor_gyro 500 0\nsensor_gyro 100 1\n"));

	const auto profile = profile_mask({SDLogProfileMask::RAW_IMU_GYRO_FIFO});
	LoggedTopics logged_topics;
	ASSERT_TRUE(logged_topics.initialize_logged_topics(profile, 1, 1));

	const auto subs = collect(logged_topics);

	EXPECT_EQ(count_of(subs, "sensor_gyro"), 2);
	// repeated topic/instance pair is not duplicated, the interval is updated
	EXPECT_TRUE(has_subscription(subs, "sensor_gyro", 0, 500));
	EXPECT_TRUE(has_subscription(subs, "sensor_gyro", 1, 100));
}

TEST_F(LoggedTopicsTest, SubscriptionCountStaysWithinCapacity)
{
	const auto profile = profile_mask({SDLogProfileMask::DEFAULT, SDLogProfileMask::ESTIMATOR_REPLAY,
					   SDLogProfileMask::THERMAL_CALIBRATION, SDLogProfileMask::SYSTEM_IDENTIFICATION,
					   SDLogProfileMask::HIGH_RATE, SDLogProfileMask::DEBUG_TOPICS,
					   SDLogProfileMask::SENSOR_COMPARISON, SDLogProfileMask::VISION_AND_AVOIDANCE,
					   SDLogProfileMask::RAW_IMU_GYRO_FIFO, SDLogProfileMask::RAW_IMU_ACCEL_FIFO,
					   SDLogProfileMask::MAVLINK_TUNNEL
					  });
	LoggedTopics logged_topics;
	logged_topics.initialize_logged_topics(profile, kMaskAllInstanceBits, kMaskAllInstanceBits);

	// use the value, not a reference to the class constant (keeps the test free of an out-of-line
	// definition of the production class member)
	const int capacity = static_cast<int>(LoggedTopics::MAX_TOPICS_NUM);
	const int count = logged_topics.subscriptions().count;
	const auto subs = collect(logged_topics);

	// The existing capacity limit is preserved: never more than MAX_TOPICS_NUM subscriptions.
	EXPECT_LE(count, capacity);

	// Subscriptions are appended in selection order, so any truncation caused by a full
	// subscription list can only drop a suffix of every topic's instance list.
	const auto instances_are_prefix = [](const std::vector<uint8_t> &instances) {
		for (size_t i = 0; i < instances.size(); i++) {
			if (instances[i] != i) {
				return false;
			}
		}

		return true;
	};

	const int gyro_instances = count_of(subs, "sensor_gyro_fifo");
	const int accel_instances = count_of(subs, "sensor_accel_fifo");
	const int health_instances = count_of(subs, "logger_status");

	if (count < capacity) {
		// The whole selection fit, so add_topic() cannot have dropped anything: every selected
		// FIFO instance and both logger_status instances have to be in the real subscription set.
		EXPECT_EQ(gyro_instances, ORB_MULTI_MAX_INSTANCES);
		EXPECT_EQ(accel_instances, ORB_MULTI_MAX_INSTANCES);
		EXPECT_TRUE(has_subscription(subs, "sensor_gyro_fifo", ORB_MULTI_MAX_INSTANCES - 1, 0));
		EXPECT_TRUE(has_subscription(subs, "sensor_accel_fifo", ORB_MULTI_MAX_INSTANCES - 1, 0));
		EXPECT_EQ(health_instances, 2);
		EXPECT_TRUE(has_subscription(subs, "logger_status", 0, 1000));
		EXPECT_TRUE(has_subscription(subs, "logger_status", 1, 1000));

	} else {
		// The configuration fills the subscription list completely. Do not assume the full
		// selection fits: assert the observable consequence (truncated suffix, unchanged full
		// rate for what did fit) instead. The corresponding "Too many subscriptions" PX4_WARN is
		// emitted by the unchanged add_topic(); this test only observes the resulting set.
		EXPECT_EQ(count, capacity);
		EXPECT_TRUE(instances_are_prefix(instances_of(subs, "sensor_gyro_fifo")));
		EXPECT_TRUE(instances_are_prefix(instances_of(subs, "sensor_accel_fifo")));
		EXPECT_TRUE(instances_are_prefix(instances_of(subs, "logger_status")));

		for (const auto &sub : subs) {
			if (sub.topic == "sensor_gyro_fifo" || sub.topic == "sensor_accel_fifo") {
				EXPECT_EQ(sub.interval_ms, 0);
			}

			if (sub.topic == "logger_status") {
				EXPECT_EQ(sub.interval_ms, 1000);
			}
		}
	}
}

// --- parameter metadata ---------------------------------------------------------

TEST_F(LoggedTopicsTest, MaskParameterDefaultsAreOne)
{
	for (const char *name : {"SDLOG_ACC_MASK", "SDLOG_GYR_MASK"}) {
		const param_t param = param_find(name);
		ASSERT_NE(param, PARAM_INVALID) << name << " is not registered";

		int32_t default_value = 0;
		ASSERT_EQ(param_get_default_value(param, &default_value), PX4_OK) << name;
		// default 1 keeps the previous behaviour: instance 0 only
		EXPECT_EQ(default_value, 1) << name;
	}
}
