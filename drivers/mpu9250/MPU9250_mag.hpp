/****************************************************************************
 *
 *   Copyright (C) 2016 James Y. Wilson. All rights reserved.
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

#pragma once

#include "MPU9250.hpp"

namespace DriverFramework
{
// Forward reference:
class MPU9250;

#define MPU9X50_INTERNAL_SAMPLE_RATE_HZ 1000

/**
 * Supported Sample rate for gyro.
 * If gyro sample rate is set to 8KHz, accelerometer sample rate is 1KHz.
 * If other sample rate is selected, the same sample rate is set for gyro and
 * accelerometer.
 */
enum gyro_sample_rate_e {
   MPU9x50_SAMPLE_RATE_100HZ = 0,
   MPU9x50_SAMPLE_RATE_200HZ,
   MPU9x50_SAMPLE_RATE_500HZ,
   MPU9x50_SAMPLE_RATE_1000HZ,
   MPU9x50_SAMPLE_RATE_8000HZ,
   NUM_MPU9X50_SAMPLE_RATE
};

#define MPU9X50_MAG_MAX_SAMPLE_RATE_HZ   100
/**
 * Sample rate for compass.
 * NOTE: only 100Hz compass sampling rate is supported in current driver.
 */
enum mag_sample_rate_e {
   MPU9x50_COMPASS_SAMPLE_RATE_100HZ = 0,
   NUM_MPU9X50_COMPASS_SAMPLE_RATE
};

class MPU9250_mag
{
public:
	MPU9250_mag(MPU9250 &imu, enum mag_sample_rate_e sample_rate) :
		_mag_initialized(false),
		_sample_rate_in_hz(_convert_sample_rate_enum_to_hz(sample_rate)),
		_imu(imu)
	{ };

	/// @brief
	/// Called to initialize the magnetometer connection via the
	/// internal I2C bus of the sensor.  The gyro and accelerometer
	/// must have been previously configured.
	/// @return 0 on success, -errno on failure
	int initialize(int gyro_sample_rate_in_hz);

	/// @return 0 on success, -errno on failure
	int get_sensitivity_adjustment(void);

	/// @return 0 on success, -errno on failure
	int detect(void);

	/// @return 0 on success, -errno on failure
	int write_imu_reg_verified(int reg, uint8_t val, uint8_t mask);

	/// @return 0 on success, -errno on failure
	int read_reg(uint8_t reg, uint8_t *val);

	/// @return 0 on success, -errno on failure
	int write_reg(uint8_t reg, uint8_t val);

	/// @return 0 on success, -errno on failure
	int process(struct mag_data &data);

protected:
	/// @brief
	/// Used internally to perform a complete mag initialization.  Called
	/// multiple times by the initialize() function if the first initialization
	/// attempt fails.
	int _initialize(int gyro_sample_rate_in_hz);

	/// @return The sample rate of the mag, converted from mag_sample_rate_e, -errno on failure
	int _convert_sample_rate_enum_to_hz(enum mag_sample_rate_e sample_rate);

private:
	float _mag_range_ga;
	float _mag_scaling;
	int _mag_sens_adj[3];
	float _mag_initialized;
	int _sample_rate_in_hz;

	MPU9250 &_imu;

	int _initialize();
};

};
