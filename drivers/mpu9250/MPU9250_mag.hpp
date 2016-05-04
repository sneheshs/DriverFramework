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

// MPU9250 Magnetometer Register Addresses: Defines only the register addresses
// used in MPU9250 driver.
#define MPU9250_MAG_REG_WIA		0x00
#define MPU9250_MAG_REG_CNTL1	0x0a
#define MPU9250_MAG_REG_ASAX	0x10
#define MPU9250_MAG_REG_ASAY    0x11
#define MPU9250_MAG_REG_ASAZ    0x12

// Full Scale Range of the magnetometer chip AK89xx in MPU9250
#define MPU9250_AK89xx_FSR		4915

// Magnetometer device ID
#define MPU9250_AKM_DEV_ID		0x48


class MPU9250_mag
{
public:
	MPU9250_mag(MPU9250 *imu) :
		_mag_initialized(false),
		_imu(imu)
	{ };

	// @returns 0 on success, -errno on failure
	int initialize();

private:
	float _mag_range_ga;
	float _mag_scaling;
	float _mag_initialized;
	int _mag_sens_adj[3];
	MPU9250 *_imu;

	int _initialize();
};

int _initialize()
{
	_mag_initialized = false;
	// Retry up to 5 times to ensure successful initialization of the
	// sensor's internal I2C bus.
	int init_max_tries = 5;
	int ret = 0;
	int i;
	for (i = 0; i < mag_init_max_tries; i++) {
		ret = _initialize_config(config);
		if (ret == 0) {
			break;
		}
		FARF(HIGH, "mag initialization failed %d tries", i + 1);
		usleep(10000);
	}

	if (ret == 0) {
		FARF(MEDIUM, "mag initialization succ after %d retries", i);
		driver_context.mag_enabled = 1;
	} else {
		FARF(ALWAYS, "failed to initialize mag!");
	}
}

int _initialize_config(void)
{

}

};
