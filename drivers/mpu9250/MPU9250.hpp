/****************************************************************************
 *
 *   Copyright (C) 2016 Julian Oes. All rights reserved.
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

#include "ImuSensor.hpp"
#include "MPU9250_mag.hpp"

namespace DriverFramework
{

// TODO: use define from some include for this
#define	M_PI		3.14159265358979323846	/* pi */

// update frequency 1000 Hz
#define MPU9250_MEASURE_INTERVAL_US 1000

// -2000 to 2000 degrees/s, 16 bit signed register, deg to rad conversion
#define GYRO_RAW_TO_RAD_S 	 (2000.0f / 32768.0f * M_PI / 180.0f)

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_MPU9250 0x41

#define MPU_WHOAMI_9250			0x71

#pragma pack(push, 1)
struct fifo_packet {
	int16_t		accel_x;
	int16_t		accel_y;
	int16_t		accel_z;
	int16_t		temp;
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
	//uint8_t		ext_data[24];
};
struct fifo_packet_with_mag {
	int16_t		accel_x;
	int16_t		accel_y;
	int16_t		accel_z;
	int16_t		temp;
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
	char        mag_st1; // 14 mag ST1 (1B)
	int16_t     mag_x;   // 15-16 (2B)
	int16_t     mag_y;   // 17-18 (2B)
	int16_t     mag_z;   // 19-20 (2B)
	char        mag_st2; // 21 mag ST2 (1B)
};
#pragma pack(pop)

class MPU9250 : public ImuSensor
{
public:
	MPU9250(const char *device_path, bool mag_enabled = false) :
		ImuSensor(device_path, MPU9250_MEASURE_INTERVAL_US, mag_enabled), // true = mag is enabled
		_last_temp_c(0.0f),
		_temp_initialized(false),
		_mag_enabled(mag_enabled)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MPU9250;
		// TODO: does the WHOAMI make sense as an address?
		m_id.dev_id_s.address = MPU_WHOAMI_9250;
	}

	// @return 0 on success, -errno on failure
	int writeReg(int reg, uint8_t val)
	{
		return _writeReg(reg, val);
	}

	int readReg(uint8_t address, uint8_t &val)
	{
		return _readReg(address, val);
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct imu_sensor_data &data);

private:
	// @returns 0 on success, -errno on failure
	int mpu9250_init();

	// @return the number of FIFO bytes to collect
	int get_fifo_count();

	void reset_fifo();

	float _last_temp_c;
	bool _temp_initialized;
	bool _mag_enabled;
	MPU9250_mag *_mag;
};

}; // namespace DriverFramework
