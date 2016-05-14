/****************************************************************************
 *
 *   Copyright (C) 2016 Bharath Ramaswamy. All rights reserved.
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

#include <stdint.h>
#include "SyncObj.hpp"
#include "I2CDevObj.hpp"



namespace DriverFramework
{

// From the schematic
#define LTC2946_DEVICE_PATH "/dev/iic-3"

// TODO: What should the correct value be?
#define DRV_DF_DEVTYPE_LTC2946 0x42


/**
 * The sensor independent data structure containing LTC2946 values.
 */
struct ltc2946_sensor_data {
	float voltage;
	float current;
	uint64_t read_counter; /*! the total number of sensor readings since the system was started */
};


class LTC2946 : public I2CDevObj
{
public:
	LTC2946(const char *device_path);

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

	static void printValues(struct ltc2946_sensor_data &data);
	static int getSensorData(DevHandle &h, struct ltc2946_sensor_data &out_data,
			bool is_new_data_required);

protected:
	virtual void _measure();
	virtual int _publish(struct ltc2946_sensor_data &data);

	struct ltc2946_sensor_data m_sensor_data;
	SyncObj m_synchronize;

private:
	// @returns 0 on success, -errno on failure
	int ltc2946_init();
};

}; // namespace DriverFramework
