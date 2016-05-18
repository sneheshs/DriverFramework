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

#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>


#include <string.h>
#include "DriverFramework.hpp"
#include "LTC2946.hpp"
#ifdef __QURT
#include "dev_fs_lib_i2c.h"
#endif

// APM has two LTC chips, one connected to battery, one connected to 5V supply. Also two different
// current sensing resistors are used to measure the corresponding currents (for APM), and one for
// ESC.
enum LtcModeType
{
	LTC_MODE_ESC_VBATT = 0,
	LTC_MODE_APM_VBATT = 1,
	LTC_MODE_APM_5V = 2,
	LTC_NUM_MODES
};

#define R_SENSE_ESC_VBATT	0.001
#define R_SENSE_APM_VBATT	0.0005
#define R_SENSE_APM_5V		0.005

// I2C address to measure battery supply voltage and current
#define LTC2946_I2C_ADDRESS_VBATT 0b1101010
// I2C address to measure 5V supply voltage and current
#define LTC2946_I2C_ADDRESS_5V 0b1101011

// *** Set this to a LtcModeType value ***
#define LTC_MODE	LTC_MODE_APM_5V

// Set the I2C address and R_sense depending on the mode
#if LTC_MODE == LTC_MODE_ESC_VBATT
	#define LTC2946_I2C_ADDRESS LTC2946_I2C_ADDRESS_VBATT
	#define R_SENSE R_SENSE_APM_VBATT
#elif LTC_MODE == LTC_MODE_APM_VBATT
	#define LTC2946_I2C_ADDRESS LTC2946_I2C_ADDRESS_VBATT
	#define R_SENSE R_SENSE_APM_VBATT
#elif LTC_MODE == LTC_MODE_APM_5V
	#define LTC2946_I2C_ADDRESS LTC2946_I2C_ADDRESS_5V
	#define R_SENSE R_SENSE_APM_5V
#endif

// TODO: Looked at while loop delay to come up with this
#define LTC2946_MEASURE_INTERVAL_US 2000

#define LTC2946_BUS_FREQUENCY_IN_KHZ 400
#define LTC2946_TRANSFER_TIMEOUT_IN_USECS 9000

#define LTC2946_BUF_SIZE    32


using namespace DriverFramework;

LTC2946::LTC2946(const char *device_path) :
	I2CDevObj("LtcSensor", device_path, LTC2946_DEVICE_PATH, LTC2946_MEASURE_INTERVAL_US)
{
	m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_LTC2946;
	m_id.dev_id_s.address = LTC2946_I2C_ADDRESS;
}

int LTC2946::i2c_read_reg(uint8_t address, uint8_t* out_buffer, int length)
{
	/* Read the data from the sensor. */
	int result = _readReg(address, out_buffer, length);

	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return -1;
	}

	return 0;
}


int LTC2946::i2c_write_reg(uint8_t address, uint8_t *in_buffer, int length)
{
	if (length + 1 > LTC2946_BUF_SIZE)
	{
		DF_LOG_ERR("ltc2946: Caller's buffer exceeds size of local buffer");
		return -1;
	}

	// Verify that the length of the caller's buffer does not exceed the local stack
	// buffer with one additional byte for the register ID.
	int result = _writeReg(address, in_buffer, length);

	if (result != 0) {
		DF_LOG_ERR("error: sensor write failed");
		return -EIO;
	}

	return 0;
}


int LTC2946::configure()
{
  uint8_t CTRLA = 0b00001000; //Gnd ref, offset evey conv, volt=sense+, alternate volt and curr
  uint8_t CTRLB = 0b00000100;

  if (i2c_write_reg(0x00, &CTRLA, 1)) return -1;
  if (i2c_write_reg(0x01, &CTRLB, 1)) return -2;

  return 0;
}


int LTC2946::ltc2946_init()
{
	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.voltage = 0.0f;
	m_sensor_data.current = 0.0f;

	m_synchronize.unlock();

	if (configure()!=0)
	{
		DF_LOG_ERR("ltc2946: configure failed!");
		return -1;
	}

	DF_LOG_INFO("ltc2946: initialization successful!");

	usleep(1000);
	return 0;
}


int LTC2946::start()
{
	int result = devOpen(0);

	/* Open the device path specified in the class initialization */
	if (result < 0) {
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		//goto exit;
		return -1;
	}

	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the LTC2946 sensor. */
	result = _setSlaveConfig(LTC2946_I2C_ADDRESS, LTC2946_BUS_FREQUENCY_IN_KHZ,
				 LTC2946_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	/* Initialize the sensor for active and continuous operation. */
	result = ltc2946_init();

	if (result != 0) {
		DF_LOG_ERR("error: sensor initialization failed, sensor read thread not started");
		goto exit;
	}


	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

exit:

	if (result != 0) {
		devClose();
	}

	return result;
}


int LTC2946::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	result = devClose();

	if (result != 0) {
		DF_LOG_ERR("device close failed");
		return result;
	}

	return 0;
}


void LTC2946::_measure(void)
{
	/* Read the data from the LTC2946 sensor. */

	static uint32_t cntr = 0;
	float voltage = 0;
	float current = 0;

	cntr++;

	// Read raw voltage measurement from 0x1E register (2 bytes)
	uint8_t vraw[2];
	if (LTC2946::i2c_read_reg(0x1E,vraw,2)) return;
	// Read raw current measurement from 0x14 register (2 bytes)
	uint8_t iraw[2];
	if (LTC2946::i2c_read_reg(0x14,iraw,2)) return;

	uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];  //MSB first
	volt16        >>= 4;                                     //data is 12 bit and left-aligned
	float v_now      = volt16/4095.0 * 102.4;                //102.4V is maximum voltage on this input

	uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];  //MSB first
	curr16        >>= 4;                                     //data is 12 bit and left-aligned

	float r_sense    = R_SENSE;
	float i_now      = curr16/4095.0 * 0.1024 / r_sense;     //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

	if (cntr == 1) voltage = v_now;                          //if first time, initialize voltage to value now

	voltage = voltage * 0.99 + v_now * 0.01;                 //filter voltage
	current = current * 0.99 + i_now * 0.01;                 //filter current

	if (cntr % 50 == 0)
	{
		DF_LOG_DEBUG("voltage = %f, current %f",voltage,current);
	}

	m_sensor_data.voltage = voltage;
	m_sensor_data.current = current;
	m_sensor_data.read_counter = cntr;

	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();

}


int LTC2946::_publish(struct ltc2946_sensor_data &data)
{
	// TBD
	return -1;
}


void LTC2946::printValues(struct ltc2946_sensor_data &data)
{
	DF_LOG_INFO("Voltage = %f V, current %f A", data.voltage, data.current);
}


int LTC2946::getSensorData(DevHandle &h, struct ltc2946_sensor_data &out_data,
		bool is_new_data_required)
{
	LTC2946 *me = DevMgr::getDevObjByHandle<LTC2946>(h);
	int ret = -1;

	if (me != nullptr) {
		me->m_synchronize.lock();

		if (is_new_data_required) {
			me->m_synchronize.waitOnSignal(0);
		}

		out_data = me->m_sensor_data;
		me->m_synchronize.unlock();
		ret = 0;
	}

	return ret;
}
