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

#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "MPU9250.hpp"
#include "MPU9250_mag.hpp"

#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_USER_CTRL_FIFO_EN		0x40
#define BITS_USER_CTRL_FIFO_RST		0x04
#define BITS_USER_CTRL_I2C_MST_EN  	0x20

#define BITS_CONFIG_FIFO_MODE_OVERWRITE	0x00
#define BITS_CONFIG_FIFO_MODE_STOP	0x40

#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
// This is FCHOICE_B which is the inverse of FCHOICE
#define BITS_BW_3600HZ			0x02
// The FCHOICE bits are the same for all Bandwidths below 3600 Hz.
#define BITS_BW_LT3600HZ		0x00

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_FIFO_ENABLE_TEMP_OUT	0x80
#define BITS_FIFO_ENABLE_GYRO_XOUT	0x40
#define BITS_FIFO_ENABLE_GYRO_YOUT	0x20
#define BITS_FIFO_ENABLE_GYRO_ZOUT	0x10
#define BITS_FIFO_ENABLE_ACCEL		0x08
#define BITS_FIFO_ENABLE_SLV2		0x04
#define BITS_FIFO_ENABLE_SLV1		0x02
#define BITS_FIFO_ENABLE_SLV0		0x01

#define BITS_ACCEL_CONFIG_16G		0x18

// This is ACCEL_FCHOICE_B which is the inverse of ACCEL_FCHOICE
#define BITS_ACCEL_CONFIG2_BW_1130HZ	0x08

#define BITS_I2C_SLV1_EN     0x80
#define BITS_I2C_SLV1_DIS    0x00

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10

#define BITS_INT_STATUS_FIFO_OVERFLOW	0x10

#define BITS_I2C_MST_CLK_400_KHZ	0x0D

#define MPU9250_ONE_G			9.80665f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

using namespace DriverFramework;

int MPU9250::mpu9250_init()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	// TODO-JYW: TESTING-TESTING
	_mag_enabled = true;
	// TODO-JYW: TESTING-TESTING

	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.accel_m_s2_x = 0.0f;
	m_sensor_data.accel_m_s2_y = 0.0f;
	m_sensor_data.accel_m_s2_z = 0.0f;
	m_sensor_data.gyro_rad_s_x = 0.0f;
	m_sensor_data.gyro_rad_s_y = 0.0f;
	m_sensor_data.gyro_rad_s_z = 0.0f;
	m_sensor_data.mag_ga_x = 0.0f;
	m_sensor_data.mag_ga_y = 0.0f;
	m_sensor_data.mag_ga_z = 0.0f;
	m_sensor_data.temp_c = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_sensor_data.fifo_overflow_counter = 0;
	m_sensor_data.fifo_corruption_counter = 0;
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	m_sensor_data.time_offset_us = 0;

	m_synchronize.unlock();

	// TODO-JYW: TESTING-TESTING
//	int result = _writeReg(MPUREG_PWR_MGMT_1,
//			       BIT_H_RESET |
//			       MPU_CLK_SEL_AUTO);
	int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}
	usleep(100000);
	DF_LOG_INFO("Reset MPU9250");

	result = _writeReg(MPUREG_PWR_MGMT_1, 0);
	if (result != 0) {
		DF_LOG_ERR("wakeup sensor failed");
	}

// TODO-JYW: TESTING-TESTING
//	result = _writeReg(MPUREG_PWR_MGMT_2, 0);
//
//	if (result != 0) {
//		DF_LOG_ERR("clock selection failed");
//	}

	usleep(1000);

	result = _writeReg(MPUREG_USER_CTRL,
			BITS_USER_CTRL_FIFO_RST | BITS_USER_CTRL_FIFO_EN);

	if (result != 0) {
		DF_LOG_ERR("User ctrl config failed");
	}

	usleep(1000);

	if (_mag_enabled) {
		result = _writeReg(MPUREG_FIFO_EN,
				BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
						| BITS_FIFO_ENABLE_GYRO_YOUT
						| BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL);
//				   BITS_FIFO_ENABLE_SLV0);  // SLV0 is configured for bulk transfer of mag data over I2C
	} else {
		result = _writeReg(MPUREG_FIFO_EN,
				BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
						| BITS_FIFO_ENABLE_GYRO_YOUT
						| BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL);
	}

	if (result != 0) {
		DF_LOG_ERR("FIFO enable failed");
	}

	usleep(1000);

	/*
	 * A samplerate_divider of 0 should give 1000Hz:
	 *
	 * sample_rate = internal_sample_rate / (1+samplerate_divider)
	 *
	 * This is only used when FCHOICE is 0b11, FCHOICE_B (inverted) 0x00,
	 * therefore commented out.
	 */
	//uint8_t samplerate_divider = 0;
	//result = _writeReg(MPUREG_FIFO_EN, samplerate_divider);
	//if (result != 0) {
	//	DF_LOG_ERR("sample rate config failed");
	//}
	//usleep(1000);
	result = _writeReg(MPUREG_CONFIG,
			BITS_DLPF_CFG_250HZ | BITS_CONFIG_FIFO_MODE_OVERWRITE);

	if (result != 0) {
		DF_LOG_ERR("config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS | BITS_BW_LT3600HZ);

	if (result != 0) {
		DF_LOG_ERR("Gyro scale config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_ACCEL_CONFIG, BITS_ACCEL_CONFIG_16G);

	if (result != 0) {
		DF_LOG_ERR("Accel scale config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_ACCEL_CONFIG2, BITS_ACCEL_CONFIG2_BW_1130HZ);

	if (result != 0) {
		DF_LOG_ERR("Accel scale config2 failed");
	}

	usleep(1000);

	// TODO-JYW: TESTING-TESTING
//	result = _writeReg(MPUREG_ACCEL_CONFIG2,
//			BIT_FIFO_SIZE_4096);
//
//	if (result != 0) {
//		DF_LOG_ERR("FIFO configuration failed");
//	}

//	usleep(1000);

	// Initialize the magnetometer inside the IMU, if enabled by the caller.
	if (_mag_enabled) {
		if (_mag == nullptr) {
			DF_LOG_INFO("Instantiating MPU9250_mag class.");
			if ((_mag = new MPU9250_mag(*this,
					MPU9x50_COMPASS_SAMPLE_RATE_100HZ)) != nullptr) {
				// Initialize the magnetometer, providing the gyro sample rate for
				// internal calculations.
				result = _mag->initialize(8000); // 8000 = sample rate in Hz of the gyro
				if (result != 0) {
					DF_LOG_ERR("Magnetometer initialization failed");
				}
				// Reset the FIFO since the mag configuration add more data to the FIFO.
				reset_fifo();
			} else {
				DF_LOG_ERR("Allocation of magnetometer object failed.");
			}
		}
	}

	return 0;
}

int MPU9250::start()
{

	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	int result = SPIDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	/* Set the bus frequency for register get/set. */
	result = _setBusFrequency(SPI_FREQUENCY_1MHZ);

	if (result != 0) {
		DF_LOG_ERR("failed setting SPI bus frequency: %d", result);
	}

	/* Try to talk to the sensor. */
	uint8_t sensor_id;
	result = _readReg(MPUREG_WHOAMI, sensor_id);

	if (result != 0) {
		DF_LOG_ERR("Unable to communicate with the MPU9250 sensor");
		goto exit;
	}

	if (sensor_id != MPU_WHOAMI_9250) {
		DF_LOG_ERR("MPU9250 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
				sensor_id, MPU_WHOAMI_9250);
		result = -1;
		goto exit;
	}

	result = mpu9250_init();

	if (result != 0) {
		DF_LOG_ERR(
				"error: IMU sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

	exit:

	if (result != 0) {
		devClose();
	}

	return result;
}

int MPU9250::stop()
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

int MPU9250::get_fifo_count()
{
	int16_t num_bytes = 0x0;

	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);
	int ret = _bulkRead(MPUREG_FIFO_COUNTH, (uint8_t *) &num_bytes,
			sizeof(num_bytes));

	if (ret == 0) {

		/* TODO: add ifdef for endianness */
		num_bytes = swap16(num_bytes);

		return num_bytes;

	} else {
		DF_LOG_ERR("FIFO count read failed");
		return ret;
	}
}

void MPU9250::reset_fifo()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);
	int result = _writeReg(MPUREG_USER_CTRL,
			BITS_USER_CTRL_FIFO_RST | BITS_USER_CTRL_FIFO_EN
					| BITS_USER_CTRL_I2C_MST_EN);

	if (result != 0) {
		DF_LOG_ERR("FIFO reset failed");
	}
}

void MPU9250::_measure()
{
	// TODO-JYW: TESTING-TESTING
//	usleep(1000);

	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);
	uint8_t int_status = 0;
	int result = _readReg(MPUREG_INT_STATUS, int_status);

	if (result != 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	if (int_status & BITS_INT_STATUS_FIFO_OVERFLOW) {
		reset_fifo();

		m_synchronize.lock();
		if ((++m_sensor_data.fifo_overflow_counter % 1000) == 0) {
			DF_LOG_ERR("FIFO overflow");
		}
		m_synchronize.unlock();

		return;
	}

	int size_of_fifo_packet;
	if (_mag_enabled) {
		size_of_fifo_packet = sizeof(fifo_packet_with_mag);
	} else {
		size_of_fifo_packet = sizeof(fifo_packet);
	}

	// Get FIFO byte count to read and floor it to the report size.
	int bytes_to_read = get_fifo_count() / size_of_fifo_packet
			* size_of_fifo_packet;

	if (bytes_to_read < 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	// TODO-JYW: TESTING-TESTING: Is this statement correct?  The FIFO should be
	// configurable.
	//
	// The FIFO buffer on the MPU is 512 bytes according to the datasheet, so let's use
	// 36*size_of_fifo_packet.
	const unsigned buf_len = 36 * size_of_fifo_packet;
	uint8_t fifo_read_buf[buf_len];

	if (bytes_to_read <= 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	const unsigned read_len = MIN((unsigned)bytes_to_read, buf_len);
	memset(fifo_read_buf, 0x0, buf_len);

	// According to the protocol specs, all sensor and interrupt registers may be read at 20 MHz.
	// It is unclear what rate the FIFO register can be read at.
	// If the FIFO buffer was read at 20 MHz, two effects were seen:
	// - The buffer is off-by-one. So the report "starts" at &fifo_read_buf[i+1].
	// - Also, the FIFO buffer seemed to prone to random corruption (or shifting), unless
	//   all other sensors ran very smooth. (E.g. Changing the bus speed of the HMC5883 driver from
	//   400 kHz to 100 kHz could cause corruption because this driver wouldn't run as regularly.
	//
	// Luckily 10 MHz seems to work fine.

	_setBusFrequency(SPI_FREQUENCY_10MHZ);
//	DF_LOG_INFO("bulk read at: 0x%X, len: %d, size of packet: %d", fifo_read_buf, read_len,
//			size_of_fifo_packet);
	result = _bulkRead(MPUREG_FIFO_R_W, fifo_read_buf, read_len);

	if (result != 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	for (unsigned i = 0; i < read_len / size_of_fifo_packet; ++i) {

		fifo_packet *report = (fifo_packet *) (&fifo_read_buf[i
				* size_of_fifo_packet]);

		/* TODO: add ifdef for endianness */
		// TODO-JYW: Let the sensor do the byte swapping.
		report->accel_x = swap16(report->accel_x);
		report->accel_y = swap16(report->accel_y);
		report->accel_z = swap16(report->accel_z);
		report->temp = swap16(report->temp);
		report->gyro_x = swap16(report->gyro_x);
		report->gyro_y = swap16(report->gyro_y);
		report->gyro_z = swap16(report->gyro_z);

		// Check if the full accel range of the accel has been used. If this occurs, it is
		// either a spike due to a crash/landing or a sign that the vibrations levels
		// measured are excessive.
		if (report->accel_x == INT16_MIN || report->accel_x == INT16_MAX
				|| report->accel_y == INT16_MIN || report->accel_y == INT16_MAX
				|| report->accel_z == INT16_MIN || report->accel_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.accel_range_hit_counter;
			m_synchronize.unlock();
		}

		// Also check the full gyro range, however, this is very unlikely to happen.
		if (report->gyro_x == INT16_MIN || report->gyro_x == INT16_MAX
				|| report->gyro_y == INT16_MIN || report->gyro_y == INT16_MAX
				|| report->gyro_z == INT16_MIN || report->gyro_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.gyro_range_hit_counter;
			m_synchronize.unlock();
		}

		const float temp_c = float(report->temp) / 361.0f + 35.0f;

		// Use the temperature field to try to detect if we (ever) fall out of sync with
		// the FIFO buffer. If the temperature changes insane amounts, reset the FIFO logic
		// and return early.
		if (!_temp_initialized) {
			// Assume that the temperature should be in a sane range of -40 to 85 deg C which is
			// the specified temperature range, at least to initialize.
			if (temp_c > -40.0f && temp_c < 85.0f) {

				// Initialize the temperature logic.
				_last_temp_c = temp_c;
				DF_LOG_INFO("IMU temperature initialized to: %f", temp_c);
				_temp_initialized = true;
			}
		} else {
			// TODO-JYW: TESTING-TESTING
			// Once initialized, check for a temperature change of more than 2 degrees which
			// points to a FIFO corruption.
			if (fabs(temp_c - _last_temp_c) > 2.0f) {
				DF_LOG_ERR(
						"FIFO corrupt, temp difference: %f, last temp: %f, current temp: %f",
						fabs(temp_c - _last_temp_c), (double)_last_temp_c, (double)temp_c);
				reset_fifo();
				_temp_initialized = false;
				m_synchronize.lock();
				++m_sensor_data.fifo_corruption_counter;
				m_synchronize.unlock();
				return;
			}

			// A FIFO sample is created at 8kHz, which means the interval between samples is 125 us.
			// The last read FIFO sample at i+1 has an offset of 0.
			m_sensor_data.time_offset_us = -((read_len / size_of_fifo_packet)
					- (i + 1)) * 125;

			_last_temp_c = temp_c;
		}

		// TODO-JYW: LEFT-OFF:
		// The FIFO is repeatedly getting corrupted.  Verify that the data appears valid
		// when it is not corrupted.  Also, change the MPU configuration to match that which
		// we are using in the legacy driver, particularly the setting which prevents
		// writing into the FIFO if it is already full.

		m_synchronize.lock();
//		DF_LOG_INFO("read valid data");

		m_sensor_data.accel_m_s2_x = float(report->accel_x)
				* (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_y = float(report->accel_y)
				* (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_z = float(report->accel_z)
				* (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.temp_c = temp_c;
		m_sensor_data.gyro_rad_s_x = float(report->gyro_x) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_y = float(report->gyro_y) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_z = float(report->gyro_z) * GYRO_RAW_TO_RAD_S;

		// TODO-JYW: Replace the number with a constant.
		// The only error of significance is the mag overflow which can corrupt the sample.
		// A data not ready error can be ignored to allow the previous sample from the sensor
		// to be used.
//		if (_mag_enabled && mag_error == 0) {
//			fifo_packet_with_mag *report_with_mag = (fifo_packet_with_mag *)report;
//			m_sensor_data.mag_ga_x = float(report_with_mag->mag_x);
//			m_sensor_data.mag_ga_y = float(report_with_mag->mag_y);
//			m_sensor_data.mag_ga_z = float(report_with_mag->mag_z);
//		}

		++m_sensor_data.read_counter;

		// TODO-JYW: TESTING-TESTING
		// Generate debug output every second, assuming that a sample is generated every
		// 125 usecs.
		if (++m_sensor_data.read_counter % (1000000 / 125) == 0) {
			DF_LOG_INFO("IMU: accel: [%f, %f, %f]",
					(double)m_sensor_data.accel_m_s2_x, (double)m_sensor_data.accel_m_s2_y, (double)m_sensor_data.accel_m_s2_z);
			DF_LOG_INFO("     gyro:  [%f, %f, %f]",
					(double)m_sensor_data.gyro_rad_s_x, (double)m_sensor_data.gyro_rad_s_y, (double)m_sensor_data.gyro_rad_s_z);
			DF_LOG_INFO("    temp:  %f C", (double)m_sensor_data.temp_c);

//			if (_mag_enabled && mag_error == 0) {
//				DF_LOG_INFO("     mag:  [%f, %f, %f] ga",
//						(double)m_sensor_data.mag_ga_x,
//						(double)m_sensor_data.mag_ga_y,
//						(double)m_sensor_data.mag_ga_z);
//			}
		}
		// TODO-JYW: TESTING-TESTING

		if (_mag_enabled) {
			struct mag_data data = { 0 };
//			int ret = _bulkRead(MPUREG_EXT_SENS_DATA_00, (uint8_t *)&data, 1);
			int ret = _bulkRead(MPUREG_EXT_SENS_DATA_00, (uint8_t *)&data, sizeof(mag_data));
//			_mag->read_reg(0x02, (uint8_t *)&data);
//			readReg(MPUREG_EXT_SENS_DATA_00, (uint8_t &) data.mag_st1);
			if (ret != 0) {
				DF_LOG_ERR("error reading mag data from external sensor data registers");
			}

			// TODO-JYW: TESTING-TESTING: checking for data ready bit
//			if (data.mag_st1 & 0x01) {
//				writeReg(MPUREG_I2C_SLV1_CTRL, BITS_I2C_SLV1_EN | 0x07);
////				usleep(500);
//
//				for (int i = 1; i < 8; i++) {
//					readReg(MPUREG_EXT_SENS_DATA_00 + i,
//							*(((uint8_t *) &data) + i));
//				}
//				writeReg(MPUREG_I2C_SLV1_CTRL, BITS_I2C_SLV1_DIS);

//				_bulkRead(MPUREG_EXT_SENS_DATA_00 + 1, (uint8_t *)&data.mag_x, sizeof(data) - sizeof(data.mag_st1));
//				writeReg(MPUREG_I2C_SLV1_CTRL, BITS_I2C_SLV1_DIS | 0x07);
				int mag_error = _mag->process(data);
				if (mag_error == 0) {
					if ((m_sensor_data.read_counter % 10000) == 0) {
						DF_LOG_INFO("     mag:  [%d, %d, %d] ga",
								data.mag_x, data.mag_y, data.mag_z);
					}
				}
//			}
		}

		_publish(m_sensor_data);

		m_synchronize.signal();
		m_synchronize.unlock();
	}
}

int MPU9250::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}
