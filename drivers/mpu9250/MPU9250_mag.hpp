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

/**
 * MPU9250 Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_REG_ADDR {
   MPU9250_REG_SMPLRT_DIV     = 25,
   MPU9250_REG_CONFIG         = 26,
   MPU9250_REG_GYRO_CONFIG    = 27,
   MPU9250_REG_ACCEL_CONFIG   = 28,
   MPU9250_REG_ACCEL_CONFIG2  = 29,
   MPU9250_REG_FIFO_ENABLE    = 35,
   MPU9250_REG_I2C_MST_CTRL   = 36,
   MPU9250_REG_I2C_SLV0_ADDR  = 37,
   MPU9250_REG_I2C_SLV0_REG   = 38,
   MPU9250_REG_I2C_SLV0_CTRL  = 39,
   MPU9250_REG_I2C_SLV1_ADDR  = 40,
   MPU9250_REG_I2C_SLV1_REG   = 41,
   MPU9250_REG_I2C_SLV1_CTRL  = 42,
   MPU9250_REG_I2C_SLV4_ADDR  = 49,
   MPU9250_REG_I2C_SLV4_REG   = 50,
   MPU9250_REG_I2C_SLV4_DO    = 51,
   MPU9250_REG_I2C_SLV4_CTRL  = 52,
   MPU9250_REG_I2C_SLV4_DI    = 53,
   MPU9250_REG_I2C_MST_STATUS = 54,
   MPU9250_REG_INT_BYPASS     = 55,
   MPU9250_REG_INT_EN         = 56,
   MPU9250_REG_INT_STATUS     = 58,
   MPU9250_REG_I2C_SLV1_DO    = 100,
   MPU9250_REG_I2C_MST_DELAY_CTRL = 103,
   MPU9250_REG_USER_CTRL      = 106,
   MPU9250_REG_PWR_MGMT1      = 107,
   MPU9250_REG_PWR_MGMT2      = 108,
   MPU9250_REG_FIFO_COUNTH    = 114,
   MPU9250_REG_FIFO_COUNTL    = 115,
   MPU9250_REG_FIFO_RW        = 116,
   MPU9250_REG_WHOAMI         = 117,
};

/**
 * MPU9250 Compass Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_COMPASS_REG_ADDR {
   MPU9250_COMP_REG_WIA       = 0x00,
   MPU9250_COMP_REG_CNTL1     = 0x0a,
   MPU9250_COMP_REG_ASAX      = 0x10,
   MPU9250_COMP_REG_ASAY      = 0x11,
   MPU9250_COMP_REG_ASAZ      = 0x12,
};

// TODO-JYW: TESTING-TESTING
///**
// * Gyro Low Pass Filter Enum
// * MPU9X50_GYRO_LPF_250HZ and MPU9X50_GYRO_LPF_3600HZ_NOLPF is only applicable
// * for 8KHz sample rate. All other LPF values are only applicable for 1KHz
// * internal sample rate.
// */
//enum gyro_lpf_e {
//   MPU9X50_GYRO_LPF_250HZ = 0,  /**< 250Hz low pass filter for 8KHz gyro sampling*/
//   MPU9X50_GYRO_LPF_184HZ,
//   MPU9X50_GYRO_LPF_92HZ,
//   MPU9X50_GYRO_LPF_41HZ,
//   MPU9X50_GYRO_LPF_20HZ,
//   MPU9X50_GYRO_LPF_10HZ,
//   MPU9X50_GYRO_LPF_5HZ,
//   MPU9X50_GYRO_LPF_3600HZ_NOLPF, /**< 3600Hz low pass filter for 8KHz gyro sampling*/
//   NUM_MPU9X50_GYRO_LPF
//};
//
///**
// * Accelerometer Low Pass Filter Enum.
// */
//enum acc_lpf_e {
//   MPU9X50_ACC_LPF_460HZ = 0,
//   MPU9X50_ACC_LPF_184HZ,
//   MPU9X50_ACC_LPF_92HZ,
//   MPU9X50_ACC_LPF_41HZ,
//   MPU9X50_ACC_LPF_20HZ,
//   MPU9X50_ACC_LPF_10HZ,
//   MPU9X50_ACC_LPF_5HZ,
//   MPU9X50_ACC_LPF_460HZ_NOLPF,
//   NUM_MPU9X50_ACC_LPF
//};
//
///**
// * Gyro Full Scale Range Enum
// */
//enum gyro_fsr_e {
//   MPU9X50_GYRO_FSR_250DPS = 0,
//   MPU9X50_GYRO_FSR_500DPS,
//   MPU9X50_GYRO_FSR_1000DPS,
//   MPU9X50_GYRO_FSR_2000DPS,
//   NUM_MPU9X50_GYRO_FSR
//};
//
///**
// * Accelerometor Full Scale Range Enum
// */
//enum acc_fsr_e {
//   MPU9X50_ACC_FSR_2G = 0,
//   MPU9X50_ACC_FSR_4G,
//   MPU9X50_ACC_FSR_8G,
//   MPU9X50_ACC_FSR_16G,
//   NUM_MPU9X50_ACC_FSR
//};

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

/**
 * Full Scale Range of the magnetometer chip AK89xx in MPU9250
 */
#define MPU9250_AK89xx_FSR  4915 // from invensense doc

#define MPU9250_AKM_DEV_ID  0x48 // compass device ID

#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_I2C_IF_DIS      (0x10)
#define BIT_FIFO_RST        (0x04)
#define BIT_I2C_MST_RST     (0x02)
#define BIT_DMP_RST         (0x08)
#define BIT_RAW_RDY_EN      (0x01)
#define BIT_WAIT_FOR_ES     (0x40)
#define BIT_I2C_MST_EN      (0x20)
#define BIT_DELAY_ES_SHADOW (0x80)
#define BIT_SLV4_DLY_EN     (0x10)
#define BIT_SLV3_DLY_EN     (0x08)
#define BIT_SLV2_DLY_EN     (0x04)
#define BIT_SLV1_DLY_EN     (0x02)
#define BIT_SLV0_DLY_EN     (0x01)
#define BIT_FIFO_EN         (0x40)
#define BIT_TEMP_FIFO_EN    (0x80)
#define BIT_GYRO_FIFO_EN    (0x70)
#define BIT_ACCEL_FIFO_EN   (0x08)
#define BIT_FIFO_OVERFLOW_INT  (0x10)

#define MPU9250_FIFO_MAX_SIZE  (4000)
#define MPU9250_FIFO_DEFAULT_SIZE  (512)

/**
 * In practice, we shall not read too many bytes from FIFO at once.
 * Reading too many bytes at once result in chip hang.
 */
#define MPU9250_FIFO_SINGLE_READ_MAX_BYTES 256

class MPU9250_mag
{
public:
	MPU9250_mag(MPU9250 &imu, enum mag_sample_rate_e sample_rate) :
		_mag_initialized(false),
		_imu(imu),
		_sample_rate_in_hz(mag_sample_rate_enum_to_hz(sample_rate))
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
	int write_reg_verified(int reg, uint8_t val, uint8_t mask);

	/// @return 0 on success, -errno on failure
	int read_reg(uint8_t reg, uint8_t *val);

	/// @return 0 on success, -errno on failure
	int write_reg(uint8_t reg, uint8_t val);

	/// @return 0 on success, -errno on failure
	int process(struct fifo_packet_with_mag *fifo_packet);

private:
	float _mag_range_ga;
	float _mag_scaling;
	float _mag_initialized;
	int _mag_sens_adj[3];
	MPU9250 &_imu;
	int _sample_rate_in_hz;

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
