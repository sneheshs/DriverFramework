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

#include "MPU9250.hpp"
#include "MPU9250_mag.hpp"

using namespace DriverFramework;

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
#define BIT_I2C_MST_CLK_400_KHZ (0x0D)
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
#define BIT_I2C_SLV0_EN     (0x80)
#define BIT_I2C_SLV1_EN     (0x80)
#define BIT_I2C_SLV2_EN     (0x80)
#define BIT_I2C_SLV4_EN     (0x80)
#define BIT_I2C_SLV4_DONE   (0x40)

#define MPU9250_FIFO_MAX_SIZE  (4000)
#define MPU9250_FIFO_DEFAULT_SIZE  (512)

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

// Magnetometer device address
#define MPU9250_AK8963_I2C_ADDR  0x0C
#define MPU9250_AK8963_I2C_READ  0x80
#define MPU9250_AK8963_I2C_WRITE 0x00

// Bit definitions for the magnetometer registers
#define BIT_MAG_CNTL1_MODE_POWER_DOWN 0x0
#define BIT_MAG_CNTL1_MODE_SINGLE_MEASURE_MODE 0x1
#define BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_1 0x2
#define BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2 0x6
#define BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE 0xF
#define BIT_MAG_CNTL1_16_BITS 0x10

/**
 * MPU9250 Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_REG_ADDR
{
	MPU9250_REG_SMPLRT_DIV = 25,
	MPU9250_REG_CONFIG = 26,
	MPU9250_REG_GYRO_CONFIG = 27,
	MPU9250_REG_ACCEL_CONFIG = 28,
	MPU9250_REG_ACCEL_CONFIG2 = 29,
	MPU9250_REG_FIFO_ENABLE = 35,
	MPU9250_REG_I2C_MST_CTRL = 36,
	MPU9250_REG_I2C_SLV0_ADDR = 37,
	MPU9250_REG_I2C_SLV0_REG = 38,
	MPU9250_REG_I2C_SLV0_CTRL = 39,
	MPU9250_REG_I2C_SLV1_ADDR = 40,
	MPU9250_REG_I2C_SLV1_REG = 41,
	MPU9250_REG_I2C_SLV1_CTRL = 42,
	MPU9250_REG_I2C_SLV2_ADDR = 43,
	MPU9250_REG_I2C_SLV2_REG = 44,
	MPU9250_REG_I2C_SLV2_CTRL = 45,
	MPU9250_REG_I2C_SLV4_ADDR = 49,
	MPU9250_REG_I2C_SLV4_REG = 50,
	MPU9250_REG_I2C_SLV4_DO = 51,
	MPU9250_REG_I2C_SLV4_CTRL = 52,
	MPU9250_REG_I2C_SLV4_DI = 53,
	MPU9250_REG_I2C_MST_STATUS = 54,
	MPU9250_REG_INT_BYPASS = 55,
	MPU9250_REG_INT_EN = 56,
	MPU9250_REG_INT_STATUS = 58,
	MPU9250_REG_I2C_SLV1_DO = 100,
	MPU9250_REG_I2C_MST_DELAY_CTRL = 103,
	MPU9250_REG_USER_CTRL = 106,
	MPU9250_REG_PWR_MGMT1 = 107,
	MPU9250_REG_PWR_MGMT2 = 108,
	MPU9250_REG_FIFO_COUNTH = 114,
	MPU9250_REG_FIFO_COUNTL = 115,
	MPU9250_REG_FIFO_RW = 116,
	MPU9250_REG_WHOAMI = 117,
};

/**
 * MPU9250 Compass Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_COMPASS_REG_ADDR
{
	MPU9250_COMP_REG_WIA = 0x00,
	MPU9250_COMP_REG_ST1 = 0x02,
	MPU9250_COMP_REG_DATA = 0x03,
	MPU9250_COMP_REG_ST2 = 0x09,
	MPU9250_COMP_REG_CNTL1 = 0x0a,
	MPU9250_COMP_REG_ASAX = 0x10,
	MPU9250_COMP_REG_ASAY = 0x11,
	MPU9250_COMP_REG_ASAZ = 0x12,
};

/**
 * AK8963 (Included MPU-9250 package.)
 */
#define AK8963_I2C_ADDR                 (0x0c)
/* Registers. */
#define AK8963_REG_WIA                  (0x00)
#define AK8963_REG_INFO                 (0x01)
#define AK8963_REG_ST1                  (0x02)
#define AK8963_REG_HX_LH                (0x03)  //2 Bytes
#define AK8963_REG_HY_LH                (0x05)  //2 Bytes
#define AK8963_REG_HZ_LH                (0x07)  //2 Bytes
#define AK8963_REG_ST2                  (0x09)
#define AK8963_REG_CNTL1                (0x0A)
#define AK8963_REG_CNTL2                (0x0B)
#define AK8963_REG_ASTC                 (0x0C)
/* --- */
#define AK8963_REG_ASAX                 (0x10)
#define AK8963_REG_ASAY                 (0x11)
#define AK8963_REG_ASAZ                 (0x12)

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
//};REG

/**
 * In practice, we shall not read too many bytes from FIFO at once.
 * Reading too many bytes at once result in chip hang.
 */
#define MPU9250_FIFO_SINGLE_READ_MAX_BYTES 256

int MPU9250_mag::_convert_sample_rate_enum_to_hz(
		enum mag_sample_rate_e sample_rate)
{
	switch (sample_rate) {
		case MPU9x50_COMPASS_SAMPLE_RATE_100HZ:
			return 100;
		default:
			return -1;
	}
}

int MPU9250_mag::_initialize(int gyro_sample_rate_in_hz)
{
// TODO-JYW: TESTING-TESTING: Uncomment when complete.
	uint8_t i2c_mst_delay = 0;
	uint8_t i2c_mst_delay_ctrl = 0;
	uint8_t i2c_mst_ctrl = 0;
	uint8_t user_ctrl = 0;
	int result;

// TODO-JYW: TESTING-TESTING
//	gyro_sample_rate = gyro_sample_rate_enum_to_hz(config->gyro_sample_rate);
//	mag_sample_rate = mag_sample_rate_enum_to_hz(
//			config->mag_sample_rate);

// I2C_MST_CTRL
// - don't wait for external I2C data (WAIT_FOR_ES = 0),
// - set I2C master clock speed as 400KHz
// NOTE: I2C rate less than 500KHz may be problematic as we observed
// that the magnetometer data ready flag is cleared when the data ready
// interrupt fires. Not sure of the exact reason.

// TODO-JYW: TESTING-TESTING: Uncomment when complete.
//	i2c_mst_ctrl &= ~BIT_WAIT_FOR_ES;

// Configure the IMU as an I2C master at 400 KHz.
	i2c_mst_ctrl |= BIT_I2C_MST_CLK_400_KHZ;
	DF_LOG_INFO("Writing to the MPU9250_REG_I2C_MST_CTRL register.");
	result = _imu.writeReg(MPU9250_REG_I2C_MST_CTRL, i2c_mst_ctrl);
	if (result != 0) {
		DF_LOG_ERR("IMU I2C master bus config failed");
		return -1;
	}
	usleep(1000);

	// Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated
	// from pins SDA/SDI and SCL/ SCLK.
	user_ctrl |= BIT_I2C_MST_EN;
	result = _imu.writeReg(MPU9250_REG_USER_CTRL, user_ctrl);
	if (result != 0) {
		DF_LOG_ERR("Failed to enabled I2C master mode");
		return -1;
	}
	usleep(1000);

	// Detect mag presence by reading whoami register
	if (detect() != 0)
		return -1;
	DF_LOG_INFO("MPU9250 mag detected");

	// get mag calibraion data from Fuse ROM
	if (get_sensitivity_adjustment() != 0) {
		return -1;
	}
	DF_LOG_INFO("MPU9250 read mag sensitivity adjustment");

	// Power on and configure the mag to produce 16 bit data in continuous measurement mode,
	// at 100 Hz.
	write_reg(MPU9250_COMP_REG_CNTL1,
			BIT_MAG_CNTL1_16_BITS | BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2);
	usleep(10000);

	// Slave 0 provides ST1, mag data, and ST2 data in a bulk transfer of
	// 8 bytes of data.  Use the address of ST1 in SLV0_REG as the beginning
	// register of the 8 byte bulk transfer.
	_imu.writeReg(MPU9250_REG_I2C_SLV0_ADDR,
			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ);
	_imu.writeReg(MPU9250_REG_I2C_SLV0_REG, MPU9250_COMP_REG_ST1);
	_imu.writeReg(MPU9250_REG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | 0x08); // 0x08 = the number of bytes of data to be transferred
	usleep(10000);

	// Configure the rate at which the mag will be sampled for new data.
	// TODO-JYW: TESTING-TESTING:
	_imu.writeReg(MPU9250_REG_SMPLRT_DIV, 0x7F);

//	// Slave 1 provides the 6 bytes of mag data. Measurement data is
//	// stored in two’s complement in Little Endian format. Measurement range of
//	// each axis is from -32760 ~ 32760 decimal in 16-bit output.
//	_imu.writeReg(MPU9250_REG_I2C_SLV1_ADDR,
//			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ);
//	_imu.writeReg(MPU9250_REG_I2C_SLV1_REG, MPU9250_COMP_REG_DATA);
//	_imu.writeReg(MPU9250_REG_I2C_SLV1_CTRL, BIT_I2C_SLV1_EN | 0x06);
//
//	// Slave 2 provides the mag ST2 register value
//	_imu.writeReg(MPU9250_REG_I2C_SLV2_ADDR,
//			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ);
//	_imu.writeReg(MPU9250_REG_I2C_SLV2_REG, MPU9250_COMP_REG_ST1);
//	_imu.writeReg(MPU9250_REG_I2C_SLV2_CTRL, BIT_I2C_SLV2_EN | 0x01);

	// TODO-JYW: Is the following really needed, since the data is read anyway?

	// conduct 1 transfer at delayed sample rate
	// I2C_MST_DLY = (gryo_sample_rate / campass_sample_rate - 1)
	// TODO-JYW: TESTING-TESTING: Uncomment when complete.

	// Enable reading of the mag every n gyro samples, since the gyro is sampled
	// at a higher rate.
//	i2c_mst_delay = gyro_sample_rate_in_hz / _sample_rate_in_hz - 1;
	// TODO-JYW: TESTING-TESTING
//	i2c_mst_delay = gyro_sample_rate_in_hz / 100;
	i2c_mst_delay = 31;
//	i2c_mst_delay = 0;
	_imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, i2c_mst_delay);
	DF_LOG_INFO("Set I2C_SLV4_CTRL i2c_mst_dly = %u", i2c_mst_delay);
	usleep(10000);

	// Delay shadowing mag data until all of the data is received.
//	i2c_mst_delay_ctrl |= BIT_DELAY_ES_SHADOW;
	// Enable delayed I2C transfers for the mag on Slave 0 registers.
	i2c_mst_delay_ctrl |= BIT_SLV0_DLY_EN;
	_imu.writeReg(MPU9250_REG_I2C_MST_DELAY_CTRL, i2c_mst_delay_ctrl);
	DF_LOG_INFO("Enabled delayed access on I2C slave 0");
	usleep(10000);

	return 0;
}

int MPU9250_mag::initialize(int gyro_sample_rate_in_hz)
{
	DF_LOG_INFO("Entering initialize");

	// Retry up to 5 times to ensure successful initialization of the
	// sensor's internal I2C bus.
	int init_max_tries = 5;
	int ret = 0;
	int i;
	for (i = 0; i < init_max_tries; i++) {
		DF_LOG_INFO("Calling _initialize(%d)", gyro_sample_rate_in_hz);
		ret = _initialize(gyro_sample_rate_in_hz);
		if (ret == 0) {
			break;
		}
		DF_LOG_ERR("mag initialization failed %d tries", i + 1);
		usleep(10000);
	}

	if (ret == 0) {
		DF_LOG_INFO("mag initialization succ after %d retries", i);
		_mag_initialized = true;
	} else {
		DF_LOG_ERR("failed to initialize mag!");
	}

	return ret;
}

int MPU9250_mag::get_sensitivity_adjustment(void)
{
	int i;
	uint8_t asa[3];

	// First set power-down mode
	if (write_reg(MPU9250_COMP_REG_CNTL1, BIT_MAG_CNTL1_MODE_POWER_DOWN) != 0) {
		return -1;
	}
	usleep(10000);

	// Enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	if (write_reg(MPU9250_COMP_REG_CNTL1,
			BIT_MAG_CNTL1_16_BITS | BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE) != 0) {
		return -1;
	}
	usleep(10000);

	// Get compass calibration register 0x10, 0x11, 0x12
	// store into context
	for (i = 0; i < 3; i++) {
		if (read_reg(MPU9250_COMP_REG_ASAX + i, &asa[i]) != 0) {
			return -1;
		}
		_mag_sens_adj[i] = (float) (((float) asa[i] - 128.0) / 256.0) + 1.0f;
	}

	// Leave in a power-down mode
	if (write_reg(MPU9250_COMP_REG_CNTL1, BIT_MAG_CNTL1_MODE_POWER_DOWN) != 0) {
		return -1;
	}
	usleep(10000);

	DF_LOG_INFO("magnetometer sensitivity adjustment: %d %d %d",
			(int) (_mag_sens_adj[0] * 1000.0), (int) (_mag_sens_adj[1] * 1000.0), (int) (_mag_sens_adj[2] * 1000.0));
	return 0;
}

int MPU9250_mag::detect(void)
{
	uint8_t b = 0;

	// get mag version ID
	int retVal = read_reg(MPU9250_MAG_REG_WIA, &b);
	if (retVal != 0) {
		DF_LOG_ERR("error reading mag whoami reg: %d", retVal);
		return -1;
	}

	if (b != MPU9250_AKM_DEV_ID) {
		DF_LOG_ERR("wrong mag ID %u (expected %u)", b, MPU9250_AKM_DEV_ID);
		return -1;
	}

	return 0;
}

// int mpu_spi_write_reg_verified(int reg, uint8_t val, uint8_t mask)
int MPU9250_mag::write_imu_reg_verified(int reg, uint8_t val, uint8_t mask)
{
	int retVal;
	uint8_t b;
	int retry = 5;
	bool err_seen;

	while (retry) {
		err_seen = FALSE;
		--retry;
		retVal = _imu.writeReg(reg, val);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		retVal = _imu.readReg(reg, b);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		if ((b & mask) != val) {
			continue;
		} else {
			DF_LOG_INFO("set_mag_reg_verified succ for reg %d=%d", reg, val);
			return 0;
		}
	}

	if (err_seen) {
		DF_LOG_ERR("set_mag_reg_verified failed for reg %d. Error %d.",
				reg, retVal);
	} else {
		DF_LOG_ERR("set_mag_reg_verified failed for reg %d. %d!=%d",
				reg, val, b);
	}

	return retVal;
}

// int compass_read_register(uint8_t reg, uint8_t *val)

int MPU9250_mag::read_reg(uint8_t reg, uint8_t *val)
{
	int retVal = 0;
	uint8_t b = 0;

	// Read operation on the mag using the slave 4 registers.
	retVal = write_imu_reg_verified(MPU9250_REG_I2C_SLV4_ADDR,
			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the mag register to read from.
	retVal = write_imu_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Read the existing value of the SLV4 control register.
	retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Set the I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits. Enable data transfer, a read transfer as configured above.
	b |= BIT_I2C_SLV4_EN;
	// Trigger the data transfer
	retVal = _imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Continuously check I2C_MST_STATUS register value for the completion
	// of I2C transfer until timeout
	retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
	if (retVal != 0) {
		return retVal;
	}

	int loop_ctrl = 1000; // wait up to 1000 * 1 ms for completion
	while (((b & BIT_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
		if (retVal != 0) {
			return retVal;
		}
	}

	if (loop_ctrl == 0) {
		DF_LOG_ERR("I2C transfer timed out");
		return -1;
	}

	// Read the value received from the mag, and copy to the caller's out parameter.
	retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_DI, *val);
	if (retVal != 0) {
		return retVal;
	}

	DF_LOG_INFO("Mag register %u read returned %u", reg, *val);

	return 0;
}

// int compass_write_register(uint8_t reg, uint8_t val)
int MPU9250_mag::write_reg(uint8_t reg, uint8_t val)
{
	int retVal = 0;
	uint8_t b = 0;

	// Configure a write operation to the mag using Slave 4.
	retVal = write_imu_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, AK8963_I2C_ADDR,
			0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the mag register address to write to using Slave 4.
	retVal = write_imu_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the value to write in the I2C_SLV4_DO register.
	retVal = write_imu_reg_verified(MPU9250_REG_I2C_SLV4_DO, val, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Read the current value of the Slave 4 control register.
	retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits.
	b |= BIT_I2C_SLV4_EN;
	// Trigger the data transfer from the byte now stored in the SLV4_DO register.
	retVal = _imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
	if (retVal != 0) {
		return retVal;
	}

	int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion
	while (((b & BIT_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
		if (retVal != 0) {
			return retVal;
		}
	}

	if (loop_ctrl == 0) {
		DF_LOG_ERR("I2C transfer to mag timed out");
		return -1;
	}

	DF_LOG_INFO("Magnetometer register %u set to %u", reg, val);

	return 0;
}

int MPU9250_mag::process(struct fifo_packet_with_mag *fifo_packet)
{
	uint8_t status1 = fifo_packet->mag_st1;
	uint8_t status2 = fifo_packet->mag_st2;
	static int data_ready_bit_counter = 0;
	static int hofl_bit_counter = 0;
//	uint8_t mag_data_ready;

	// TODO-JYW: LEFT-OFF:
	// Change the code to not use the FIFO to read mag samples.  Periodically poll the
	// the data_ready_bit and transfer the data when it is ready.

//	DF_LOG_INFO("status1 %u status2 %u", status1, status2);
//	mag_data_ready = (status1 & 0x01);

//	data->mag_data_ready = (status1 & 0x01);
//	data->mag_range_ga = driver_context.mag_range_ga;
//	data->mag_scaling = driver_context.mag_scaling;
//
	// magnetic sensor overflow HOFL bit set
	// TODO-JYW: Replace the magic number.
	if (status2 & 0x08) {
		if ((++hofl_bit_counter % 10000) == 0) {
			DF_LOG_ERR("overflow HOFL bit set (x1000)");
		}
		return -3;
	}

	// Data Ready flag not set or data overrun bit set.
	int return_status;
	if (!(status1 & 0x01)) {
		// TODO-JYW: TESTING-TESTING
		if ((++data_ready_bit_counter % 10000) == 0) {
			DF_LOG_ERR("data ready bit not set (x10000)");
		}
		return -2;
	}
	else
	{
		return_status = 0;
	}

	// TODO-JYW: TESTING-TESTING:
//	_imu.writeReg(MPU9250_REG_I2C_SLV0_ADDR,
//			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ);
//	_imu.writeReg(MPU9250_REG_I2C_SLV0_REG, MPU9250_COMP_REG_ST1);
//	_imu.writeReg(MPU9250_REG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | 0x08); // 0x08 = the number of bytes of data to be transferred

//
//	data->mag_data_ready = true;

//	DF_LOG_INFO("valid mag sample detected");
	fifo_packet->mag_x = ImuSensor::swap16(fifo_packet->mag_x);
	fifo_packet->mag_y = ImuSensor::swap16(fifo_packet->mag_y);
	fifo_packet->mag_z = ImuSensor::swap16(fifo_packet->mag_z);

	// H_adj = H * ((ASA-128)*0.5/128 + 1)
	//       = H * ((ASA-128) / 256 + 1)
	// H is the raw compass reading, ((ASA-128) / 256 + 1) has been
	// computed and stored in compass_cal_f
	fifo_packet->mag_x =
			(int16_t) ((int) fifo_packet->mag_x * _mag_sens_adj[0]);
	fifo_packet->mag_y =
			(int16_t) ((int) fifo_packet->mag_y * _mag_sens_adj[1]);
	fifo_packet->mag_z =
			(int16_t) ((int) fifo_packet->mag_z * _mag_sens_adj[2]);

	// Convert the native units of the sensor (micro-Teslas) to Gauss
	fifo_packet->mag_x /= 100;
	fifo_packet->mag_y /= 100;
	fifo_packet->mag_z /= 100;

	// Swap magnetometer x and y axis
	// Magnetometer X axis = Gyro and Accel Y axis
	// Magnetometer Y axis = Gyro and Accel X axis
	// Magnetometer Z axis = Gyro and Accel Z axis
	int16_t temp_mag_x = fifo_packet->mag_x;
	fifo_packet->mag_x = fifo_packet->mag_y;
	fifo_packet->mag_y = temp_mag_x;

	return return_status;
}
