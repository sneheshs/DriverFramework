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
#define BIT_DELAY_ES_SHADOW (0x80)https://confluence.qualcomm.com/confluence/display/AT/Source+Level+Debugging+of+Dynamic+Libraries
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

/**
 * In practice, we shall not read too many bytes from FIFO at once.
 * Reading too many bytes at once result in chip hang.
 */
#define MPU9250_FIFO_SINGLE_READ_MAX_BYTES 256

int MPU9250_mag::_convert_sample_rate_enum_to_hz(enum mag_sample_rate_e sample_rate)
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
	i2c_mst_ctrl &= ~BIT_WAIT_FOR_ES;
	i2c_mst_ctrl |= BIT_I2C_MST_CLK_400_KHZ;
	DF_LOG_INFO("Writing to the MPU9250_REG_I2C_MST_CTRL register.");
	result = _imu.writeReg(MPU9250_REG_I2C_MST_CTRL, i2c_mst_ctrl);
	if (result != 0) {
		DF_LOG_ERR("IMU I2C master bus config failed");
		return -1;
	}

	// Configure mpu9250 as I2C master to communicate with mag slave device
	user_ctrl |= BIT_I2C_MST_EN;
	result = _imu.writeReg(MPU9250_REG_USER_CTRL, user_ctrl);
	if (result != 0) {
		DF_LOG_ERR("Failed to enabled I2C master mode");
		return -1;
	}

	// Detect mag presence by reading whoami register
	if (detect() != 0)
		return -1;
	DF_LOG_INFO("MPU9250 mag detected");

	// get mag calibraion data from Fuse ROM
	if (get_sensitivity_adjustment() != 0)
		return -1;
	DF_LOG_INFO("MPU9250 read mag sensitivity adjustment");

	// Slave 0 reads from slave address 0x0C, i.e. mag
	_imu.writeReg(MPU9250_REG_I2C_SLV0_ADDR, 0x8c);

	// start reading from register 0x02 ST1
	_imu.writeReg(MPU9250_REG_I2C_SLV0_REG, 0x02);

	// Enable 8 bytes reading from slave 0 at gyro sample rate
	// NOTE: mag sample rate is lower than gyro sample rate, so
	// the same measurement samples are transferred to SLV0 at gyro sample rate.
	// but this is fine
	_imu.writeReg(MPU9250_REG_I2C_SLV0_CTRL, 0x88);

	// Slave 1 sets mag measurement mode
	_imu.writeReg(MPU9250_REG_I2C_SLV1_ADDR, 0x0C);

	// write to mag CNTL1 register
	_imu.writeReg(MPU9250_REG_I2C_SLV1_REG, MPU9250_COMP_REG_CNTL1);

	// enable 1 byte write on slave 1
	_imu.writeReg(MPU9250_REG_I2C_SLV1_CTRL, 0x81);

	// set slave 1 data to write
	// 0x10 indicates use 16bit mag measurement
	// 0x01 indicates using single measurement mode
	_imu.writeReg(MPU9250_REG_I2C_SLV1_DO, 0x11);

	// conduct 1 trasnfer at delayed sample rate
	// I2C_MST_DLY = (gryo_sample_rate / campass_sample_rate - 1)
	i2c_mst_delay = gyro_sample_rate_in_hz / _sample_rate_in_hz - 1;
	_imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, i2c_mst_delay);
	DF_LOG_INFO("Set I2C_SLV4_CTRL i2c_mst_dly = %u", i2c_mst_delay);

	// trigger slave 1 transfer every (1+I2C_MST_DLY) samples
	// every time slave 1 transfer is triggered, a mag measurement conducted
	// reg_cfg.i2c_mst_delay_ctrl |= BIT_DELAY_ES_SHADOW;
	i2c_mst_delay_ctrl |= BIT_SLV1_DLY_EN;
	_imu.writeReg(MPU9250_REG_I2C_MST_DELAY_CTRL, i2c_mst_delay_ctrl);
	DF_LOG_INFO("Enabled delayed access on I2C slave 4");

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

	// first set power-down mode

	if (write_reg(MPU9250_COMP_REG_CNTL1, 0x00) != 0)
		return -1;
	usleep(1000);

	// enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	if (write_reg(MPU9250_COMP_REG_CNTL1, 0x1f) != 0)
		return -1;
	usleep(1000);

	// get compass calibration register 0x10, 0x11, 0x12
	// store into context
	for (i = 0; i < 3; i++) {
		if (read_reg(MPU9250_COMP_REG_ASAX + i, asa + i) != 0)
			return -1;
		_mag_sens_adj[i] = (float) ((float) asa[i] - 128.0)
				/ 256.0 + 1.0f;
	}

	// set power-down mode
	if (write_reg(MPU9250_COMP_REG_CNTL1, 0x00) != 0)
		return -1;
	usleep(1000);

	DF_LOG_INFO("magnetometer sensitivity adjustment: %d %d %d",
			(int) (_mag_sens_adj[0] * 1000.0),
			(int) (_mag_sens_adj[1] * 1000.0),
			(int) (_mag_sens_adj[2] * 1000.0));
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
int MPU9250_mag::write_reg_verified(int reg, uint8_t val, uint8_t mask)
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
		DF_LOG_ERR("set_mag_reg_verified failed for reg %d. %d!=%d", reg,
				val, b);
	}

	return retVal;
}

// int compass_read_register(uint8_t reg, uint8_t *val)

int MPU9250_mag::read_reg(uint8_t reg, uint8_t *val)
{
	int retVal = 0;
	uint8_t b = 0;

	// I2C_SLV4_ADDR
	// write operation on compass address 0x0C
	retVal = write_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x8c, 0xff);
	if (retVal != 0)
		return retVal;

	// I2C_SLV4_REG
	// set the compass register address to write to
	retVal = write_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0)
		return retVal;

	retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0)
		return retVal;

	// set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits, which specifies the sample rate
	b |= 0x80;
	// Trigger the data transfer
	retVal = _imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0)
		return retVal;

	int loop_ctrl = 1000; // wait up to 1000 * 1 ms for completion

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
	if (retVal != 0)
		return retVal;

	while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
		if (retVal != 0)
			return retVal;
	}

	if (loop_ctrl == 0) {
		DF_LOG_ERR("I2C transfer timed out");
		return -1;
	}

	// get value into pointer provided on call
	retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_DI, *val);
	if (retVal != 0)
		return retVal;

	DF_LOG_INFO("Mag register %u read returned %u", reg, *val);

	return 0;
}

// int compass_write_register(uint8_t reg, uint8_t val)
int MPU9250_mag::write_reg(uint8_t reg, uint8_t val)
{
   int retVal = 0;
   uint8_t b = 0;

   // I2C_SLV4_ADDR
   // write operation on compass address 0x0C
   retVal = write_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x0c, 0xff);
   if (retVal != 0) return retVal;

   // I2C_SLV4_REG
   // set the compass register address to write to
   retVal = write_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
   if (retVal != 0) return retVal;

   // I2C_SLV4_DO
   // set the value to write in I2C_SLV4_DO register
   retVal = write_reg_verified(MPU9250_REG_I2C_SLV4_DO, val, 0xff);
   if (retVal != 0) return retVal;

   retVal = _imu.readReg(MPU9250_REG_I2C_SLV4_CTRL, b);
   if (retVal != 0) return retVal;

   // set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
   // bits, which specifies the sample rate
   b |= 0x80;
   // Trigger the data transfer
   retVal = _imu.writeReg(MPU9250_REG_I2C_SLV4_CTRL, b);
   if (retVal != 0) return retVal;

   int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion

   // Continuously check I2C_MST_STATUS regsiter value for the completion
   // of I2C transfer until timeout
   retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
   if (retVal != 0) return retVal;

   while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
      usleep(1000);
      retVal = _imu.readReg(MPU9250_REG_I2C_MST_STATUS, b);
      if (retVal != 0) return retVal;
   }

   if (loop_ctrl == 0)
   {
	   DF_LOG_ERR("I2C transfer timed out");
      return -1;
   }

   DF_LOG_INFO("Magnetometer register %u set to %u", reg, val);

   return 0;
}

int MPU9250_mag::process(struct fifo_packet_with_mag *fifo_packet)
{
	uint8_t status1 = fifo_packet->mag_st1;
	uint8_t status2 = fifo_packet->mag_st2;
//	uint8_t mag_data_ready;

	DF_LOG_INFO("status1 %u status2 %u", status1, status2);
//	mag_data_ready = (status1 & 0x01);

//	data->mag_data_ready = (status1 & 0x01);
//	data->mag_range_ga = driver_context.mag_range_ga;
//	data->mag_scaling = driver_context.mag_scaling;
//
	// Data Ready flag not set or data overrun bit set
	if (!(status1 & 0x01)) {
		DF_LOG_ERR("data ready bit not set");
	    return -2;
	}
//
	// magnetic sensor overflow HOFL bit set
	if (status2 & 0x08) {
		DF_LOG_ERR("overflow HOFL bit set");
		return -3;
	}
//
//	data->mag_data_ready = true;

	DF_LOG_INFO("valid mag sample detected");
	fifo_packet->mag_x = ImuSensor::swap16(fifo_packet->mag_x);
	fifo_packet->mag_y = ImuSensor::swap16(fifo_packet->mag_y);
	fifo_packet->mag_z = ImuSensor::swap16(fifo_packet->mag_z);

	// H_adj = H * ((ASA-128)*0.5/128 + 1)
	//       = H * ((ASA-128) / 256 + 1)
	// H is the raw compass reading, ((ASA-128) / 256 + 1) has been
	// computed and stored in compass_cal_f
	fifo_packet->mag_x = (int16_t)((int)fifo_packet->mag_x * _mag_sens_adj[0]);
	fifo_packet->mag_y = (int16_t)((int)fifo_packet->mag_y * _mag_sens_adj[1]);
	fifo_packet->mag_z = (int16_t)((int)fifo_packet->mag_z * _mag_sens_adj[2]);

	// swap magnetometer x and y axis
	// Magnetometer X axis = Gyro and Accel Y axis
	// Magnetometer Y axis = Gyro and Accel X axis
	// Magnetometer Z axis = - Gyro and Accel Z axis
	int16_t temp_mag_x = fifo_packet->mag_x;
	fifo_packet->mag_x = fifo_packet->mag_y;
	fifo_packet->mag_y = temp_mag_x;

	return 0;
}
