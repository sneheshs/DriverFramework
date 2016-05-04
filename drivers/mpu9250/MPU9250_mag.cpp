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

int MPU9250_mag::initialize()
{
//	int mag_sample_rate;
//	int gyro_sample_rate;
	uint8_t i2c_mst_delay;
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
	i2c_mst_ctrl |= BITS_I2C_MST_CLK_400_KHZ;
	result = write_mag_reg(MPUREG_I2C_MST_CTRL, i2c_mst_ctrl);
	if (result != 0) {
		DF_LOG_ERR("IMU I2C master bus config failed");
		return -1;
	}

	// Configure mpu9250 as I2C master to communicate with mag slave device
	user_ctrl |= BIT_I2C_MST_EN;
	result = write_mag_reg(MPUREG_USER_CTRL, reg_cfg.user_ctrl);
	if (result != 0) {
		DF_LOG_ERR("Failed to enabled I2C master mode");
		return -1;
	}

	// Detect mag presence by reading whoami register
	if (detect_mag() != 0)
		return -1;
	FARF(MEDIUM, "MPU9250 mag detected");

	// get mag calibraion data from Fuse ROM
	if (get_mag_senitivity_adjustment() != 0)
		return -1;
	FARF(MEDIUM, "MPU9250 read mag sensitivity adjustment");

	// Slave 0 reads from slave address 0x0C, i.e. mag
	imu->writeReg(MPU9250_REG_I2C_SLV0_ADDR, 0x8c);

	// start reading from register 0x02 ST1
	imu->writeReg(MPU9250_REG_I2C_SLV0_REG, 0x02);

	// TODO-JYW: LEFT-OFF:

	// Enable 8 bytes reading from slave 0 at gyro sample rate
	// NOTE: mag sample rate is lower than gyro sample rate, so
	// the same measurement samples are transferred to SLV0 at gyro sample rate.
	// but this is fine
	mpu_spi_set_reg(MPU9250_REG_I2C_SLV0_CTRL, 0x88);

	// Slave 1 sets mag measurement mode
	mpu_spi_set_reg(MPU9250_REG_I2C_SLV1_ADDR, 0x0C);

	// write to mag CNTL1 register
	mpu_spi_set_reg(MPU9250_REG_I2C_SLV1_REG, MPU9250_COMP_REG_CNTL1);

	// enable 1 byte write on slave 1
	mpu_spi_set_reg(MPU9250_REG_I2C_SLV1_CTRL, 0x81);

	// set slave 1 data to write
	// 0x10 indicates use 16bit mag measurement
	// 0x01 indicates using single measurement mode
	mpu_spi_set_reg(MPU9250_REG_I2C_SLV1_DO, 0x11);

// TODO-JYW: TESTING-TESTING
	// conduct 1 trasnfer at delayed sample rate
	// I2C_MST_DLY = (gryo_sample_rate / campass_sample_rate - 1)
//	i2c_mst_delay = gyro_sample_rate / mag_sample_rate - 1;
//	mpu_spi_set_reg(MPU9250_REG_I2C_SLV4_CTRL, i2c_mst_delay);
//	FARF(MEDIUM, "Set I2C_SLV4_CTRL i2c_mst_dly = %u", i2c_mst_delay);

	// trigger slave 1 transfer every (1+I2C_MST_DLY) samples
	// every time slave 1 transfer is triggered, a mag measurement conducted
	// reg_cfg.i2c_mst_delay_ctrl |= BIT_DELAY_ES_SHADOW;
	reg_cfg.i2c_mst_delay_ctrl |= BIT_SLV1_DLY_EN;
	mpu_spi_set_reg(MPU9250_REG_I2C_MST_DELAY_CTRL, reg_cfg.i2c_mst_delay_ctrl);
	FARF(MEDIUM, "Enabled delayed access on I2C slave 4");

	return 0;
}

int MPU9250_mag::get_mag_senitivity_adjustment()
{
	int i;
	uint8_t asa[3];

	// first set power-down mode

	if (write_mag_reg(MPU9250_COMP_REG_CNTL1, 0x00) != 0)
		return -1;
	usleep(1000);

	// enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	if (write_mag_reg(MPU9250_COMP_REG_CNTL1, 0x1f) != 0)
		return -1;
	usleep(1000);

	// get compass calibration register 0x10, 0x11, 0x12
	// store into context
	for (i = 0; i < 3; i++) {
		if (read_mag_reg(MPU9250_COMP_REG_ASAX + i, asa + i) != 0)
			return -1;
		_mag_sens_adj[i] = (float) ((float) asa[i] - 128.0)
				/ 256.0 + 1.0f;
	}

	// set power-down mode
	if (write_mag_reg(MPU9250_COMP_REG_CNTL1, 0x00) != 0)
		return -1;
	usleep(1000);

	FARF(MEDIUM, "magnetometer sensitivity adjustment: %d %d %d",
			(int) (_mag_sens_adj[0] * 1000.0),
			(int) (_mag_sens_adj[1] * 1000.0),
			(int) (_mag_sens_adj[2] * 1000.0));
	return 0;
}

int MPU9250_mag::detect_mag()
{
	uint8_t b = 0;

	// get mag version ID
	int retVal = read_mag_reg(MPUREG_MAG_REG_WIA, &b);
	if (retVal != 0) {
		FARF(ALWAYS, "error reading mag whoami reg: %d", retVal);
		return -1;
	}

	if (b != MPU9250_AKM_DEV_ID) {
	  FARF(ALWAYS, "wrong mag ID %u (expected %u)", b, MPU9250_AKM_DEV_ID);
	  return -1;
	}

	return 0;
}

// int compass_read_register(uint8_t reg, uint8_t *val)

int MPU9250_mag::read_mag_reg(uint8_t reg, uint8_t *val)
{
	int retVal = 0;
	uint8_t b = 0;

	// I2C_SLV4_ADDR
	// write operation on compass address 0x0C
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x8c, 0xff);
	if (retVal != 0)
		return retVal;

	// I2C_SLV4_REG
	// set the compass register address to write to
	retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0)
		return retVal;

	retVal = imu->readReg(MPU9250_REG_I2C_SLV4_CTRL, &b);
	if (retVal != 0)
		return retVal;

	// set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits, which specifies the sample rate
	b |= 0x80;
	// Trigger the data transfer
	retVal = imu->writeReg(MPU9250_REG_I2C_SLV4_CTRL, b);
	if (retVal != 0)
		return retVal;

	int loop_ctrl = 1000; // wait up to 1000 * 1 ms for completion

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);
	if (retVal != 0)
		return retVal;

	while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);
		if (retVal != 0)
			return retVal;
	}

	if (loop_ctrl == 0) {
		FARF(ALWAYS, "I2C transfer timed out");
		return -1;
	}

	// get value into pointer provided on call
	retVal = mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_DI, val);
	if (retVal != 0)
		return retVal;

	FARF(MEDIUM, "Compass register %u read returned %u", reg, *val);

	return 0;
}

// int mpu_spi_set_reg_verified(int reg, uint8_t val, uint8_t mask)
int MPU9250_mag::set_mag_reg_verified(int reg, uint8_t val, uint8_t mask)
{
	int retVal;
	uint8_t b;
	int retry = 5;
	bool err_seen;

	while (retry) {
		err_seen = FALSE;
		--retry;
		retVal = imu->writeReg(reg, val);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		retVal = imu->readReg(reg, &b);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		if ((b & mask) != val) {
			continue;
		} else {
			FARF(LOW, "mpu_spi_set_reg_verified succ for reg %d=%d", reg, val);
			return 0;
		}
	}

	if (err_seen) {
		FARF(ALWAYS, "mpu_spi_set_reg_verified failed for reg %d. Error %d.",
				reg, retVal);
	} else {
		FARF(ALWAYS, "mpu_spi_set_reg_verified failed for reg %d. %d!=%d", reg,
				val, b);
	}

	return retVal;
}


// int compass_write_register(uint8_t reg, uint8_t val)

int MPU9250_mag::write_mag_reg(uint8_t reg, uint8_t val)
{
   int retVal = 0;
   uint8_t b = 0;

   // I2C_SLV4_ADDR
   // write operation on compass address 0x0C
   retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_ADDR, 0x0c, 0xff);
   if (retVal != 0) return retVal;

   // I2C_SLV4_REG
   // set the compass register address to write to
   retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_REG, reg, 0xff);
   if (retVal != 0) return retVal;

   // I2C_SLV4_DO
   // set the value to write in I2C_SLV4_DO register
   retVal = mpu_spi_set_reg_verified(MPU9250_REG_I2C_SLV4_DO, val, 0xff);
   if (retVal != 0) return retVal;

   retVal = mpu_spi_get_reg(MPU9250_REG_I2C_SLV4_CTRL, &b);
   if (retVal != 0) return retVal;

   // set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
   // bits, which specifies the sample rate
   b |= 0x80;
   // Trigger the data transfer
   retVal = mpu_spi_set_reg(MPU9250_REG_I2C_SLV4_CTRL, b);
   if (retVal != 0) return retVal;

   int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion

   // Continuously check I2C_MST_STATUS regsiter value for the completion
   // of I2C transfer until timeout
   retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);
   if (retVal != 0) return retVal;

   while (((b & 0x40) == 0x00) && (--loop_ctrl)) {
      usleep(1000);
      retVal = mpu_spi_get_reg(MPU9250_REG_I2C_MST_STATUS, &b);
      if (retVal != 0) return retVal;
   }

   if (loop_ctrl == 0)
   {
      FARF(ALWAYS, "I2C transfer timed out");
      return -1;
   }

   FARF(MEDIUM, "Compass register %u set to %u", reg, val);

   return 0;
}

};
