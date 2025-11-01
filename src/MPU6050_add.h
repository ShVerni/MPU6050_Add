/*
 * This file and associated .cpp file are licensed under the MIT License Copyright (c) 2025 Sam Groveman
 *
 * Portions have been adapted from the below libraries and are copyright their respective owners:
 * https://github.com/adafruit/Adafruit_MPU6050_add/ (BSD License)
 * https://github.com/tockn/MPU6050_tockn/ (MIT License)
 *
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/// @brief Interface for MPU6050
class MPU6050Add {
	public:
		/// @brief FSYNC output values
		enum fsync_out {
			MPU6050_FSYNC_OUT_DISABLED = 0x00,
			MPU6050_FSYNC_OUT_TEMP     = 0x08,
			MPU6050_FSYNC_OUT_GYROX    = 0x10,
			MPU6050_FSYNC_OUT_GYROY    = 0x18,
			MPU6050_FSYNC_OUT_GYROZ    = 0x20,
			MPU6050_FSYNC_OUT_ACCELX   = 0x28,
			MPU6050_FSYNC_OUT_ACCELY   = 0x30,
			MPU6050_FSYNC_OUT_ACCEL_Z  = 0x38
		};

		/// @brief Clock source options
		enum clock_select {
			MPU6050_INTR_8MHz     = 0x00,
			MPU6050_PLL_GYROX     = 0x01,
			MPU6050_PLL_GYROY     = 0x02,
			MPU6050_PLL_GYROZ     = 0x03,
			MPU6050_PLL_EXT_32K   = 0x04,
			MPU6050_PLL_EXT_19MHz = 0x05,
			MPU6050_STOP          = 0x07
		};

		/// @brief Accelerometer range options
		enum accel_range {
			MPU6050_RANGE_2_G  = 0x00, ///< +/- 2g (default value)
			MPU6050_RANGE_4_G  = 0x08, ///< +/- 4g
			MPU6050_RANGE_8_G  = 0x10, ///< +/- 8g
			MPU6050_RANGE_16_G = 0x18  ///< +/- 16g
		};

		
		/// @brief Gyroscope range options
		enum gyro_range {
			MPU6050_RANGE_250_DEG  = 0x00, ///< +/- 250 deg/s (default value)
			MPU6050_RANGE_500_DEG  = 0x08, ///< +/- 500 deg/s
			MPU6050_RANGE_1000_DEG = 0x10, ///< +/- 1000 deg/s
			MPU6050_RANGE_2000_DEG = 0x18  ///< +/- 2000 deg/s
		};

		/// @brief Digital low pass filter bandwidth options
		enum lowpass {
			MPU6050_BAND_260_HZ = 0x00, ///< Docs imply this disables the filter
			MPU6050_BAND_184_HZ = 0x01,
			MPU6050_BAND_94_HZ  = 0x02,
			MPU6050_BAND_44_HZ  = 0x03,
			MPU6050_BAND_21_HZ  = 0x04,
			MPU6050_BAND_10_HZ  = 0x05,
			MPU6050_BAND_5_HZ   = 0x06
		};

		/// @brief Accelerometer high pass filter options
		enum accel_highpass {
			MPU6050_HIGHPASS_DISABLE = 0x00,
			MPU6050_HIGHPASS_5_HZ    = 0x01,
			MPU6050_HIGHPASS_2_5_HZ  = 0x02,
			MPU6050_HIGHPASS_1_25_HZ = 0x03,
			MPU6050_HIGHPASS_0_63_HZ = 0x04,
			MPU6050_HIGHPASS_UNUSED  = 0x05,
			MPU6050_HIGHPASS_HOLD    = 0x06
		};

		/// @brief Periodic measurement options
		enum cycle_rate {
			MPU6050_CYCLE_1_25_HZ = 0x00,
			MPU6050_CYCLE_5_HZ    = 0x40,
			MPU6050_CYCLE_20_HZ   = 0x80,
			MPU6050_CYCLE_40_HZ   = 0xC0
		};

		MPU6050Add(int address = 0x68, TwoWire* i2c_bus = &Wire);
		MPU6050Add(float aC, float gC, int address = 0x68, TwoWire* i2c_bus = &Wire);

		bool begin();
		void setGyroOffsets(float x, float y, float z);

		/// @brief Gets the raw (register) X reading of the accelerometer
		/// @return The reading as 16-bit int
		int16_t getRawAccX(){ return rawAccX; };
		
		/// @brief Gets the raw (register) Y reading of the accelerometer
		/// @return The reading as 16-bit int
		int16_t getRawAccY(){ return rawAccY; };

		/// @brief Gets the raw (register) Z reading of the accelerometer
		/// @return The reading as 16-bit int
		int16_t getRawAccZ(){ return rawAccZ; };

		/// @brief Gets the raw (register) temperature reading
		/// @return The reading as 16-bit int
		int16_t getRawTemperature(){ return rawTemp; };

		/// @brief Gets the raw (register) X reading of the gyroscope
		/// @return The reading as 16-bit int
		int16_t getRawGyroX(){ return rawGyroX; };

		/// @brief Gets the raw (register) Y reading of the gyroscope
		/// @return The reading as 16-bit int
		int16_t getRawGyroY(){ return rawGyroY; };

		/// @brief Gets the raw (register) Z reading of the gyroscope
		/// @return The reading as 16-bit int
		int16_t getRawGyroZ(){ return rawGyroZ; };

		/// @brief Gets the calculated temperature reading
		/// @return The reading as 16-bit int
		float getTemperature(){ return temp; };

		/// @brief Gets the calculated X reading of the accelerometer
		/// @return The reading as 16-bit int
		float getAccX(){ return accX; };
		
		/// @brief Gets the calculated Y reading of the accelerometer
		/// @return The reading as 16-bit int
		float getAccY(){ return accY; };

		/// @brief Gets the calculated Z reading of the accelerometer
		/// @return The reading as 16-bit int
		float getAccZ(){ return accZ; };

		/// @brief Gets the calculated X reading of the gyroscope
		/// @return The reading as 16-bit int
		float getGyroX(){ return gyroX; };

		/// @brief Gets the calculated Y reading of the gyroscope
		/// @return The reading as 16-bit int
		float getGyroY(){ return gyroY; };

		/// @brief Gets the calculated Z reading of the gyroscope
		/// @return The reading as 16-bit int
		float getGyroZ(){ return gyroZ; };

		/// @brief Gets the X angle calculated from the accelerometer alone
		/// @return A float with the angle in degrees
		float getAccAngleX(){ return angleAccX; };

		/// @brief Gets the Y angle calculated from the accelerometer alone
		/// @return A float with the angle in degrees
		float getAccAngleY(){ return angleAccY; };

		/// @brief Gets the Z angle calculated from the accelerometer alone
		/// @return A float with the angle in degrees
		float getAccAngleZ(){ return angleAccZ; };

		/// @brief Gets the X angle calculated from the gyroscope alone
		/// @return A float with the angle in degrees
		float getGyroAngleX(){ return angleGyroX; };

		/// @brief Gets the X angle calculated from the gyroscope alone
		/// @return A float with the angle in degrees
		float getGyroAngleY(){ return angleGyroY; };

		/// @brief Gets the X angle calculated from the gyroscope alone
		/// @return A float with the angle in degrees
		float getGyroAngleZ(){ return angleGyroZ; };

		/// @brief Gets the X angle calculated from the gyroscope and accelerometer combined
		/// @return A float with the angle in degrees
		float getAngleX(){ return angleX; };

		/// @brief Gets the Y angle calculated from the gyroscope and accelerometer combined
		/// @return A float with the angle in degrees
		float getAngleY(){ return angleY; };

		/// @brief Gets the Z angle calculated from the gyroscope and accelerometer combined
		/// @return A float with the angle in degrees
		float getAngleZ(){ return angleZ; };

		/// @brief gets current X-axis offset for gyroscope
		float getGyroXoffset(){ return gyroXoffset; };

		/// @brief gets current Y-axis offset for gyroscope
		float getGyroYoffset(){ return gyroYoffset; };

		/// @brief gets current Z-axis offset for gyroscope
		float getGyroZoffset(){ return gyroZoffset; };

		accel_range getAccelerometerRange();
		void setAccelerometerRange(accel_range range);

		gyro_range getGyroRange();
		void setGyroRange(gyro_range range);

		void setInterruptPinPolarity(bool active_low);
		void setInterruptPinLatch(bool held);

		accel_highpass getHighPassFilter();
		void setHighPassFilter(accel_highpass bandwidth);

		void setMotionInterrupt(bool active);
		void setMotionDetectionThreshold(uint8_t thr);
		void setMotionDetectionDuration(uint8_t dur);
		bool getMotionInterruptStatus();

		fsync_out getFsyncSampleOutput();
		void setFsyncSampleOutput(fsync_out fsync_output);

		void setI2CBypass(bool bypass);

		clock_select getClock();
		void setClock(clock_select clock);

		lowpass getFilterBandwidth();
		void setFilterBandwidth(lowpass bandwidth);

		uint8_t getSampleRateDivisor();
		void setSampleRateDivisor(uint8_t);

		void enableSleep(bool enable);
		void enableCycle(bool enable);

		cycle_rate getCycleRate();
		void setCycleRate(cycle_rate rate);

		void setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
		void setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
		void setTemperatureStandby(bool enable);
		
		void resetAngles();
		void calcGyroOffsets(uint16_t delayBefore = 1000, uint16_t delayAfter = 1000, bool console = false);
		void update();

	private:
		/// @brief Register addresses for the MPU6050
		enum Registers {
			MPU6050_SELF_TEST_X       = 0x0D,
			MPU6050_SELF_TEST_Y       = 0x0E,
			MPU6050_SELF_TEST_Z       = 0x0F,
			MPU6050_SELF_TEST_A       = 0x10,
			MPU6050_SMPLRT_DIV        = 0x19,
			MPU6050_CONFIG            = 0x1A,
			MPU6050_GYRO_CONFIG       = 0x1B,
			MPU6050_ACCEL_CONFIG      = 0x1C,
			MPU6050_MOT_THR 		  = 0x1F,
			MPU6050_MOT_DUR           = 0x20,
			MPU6050_INT_PIN_CONFIG    = 0x37,
			MPU6050_INT_ENABLE        = 0x38,
			MPU6050_INT_STATUS        = 0x3A,
			MPU6050_ACCEL_OUT         = 0x3B,
			MPU6050_TEMP_H            = 0x41,
			MPU6050_TEMP_L            = 0x42,
			MPU6050_GYRO_OUT		  = 0x43,
			MPU6050_SIGNAL_PATH_RESET = 0x68,
			MPU6050_USER_CTRL         = 0x6A,
			MPU6050_PWR_MGMT_1        = 0x6B,
			MPU6050_PWR_MGMT_2        = 0x6C,
			MPU6050_WHO_AM_I          = 0x75
		};

		/// @brief Bit masks for writing to specific bits in registers
		enum Masks {
			FSYNC_OUT_MASK      = 0xC7,
			CLOCK_SELECT_MASK   = 0xF8,
			ACCEL_RANGE_MASK    = 0xE7,
			GYRO_RANG_MASK      = 0xE7,
			LOWPASS_MASK        = 0xF8,
			ACCEL_HIGHPASS_MASK = 0xF8,
			CYCLE_RATE_MASK     = 0x3F
		};

		/// @brief the I2C bus to use
		TwoWire* wire;

		/// @brief The I2C address of the MPU6050
		int _address;

		/// @brief Stores raw sensor readings
		int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

		/// @brief Stores the calibration factors for the gyro
		float gyroXoffset, gyroYoffset, gyroZoffset;

		/// @brief Stores corrected sensor readings
		float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

		/// @brief Stores calculated angle readings
		float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY, angleAccZ;

		/// @brief Stores composite angle (angleGyro combined with angleAcc)
		float angleX, angleY, angleZ;

		/// @brief Stores last measurement time for total angle calculations (from deg/s to deg)
		long preInterval;

		/// @brief Stores coefficients for gyro and accelerometer angle calculations
		float accCoef, gyroCoef;

		void writeBits(Registers reg, uint8_t data, uint8_t mask);
		uint8_t readBits(Registers reg, uint8_t mask);
		
		void writeMPU6050(Registers reg, uint8_t data);
		uint8_t readMPU6050(Registers reg);
};