#include "MPU6050_add.h"

/// @brief Creates an MPU6050 device
/// @param address The I2C address to use
/// @param i2c_bus The I2C bus to use
MPU6050Add::MPU6050Add(int address, TwoWire* i2c_bus) {
	wire = i2c_bus;
	accCoef = 0.02f;
	gyroCoef = 0.98f;
	_address = address;
}

/// @brief Creates an MPU6050
/// @param aC The accelerometer coefficient to use for angle calculations
/// @param gC The gyroscope coefficient to use for angle calculations
/// @param address The I2C address to use
/// @param i2c_bus The I2C bus to use
MPU6050Add::MPU6050Add(float aC, float gC, int address, TwoWire* i2c_bus) {
	wire = i2c_bus;
	accCoef = aC;
	gyroCoef = gC;
	_address = address;
}

/// @brief Starts an MP6050
/// @return True on success
bool MPU6050Add::begin() {
	if (!wire->begin()) {
		return false;
	}
	// Make sure we're talking to the right chip
	if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
		return false;
	}
	return true;
}

/// @brief Resets all cumulative angle measurements
void MPU6050Add::resetAngles() {
	angleGyroX = 0;
	angleGyroY = 0;
	angleX = 0;
	angleY = 0;
	preInterval = millis();
}

/// @brief Gets the sample rate divisor
/// @return The sample rate divisor
uint8_t MPU6050Add::getSampleRateDivisor() {
	return readMPU6050(MPU6050_SMPLRT_DIV);
}

/// @brief Sets the divisor used to divide the base clock rate into a measurement rate
/// @param divisor The new clock divisor
void MPU6050Add::setSampleRateDivisor(uint8_t divisor) {
	writeMPU6050(MPU6050_SMPLRT_DIV, divisor);
}

/// @brief Gets the acceleration measurement range
/// @return The acceleration measurement range
MPU6050Add::accel_range MPU6050Add::getAccelerometerRange() {
	return (accel_range)readBits(MPU6050_ACCEL_CONFIG, (uint8_t)ACCEL_RANGE_MASK);
}

/// @brief Sets the accelerometer measurement range
/// @param  new_range The new range to set
void MPU6050Add::setAccelerometerRange(accel_range new_range) {
	writeBits(MPU6050_ACCEL_CONFIG, (uint8_t)new_range, (uint8_t)ACCEL_RANGE_MASK);
}

/// @brief Gets the gyroscope measurement range
/// @return The gyroscope measurement range
MPU6050Add::gyro_range MPU6050Add::getGyroRange() {
	return (gyro_range)readBits(MPU6050_GYRO_CONFIG, (uint8_t)GYRO_RANG_MASK);
}

/// @brief Sets the gyroscope measurement range
/// @param new_range The new range to set
void MPU6050Add::setGyroRange(gyro_range new_range) {
	writeBits(MPU6050_GYRO_CONFIG, (uint8_t)new_range, (uint8_t)GYRO_RANG_MASK);
}

/// @brief Gets clock source
/// @return The current clock source
MPU6050Add::clock_select MPU6050Add::getClock() {
	return (clock_select)readBits(MPU6050_PWR_MGMT_1, (uint8_t)CLOCK_SELECT_MASK);
}

/// @brief Sets clock source
/// @param new_cloc The clock source to set
void MPU6050Add::setClock(clock_select new_clock) {
	writeBits(MPU6050_PWR_MGMT_1, (uint8_t)new_clock, (uint8_t)CLOCK_SELECT_MASK);
}

/// @brief Gets the location that the FSYNC pin sample is stored
/// @return The fsync_output
MPU6050Add::fsync_out MPU6050Add::getFsyncSampleOutput() {
	return (fsync_out)readBits(MPU6050_CONFIG, (uint8_t)FSYNC_OUT_MASK);
}

/// @brief Sets the location that the FSYNC pin sample is stored
/// @param fsync_output Specifies the LSB of which data register should be used to store the state of the FSYNC pin
void MPU6050Add::setFsyncSampleOutput(fsync_out fsync_output) {
	writeBits(MPU6050_CONFIG, (uint8_t)fsync_output, (uint8_t)FSYNC_OUT_MASK);
}

/// @brief Gets bandwidth of the digital low-pass filter
/// @return The current filter bandwidth
MPU6050Add::lowpass MPU6050Add::getFilterBandwidth() {
	return (lowpass)readBits(MPU6050_CONFIG, (uint8_t)LOWPASS_MASK);
}

/// @brief Sets the bandwidth of the digital Low-Pass Filter
/// @param bandwidth The new bandwidth
void MPU6050Add::setFilterBandwidth(lowpass bandwidth) {
	writeBits(MPU6050_CONFIG, bandwidth, (uint8_t)LOWPASS_MASK);
}

/// @brief Gets bandwidth of the accelerometer high-pass filter
/// @return The current filter bandwidth
MPU6050Add::accel_highpass MPU6050Add::getHighPassFilter() {
	return (accel_highpass)readBits(MPU6050_ACCEL_CONFIG, (uint8_t)ACCEL_HIGHPASS_MASK);
}

/// @brief Sets the bandwidth of the accelerometer high-pass filter
/// @param bandwidth The new bandwidth
void MPU6050Add::setHighPassFilter(accel_highpass bandwidth) {
	writeBits(MPU6050_ACCEL_CONFIG, (uint8_t)bandwidth, (uint8_t)ACCEL_HIGHPASS_MASK);
}

/// @brief Sets the polarity of the interrupt pin when active
/// @param active_lo If true the pin will be low when an interrupt is active, if false the pin will be high when an interrupt is active
void MPU6050Add::setInterruptPinPolarity(bool active_low) {
	if (active_low) {
		writeBits(MPU6050_INT_PIN_CONFIG, 0x80, 0x7F);
	} else {
		writeBits(MPU6050_INT_PIN_CONFIG, 0, 0x7F);
	}
}

/// @brief Sets the latch behavior of the INT pin when active
/// @param held If true the pin will remain held until cleared, if false the pin will reset after a 50us pulse
void MPU6050Add::setInterruptPinLatch(bool held) {
	if (held) {
		writeBits(MPU6050_INT_PIN_CONFIG, 0x20, 0xDF);
	} else {
		writeBits(MPU6050_INT_PIN_CONFIG, 0, 0xDF);
	}
}

/// @brief  Sets the motion interrupt
/// @param  active If true motion interrupt will activate based on threshold and duration, if false motion interrupt will be disabled
void MPU6050Add::setMotionInterrupt(bool active) {
	if (active) {
		writeBits(MPU6050_INT_ENABLE, 0x40, 0xBF);
	} else {
		writeBits(MPU6050_INT_ENABLE, 0, 0xBF);
	}
}

/// @brief Gets motion interrupt status
/// @return True if interrupt has triggered
bool MPU6050Add::getMotionInterruptStatus(void) {
	return (bool)(readMPU6050(MPU6050_INT_STATUS) & 0x40);
}

/// @brief  Sets the motion detection threshold
/// @param  thr Threshold value to set
void MPU6050Add::setMotionDetectionThreshold(uint8_t thr) {
	writeMPU6050(MPU6050_MOT_THR, thr);
}

/// @brief  Sets the motion detection duration
/// @param  dur The duration in ms
void MPU6050Add::setMotionDetectionDuration(uint8_t dur) {
	writeMPU6050(MPU6050_MOT_DUR, dur);
}

///  @brief  Connects or disconnects the I2C master pins to the main I2C pins
/// @param  bypass If true the I2C Master pins are connected to the main I2C pins, bypassing the I2C Master functions of the sensor. If false the I2C Master pins are controlled by the I2C master functions of the sensor
void MPU6050Add::setI2CBypass(bool bypass) {
	if (bypass) {
		writeBits(MPU6050_INT_PIN_CONFIG, 0x02, 0xFD);
		writeBits(MPU6050_USER_CTRL, 0x20, 0xDF);
	} else {
		writeBits(MPU6050_INT_PIN_CONFIG, 0, 0xFD);
		writeBits(MPU6050_USER_CTRL, 0, 0xDF);
	}
}

/// @brief  Controls the sleep state of the sensor
/// @param enable If true the sensor is put into a low power stat and measurements are halted until sleep mode is deactivated, setting false wakes up the sensor from sleep mode
void MPU6050Add::enableSleep(bool enable) {
	if (enable) {
		writeBits(MPU6050_PWR_MGMT_1, 0x40, 0xBF);
	} else {
		writeBits(MPU6050_PWR_MGMT_1, 0, 0xBF);
	}
}

/// @brief Controls sensor's Cycle measurement mode
/// @param enable If true the sensor will take measurements at the rate set by calling setCycleRate, sleeping between measurements.
void MPU6050Add::enableCycle(bool enable) {
	if (enable) {
		writeBits(MPU6050_PWR_MGMT_1, 0x20, 0xDF);
	} else {
		writeBits(MPU6050_PWR_MGMT_1, 0, 0xDF);
	}
}

/// @brief Gets the frequency of measurements in Cycle mode
/// @return The current Cycle measurement frequency
MPU6050Add::cycle_rate MPU6050Add::getCycleRate(void) {
	return (cycle_rate)readBits(MPU6050_PWR_MGMT_2, (uint8_t)CYCLE_RATE_MASK);
}

/// @brief Sets the frequency of measurement in Cycle mode
/// @param rate The specifying the desired measurement rate
void MPU6050Add::setCycleRate(cycle_rate rate) {
	writeBits(MPU6050_PWR_MGMT_2, rate, (uint8_t)CYCLE_RATE_MASK);
}

/// @brief Sets standby mode for each of the gyroscope axes
/// @param xAxisStandby If true the gyroscope stops sensing in the X-axis
/// @param yAxisStandby If true the gyroscope stops sensing in the Y-axis
/// @param zAxisStandby If true the gyroscope stops sensing in the Z-axis
void MPU6050Add::setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby) {
	writeBits(MPU6050_PWR_MGMT_2, xAxisStandby << 2 | yAxisStandby << 1 | zAxisStandby, 0xF8);
}

/// @brief Sets standby mode for each of the accelerometer axes
/// @param xAxisStandby If true the accelerometer stops sensing in the X-axis
/// @param yAxisStandby If true the accelerometer stops sensing in the Y-axis
/// @param zAxisStandby If true the accelerometer stops sensing in the Z-axis
void MPU6050Add::setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby) {
	writeBits(MPU6050_PWR_MGMT_2, xAxisStandby << 5 | yAxisStandby << 4 | zAxisStandby << 3, 0xC7);
}

/// @brief Sets disable mode for thermometer sensor
///  @param enable If true the temperature sensor will stop taking measurements
void MPU6050Add::setTemperatureStandby(bool enable) {
	if (enable) {
		writeBits(MPU6050_PWR_MGMT_1, 0x08, 0xF7);
	} else {
		writeBits(MPU6050_PWR_MGMT_1, 0, 0xF7);
	}
}

/// @brief Sets the offset (calibration) values for the gyroscope
/// @param x The X-axis offset
/// @param y The Y-axis offset
/// @param z The Z-axis offset
void MPU6050Add::setGyroOffsets(float x, float y, float z){
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

/// @brief Calculates and sets the gyroscope offset (calibration) values
/// @param delayBefore Milliseconds to delay before starting calibration measurement
/// @param delayAfter Milliseconds to delay after finishing calibration measurement
/// @param console True will output results to serial
void MPU6050Add::calcGyroOffsets(uint16_t delayBefore, uint16_t delayAfter, bool console){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

	delay(delayBefore);
	if(console){
		Serial.println();
		Serial.println("========================================");
		Serial.println("Calculating gyro offsets");
		Serial.print("DO NOT MOVE MPU6050");
	}
	for(int i = 0; i < 3000; i++){
		if(console && i % 1000 == 0){
			Serial.print(".");
		}
		wire->beginTransmission(_address);
		wire->write(MPU6050_GYRO_OUT);
		wire->endTransmission(false);
		wire->requestFrom((int)_address, 6);

		rx = wire->read() << 8 | wire->read();
		ry = wire->read() << 8 | wire->read();
		rz = wire->read() << 8 | wire->read();

		x += ((float)rx) / 65.5;
		y += ((float)ry) / 65.5;
		z += ((float)rz) / 65.5;
	}
	gyroXoffset = x / 3000;
	gyroYoffset = y / 3000;
	gyroZoffset = z / 3000;

	if(console){
		Serial.println();
		Serial.println("Done!");
		Serial.print("X : ");Serial.println(gyroXoffset);
		Serial.print("Y : ");Serial.println(gyroYoffset);
		Serial.print("Z : ");Serial.println(gyroZoffset);
		Serial.print("========================================");
		delay(delayAfter);
	}
}

/// @brief Updates all sensor readings
void MPU6050Add::update(){
	wire->beginTransmission(_address);
	wire->write(MPU6050_ACCEL_OUT);
	wire->endTransmission(false);
	wire->requestFrom(_address, 14);

	rawAccX = wire->read() << 8 | wire->read();
	rawAccY = wire->read() << 8 | wire->read();
	rawAccZ = wire->read() << 8 | wire->read();
	rawTemp = wire->read() << 8 | wire->read();
	rawGyroX = wire->read() << 8 | wire->read();
	rawGyroY = wire->read() << 8 | wire->read();
	rawGyroZ = wire->read() << 8 | wire->read();

	float interval = (millis() - preInterval) * 0.001;

	temp = (rawTemp / 340.0) + 36.53;

	accel_range accel_range = getAccelerometerRange();
	preInterval = millis();

	float accel_scale = 1;
	switch (accel_range) {
		case MPU6050_RANGE_16_G:
			accel_scale = 2048;
			break;
		case MPU6050_RANGE_8_G:
			accel_scale = 4096;
			break;
		case MPU6050_RANGE_4_G:
			accel_scale = 8192;
			break;
		case MPU6050_RANGE_2_G:
			accel_scale = 16384;
			break;
	};

	// setup range dependant scaling
	accX = ((float)rawAccX) / accel_scale;
	accY = ((float)rawAccY) / accel_scale;
	accZ = ((float)rawAccZ) / accel_scale;

	angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
	angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;

	gyro_range gyro_range = getGyroRange();

	float gyro_scale = 1;
	switch (accel_range) {
		case MPU6050_RANGE_250_DEG:
			gyro_scale = 131;
			break;
		case MPU6050_RANGE_500_DEG:
			gyro_scale = 65.5;
			break;
		case MPU6050_RANGE_1000_DEG:
			gyro_scale = 32.8;
			break;
		case MPU6050_RANGE_2000_DEG:
			gyro_scale = 16.4;
			break;
	};

	gyroX = ((float)rawGyroX) / gyro_scale;
	gyroY = ((float)rawGyroY) / gyro_scale;
	gyroZ = ((float)rawGyroZ) / gyro_scale;

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;

	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
	angleZ = angleGyroZ;
}

/// @brief Writes specific bits to a register using a mask
/// @param reg The register address
/// @param data The data to write
/// @param mask The mask that will be used to only write the bits of interest
void MPU6050Add::writeBits(Registers reg, uint8_t data, uint8_t mask) {
	uint8_t current = readMPU6050(reg);
	writeMPU6050(reg, (mask & current) | data);
}

/// @brief Reads specific bits to from register using a mask
/// @param reg The register address
/// @param mask The mask that will be used to filter for the bits of interest
uint8_t MPU6050Add::readBits(Registers reg, uint8_t mask) {
	return readMPU6050(reg) & ~mask;
}

/// @brief Writes a byte to a register
/// @param reg The register address
/// @param data The data to write
void MPU6050Add::writeMPU6050(Registers reg, uint8_t data){
	wire->beginTransmission(_address);
	wire->write(reg);
	wire->write(data);
	wire->endTransmission();
}

/// @brief Reads a byte from the register
/// @param reg The register address to read
/// @return The byte read from the register
uint8_t MPU6050Add::readMPU6050(Registers reg) {
	wire->beginTransmission(_address);
	wire->write(reg);
	wire->endTransmission(true);
	wire->requestFrom(_address, 1);
	byte data =  wire->read();
	return data;
}