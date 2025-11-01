#include <Arduino.h>
#include <Wire.h>
#include <MPU6050Add.h>

MPU6050Add mpu6050;

long timer = 0;

void setup() {
	Serial.begin(115200);
	delay(2000);
	Wire.begin();
	if (!mpu6050.begin()) {
		Serial.println("Could not start MPU");
		while(1);
	}

	Serial.print("Accelerometer range set to: ");
	switch (mpu6050.getAccelerometerRange()) {
	case MPU6050Add::accel_range::MPU6050_RANGE_2_G:
		Serial.println("+-2G");
		break;
	case MPU6050Add::accel_range::MPU6050_RANGE_4_G:
		Serial.println("+-4G");
		break;
	case MPU6050Add::accel_range::MPU6050_RANGE_8_G:
		Serial.println("+-8G");
		break;
	case MPU6050Add::accel_range::MPU6050_RANGE_16_G:
		Serial.println("+-16G");
		break;
	}

	Serial.print("Gyro range set to: ");
	switch (mpu6050.getGyroRange()) {
	case MPU6050Add::gyro_range::MPU6050_RANGE_250_DEG:
		Serial.println("+- 250 deg/s");
		break;
	case MPU6050Add::gyro_range::MPU6050_RANGE_500_DEG:
		Serial.println("+- 500 deg/s");
		break;
	case MPU6050Add::gyro_range::MPU6050_RANGE_1000_DEG:
		Serial.println("+- 1000 deg/s");
		break;
	case MPU6050Add::gyro_range::MPU6050_RANGE_2000_DEG:
		Serial.println("+- 2000 deg/s");
		break;
	}

	Serial.print("Filter bandwidth set to: ");
	switch (mpu6050.getFilterBandwidth()) {
	case MPU6050Add::lowpass::MPU6050_BAND_260_HZ:
		Serial.println("260 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_184_HZ:
		Serial.println("184 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_94_HZ:
		Serial.println("94 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_44_HZ:
		Serial.println("44 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_21_HZ:
		Serial.println("21 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_10_HZ:
		Serial.println("10 Hz");
		break;
	case MPU6050Add::lowpass::MPU6050_BAND_5_HZ:
		Serial.println("5 Hz");
		break;
	}
	mpu6050.calcGyroOffsets(1000, 2000, true);
	mpu6050.resetAngles();
}

void loop() {	
	mpu6050.update();
	if(millis() - timer > 1000) {	
		Serial.println("=======================================================");
		Serial.print("temp : ");Serial.println(mpu6050.getTemperature());
		Serial.print("accX : ");Serial.print(mpu6050.getAccX());
		Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
		Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

		Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
		Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
		Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());

		Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
		Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());

		Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
		Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
		Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
		
		Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
		Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
		Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
		Serial.println("=======================================================\n");
		timer = millis();		
	}
}