# MPU6050Add
Advanced Arduino library for easy communicating with the MPU6050.

While the [Adafruit library](https://github.com/adafruit/Adafruit_MPU6050/) is excellent, especially if using more than one of their sensors, it has a fair bit of overhead. The library by [tockn](https://github.com/tockn) is much lighter weight, by lacks many features and is not very flexible. This library aims to combine the best of both of these libraries. This is a stand alone library with access to all of the MPU6050 features as provided by the Adafruit library.

> [!CAUTION]
> This library is very much in progress; not all of the registers and settings for the MPU6050 have been tested.

## Usage
You can see an [example sketch](examples/GetAllData/GetAllData.ino).  
  
If you want to get data from the MPU6050, you must execute `update()` method before using the appropriate `get` methods. `update()` will get all the data from MPU6050, and calculate the total angle moved by accelerometer, gyroscope and complementary filter.  

### Complementary Filter
The `update()` method calculates the total angle by accelerometer and gyroscope using complementary filter. Those two coefficients are determined by constructor. The default coefficient of accelerometer is 0.02, gyroscope is 0.98.

`filtered_angle = (0.02 * accel) + (0.98 * gyro)`  

Calling the `resetAngles()` method will reset the total calculated angle back to 0, i.e. it will start the measurement over.

#### Example
If you want to set 0.1 to the accelerometer coefficient and 0.9 to the gyroscope coefficient, use this constructor:

`MPU6050 mpu6050(0.1, 0.9);` 

### Auto calibration
If you use `calcGyroOffsets()` it will calculate the calibration for the gyroscope, and the values will be automatically saved and applied.  
> [!WARNING]
> DO NOT MOVE MPU6050 during calculations!

```cpp
#include <MPU6050Add.h>

MPU6050 mpu6050;

void setup(){
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();
}

```

If you use `calcGyroOffsets(true)`, you can see the state of the calibration in the serial monitor.  

```cpp
#include <MPU6050Add.h>

MPU6050 mpu6050;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
```
Serial monitor:
```
Calculate gyro offsets
DO NOT MOVE MPU6050.....
Done!
X : 1.45
Y : 1.23
Z : -1.32
```  
  
If you know the offsets for the gyroscope, you can set them using `setGyroOffsets`, then you don't need to execute `calcGyroOffsets()`, so you can launch the program more quickly.
#### Example
```cpp
#include <MPU6050Add>

MPU6050 mpu6050;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(1.45, 1.23, -1.32);
}
```

## Author

Contributing author:
[ShVerni](https://github.com/ShVerni)

Original authors:
[tockn](https://github.com/tockn)
[adafruit](https://github.com/adafruit/)
