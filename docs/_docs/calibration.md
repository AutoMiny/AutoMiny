---
title: "Calibration"
permalink: /docs/calibration/
excerpt: "IMU calibration for MPU 6050"
toc: true
---

## IMU
AutoMiny comes with a BNO055 USB Stick IMU and an Invensense MPU6050. By default, the BNO055 will be used. The MPU6050 can be used by flashing ```main-mpu6050.cpp``` onto the Arduino. When using the MPU6050 the BNO055 should be disconnected since both devices publish their data on ```/sensors/imu```.

### IMU calibration (BNO055)
The BNO055 sensor calibrates itself automatically. The accelerometer shouldn't need any calibration since it is already calibrated by factory. The gyroscope is autocalibrated if the device is not moving for 8 seconds, which is usually the case while booting the car.

### IMU calibration (MPU6050)
If your IMU readings are unstable or weird the IMU probably needs a recalibration. Place the car on a stable flat surface and power it up.  The calibration program finds the offsets for the accelerometer and gyroscope to keep it in the zero position. During calibration the car needs to be kept stable and must not be moved.

```bash
rosservice call /sensors/arduino/calibrate_imu
```

The calibration will now start. The LEDs will light up and show progress during calibration. The calibration process takes anywhere from 5 to 10 minutes. Once calibration has finished all the LEDs will light up for a short amount of time and afterwards the middle LEDs should indicate the voltage status again. The calibration values are saved in the Arduino's EEPROM memory.