## IMU
If your IMU readings are unstable or weird the IMU probably needs a recalibration. Place the car on a stable flat surface and power it up.  The calibration program finds the offsets for the accelerometer and gyroscope to keep it in the zero position. During calibration the car needs to be kept stable and must not be moved.

```bash
rosservice call /sensors/arduino/calibrate_imu
```

The calibration will now start. The LEDs will light up and show progress during calibration. The calibration process takes anywhere from 5 to 10 minutes. Once calibration has finished all the LEDs will light up for a short amount of time and afterwards the middle LEDs should indicate the voltage status again. The calibration values are saved in the Arduino's EEPROM memory.