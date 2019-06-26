## Arduino

### Serial protocol
We use a binary protocol for the communication between the Intel NUC and the Arduino Nano. The protocol uses [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) encoding for packet framing. Each packet consists of a flag indicating the message type and data. The data length is defined for each message type differently.

On the Arduino side the communication is being handled through the PacketSerial library and on the Intel NUC side through the ROS package arduino_communication. The communication is performed at 115200 baud.

### Message types
We defined the following message types:

```
enum class MessageType :uint8_t {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    SPEED_CMD,
    STEERING_CMD,
    LED_CMD,
    STEERING_ANGLE,
    TICKS,
    IMU,
    VOLTAGE,
    HEARTBEAT,
    IMU_CALIBRATION
};
```

| Type            | Info                                                                                                                 |
|:----------------|:---------------------------------------------------------------------------------------------------------------------|
| DEBUG           | Logging information with severity DEBUG. Will be broadcast as ROS_DEBUG in ROS                                       |
| INFO            | Logging information with severity INFO. Will be broadcast as ROS_DEBUG in ROS                                        |
| WARN            | Logging information with severity WARN. Will be broadcast as ROS_DEBUG in ROS                                        |
| ERROR           | Logging information with severity ERROR. Will be broadcast as ROS_DEBUG in ROS                                       |
| SPEED_CMD       | int16_t Value between -1000 and 1000 defining the PWM signal to send to the speed motor                              |
| STEERING_CMD    | uint16_t defining the PWM signal to send to the steering servo motor                                                 |
| LED_CMD         | char* LED command (turn left, turn right, etc.)                                                                      |
| STEERING_ANGLE  | uint16_t Steering angle feedback between 0 and 1024                                                                  |
| TICKS           | uint8_t Captured encoder ticks from the speed motor between one time interval of 0.01s                               |
| IMU             | uint8_t\[23\] MPU6050 data consisting of orientation, acceleration, and rotational velocity                          |
| VOLTAGE         | Averaged battery voltage from readings in the last 3 seconds.                                                        |
| HEARTBEAT       | Heartbeat command that must be send at least every 30ms else the motor will stop                                     |
| IMU_CALIBRATION | Perform MPU6050 calibration during this time no input will be handled and the car must be stable (takes > 5 minutes) |


### ROS Node
The ROS node `arduino_communication` manages the communication with the arduino. All of the above message types are mapped to subscribers and publishers in this node. The heartbeat is send to the Arduino at 100hz.

#### Publishers

| Topic            | Type                           | Description                                                     |
|:-----------------|:-------------------------------|:----------------------------------------------------------------|
| ~voltage         | autominy_msgs/Voltage          | Battery voltage \[V\]                                           |
| ~ticks           | autominy_msgs/Ticks            | Number of ticks from speed motor in 0.01s                       |
| ~steering_angle  | autominy_msgs/SteeringFeedback | Steering angle feedback voltage from steering servo motor \[V\] |
| ~imu             | sensor_msgs/IMU                | IMU data from MPU6050 (if enabled)                              |
| ~imu/temperature | sensor_msgs/Temperature        | IMU temperature from MPU6050 (if enabled)                       |

#### Subscribers

| Topic     | Type                             | Description                               |
|:----------|:---------------------------------|:------------------------------------------|
| ~steering | autominy_msgs/SteeringPWMCommand | Steering motor command in PWM Signal duty |
| ~speed    | autominy_msgs/SpeedPWMCommand    | Speed motor command in PWM Signal duty    |

#### Services

| Topic          | Type           | Description                                                                                          |
|:---------------|:---------------|:-----------------------------------------------------------------------------------------------------|
| ~calibrate_imu | std_msgs/Empty | Calibrates the MPU6050 (if enabled). Takes ~5 minutes and the car must be stable during calibration. |
