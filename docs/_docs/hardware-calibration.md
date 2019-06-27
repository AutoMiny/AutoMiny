---
title: "Hardware calibration"
permalink: /docs/hardware-calibration/
excerpt: "Hardware calibration"
toc: true
---
The `hardware_calibration` package transforms between the hardware signals and their mappings as SI units. Readings from the arduino are reported as voltages or signals and need to be mapped into SI units. In case of actuators the arduino takes a voltage to apply to the actuators. This module transforms the output into the right voltages.

### Steering
The module reads the left and right steering angle values and their corresponding commands to convert from steering angle feedback voltages and to steering actuator voltages. A linear relationship between the voltage and the output is assumed. To reduce noise on the steering feedback a moving average filter is applied with configurable size.

### Speed
The module reads the reverse and forward speed values and their corresponding commands to convert an input speed to a motor signal voltage. The relationship between input in m/s and output in voltage is modeled as a cubic function. The encoder tick feedback is translated into speed by applying a moving average filter and then multiplying the ticks with an configurable tick to meter value.


### Configuration
Configuration of the `hardware_calibration` package is done through rqt dynamic reconfigure. It offers the following settings:

| Name                           | Default | Information                                                                  |
|:-------------------------------|:--------|:-----------------------------------------------------------------------------|
| minimum_steering_feedback      | 192     | Minimum steering feedback value reported by Arduino                          |
| maximum_steering_feedback      | 420     | Maximum steering feedback value reported by Arduino                          |
| minimum_steering_radians       | 0.512   | Fully left steering angle                                                    |
| maximum_steering_radians       | -0.498  | Fully right steering angle                                                   |
| minimum_steering_pwm           | 950     | Fully left steering motor PWM pulse                                          |
| maximum_steering_pwm           | 2150    | Fully right steering motor PWM pulse                                         |
| minimum_speed_pwm              | -1000   | Full speed reverse speed motor PWM pulse                                     |
| maximum_speed_pwm              | 1000    | Full speed forward speed motor PWM pulse                                     |
| number_of_ticks_filter         | 20      | Moving average filter message amount over the motor ticks to reduce noise    |
| number_of_steering_msgs_filter | 10      | Moving average filter message amount over the steering angle to reduce noise |

### Subscribers

| Topic                          | Type                                    | Information                          |
|:-------------------------------|:----------------------------------------|:-------------------------------------|
| ~actuators/speed               | autominy_msgs/SpeedCommand              | Speed in m/s                         |
| ~actuators/steering            | autominy_msgs/SteeringCommand           | Steering in rad                      |
| ~actuators/speed_normalized    | autominy_msgs/NormalizedSpeedCommand    | Normalized speed from -1.0 to 1.0    |
| ~actuators/steering_normalized | autominy_msgs/NormalizedSteeringCommand | Normalized steering from -1.0 to 1.0 |
| ~arduino/ticks                 | autominy_msgs/Ticks                     | Ticks from speed motor               |
| ~arduino/steering_angle        | autominy_msgs/SteeringFeedback          | Steering feedback voltage            |

### Publishers

| Topic                      | Type                             | Information                        |
|:---------------------------|:---------------------------------|:-----------------------------------|
| ~arduino/steering          | autominy_msgs/SteeringPWMCommand | Steering as PWM pulse value        |
| ~arduino/speed             | autominy_msgs/SpeedPWMCommand    | Speed as PWM pulse value           |
| ~carstate/calibrated_speed | autominy_msgs/Speed              | Speed in m/s calculated from ticks |
| ~carstate/steering_angle   | autominy_msgs/SteeringAngle      | Steering angle in rad              |