---
title: "Emergency Stop"
permalink: /docs/emergency-stop/
excerpt: "Emergency stop maneuvers"
toc: true
---
The emergency stop module monitors obstacles that are in the driving direction and can stop the car if an crash is imminent. The module uses the LiDAR and checks if there are any obstacles too close. The module monitors the input to the actuators and in cases of an crash intercepts the communication with the actuators. In such a scenario the emergency stop module immediately stops the car and allows for no further movement.

### Configuration
Configuration of the `emergency_stop` package is done through rqt dynamic reconfigure. It offers the following settings.

| Name                          | Default | Information                                                          |
|:------------------------------|:--------|:---------------------------------------------------------------------|
| angle_front                   | 0.7     | Angle in front of the car to monitor                                 |
| angle_back                    | 0.7     | Angle in back of the car to monitor                                  |
| brake_distance                | 0.45    | Constant brake distance                                              |
| brake_distance_based_on_speed | False   | Calculate brake distance based on speed                              |
| reverse_minimum_distance      | 0.28    | Minimum distance of obstacles to be considered while reverse driving |
| forward_minimum_distance      | 0.07    | Minimum distance of obstacles to be considered while forward driving |
| negative_acceleration         | 4.0     | Acceleration used in speed based braking                             |

### Subscribers

| Topic           | Type                          | Information                    |
|:----------------|:------------------------------|:-------------------------------|
| ~wanted_speed   | autominy_msgs/SpeedPWMCommand | Speed command sent to actuator |
| ~carstate/speed | autominy_msgs/Speed           | Current car speed              |
| ~scan           | sensor_msgs/LaserScan         | Laser scan from LiDAR          |

### Publishers

| Topic  | Type                          | Information                                                                 |
|:-------|:------------------------------|:----------------------------------------------------------------------------|
| ~speed | autominy_msgs/SpeedPWMCommand | Requested speed if there are no obstacles or 0 in case of an emergency stop |
