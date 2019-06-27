---
title: "Odometry"
permalink: /docs/odometry/
excerpt: "Wheel odometry data"
toc: true
---

The car uses the IMU, steering angle and speed motor encoder ticks to roughly estimate how the car moved. The odometry uses the Ackermann Steering model. This estimation however is not very accurate as it is subject to slip and IMU drift. Therefore localization cannot rely on wheel odometry alone. Although wheel odometry has its downsides, it is available at a high rate and can thus be used to interpolate localization using a filter.

### Configuration

|               |       |                                                                             |
|:--------------|:------|:----------------------------------------------------------------------------|
| initial_x     | 0.0   | Initial x coordinate                                                        |
| initial_y     | 0.0   | Initial y coordinate                                                        |
| initial_yaw   | 0.0   | Initial yaw                                                                 |
| bicycle_model | true  | Whether to use the bicycle model or fallback to simple model using IMU only |
| publish_tf    | false | Publish transform from map -> base_link                                     |

### Subscribers

| Topic     | Type                   | Information                                      |
|:----------|:-----------------------|:-------------------------------------------------|
| ~speed    | autominy_msgs/Speed    | Speed information                                |
| ~steering | autominy_msgs/Steering | Steering angle, only used in bicycle model mode  |
| ~imu      | sensor_msgs/IMU        | IMU data. Uses the relative orientation from IMU |

### Publishers

| Topic | Type              | Information         |
|:------|:------------------|:--------------------|
| ~odom | nav_msgs/Odometry | Calculated odometry |
