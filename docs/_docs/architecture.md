---
title: "Architecture"
permalink: /docs/architecture/
excerpt: "Software architecture and modules"
toc: true
architecture_gallery:
  - image_path: /assets/images/architecture.png
    url: /assets/images/architecture.png
    alt: "Architecture"
---

Autominy is divided into separate layers.

### Sensors
Sensor modules publish raw data from the sensors. In Autominy sensors for steering, speed, stereo camera and IMU are provided.
### Virtual sensors
Virtual sensors process raw sensor data or other virtual sensors. In AutoMiny virtual sensors for lidar pose estimation, camera pose estimation, obstacle detection, odometry, road marking localization, kalman filter and hardware calibration are included.
### Autonomics
These modules run in the background and can take over control if the car is in danger. AutoMiny includes a module to do emergency stopping using the LiDAR data.

### Virtual actuators
Virtual actuators give a more high level interaction with the actuators. For instance AutoMiny provides modules to map from steering angle radian values to the corresponding control signals for the motor. Virtual actuators are provided for normalized steering angle, steering angle, normalized speed and speed.

### Actuators
These modules interact with the actuators. These modules are on the hardware level and in case of AutoMiny this means the actuators expect control signals or voltages to send to the actuators.

{% include gallery id="architecture_gallery" caption="Architecture" %}
