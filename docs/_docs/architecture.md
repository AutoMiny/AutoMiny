---
title: "Architecture"
permalink: /docs/architecture/
excerpt: "Software architecture and modules"
toc: true
architecture_gallery:
  - image_path: /assets/images/architecture.png
    url: /assets/images/architecture.png
    alt: "Architecture overview"
---

Below you find an general overview of the architecture of our software stack. For an in-depth view of the separate modules refer to the online documentation or use the rqt node graph provided by ROS.

{% include gallery id="architecture_gallery" caption="Architecture" %}

AutoMiny is structured into separate layers which are responsible for different tasks. This architecture allows for an easy to understand structure of the AutoMiny software stack.
### Sensors
Sensor modules publish raw data from the sensors. In Autominy sensors for steering angle, encoder ticks, stereo camera and IMU are provided.
### Virtual sensors
Virtual sensors process raw sensor data or other virtual sensors. In AutoMiny virtual sensors such as  lidar pose estimation, camera pose estimation, obstacle detection, odometry, road marking localization, kalman filter, hardware calibration, pointcloud  and fake GPS  are included.
### Communication
AutoMiny automatically connects to other cars in the same network and broadcasts the car’s position. It uses a mesh network using the multimaster_fkie package to communicate with other cars. It can also receive localization from the fake GPS through this module. It is also possible to send additional messages through the mesh network.
### Navigation
For autonomous driving we offer an vector field controller with obstacle avoidance. AutoMiny also can also be controlled with an JoyStick through an remote control package.
### Autonomics
These modules run in the background and can take over control if the car is in danger. AutoMiny includes a module to do emergency stopping using the LiDAR data.
### Virtual actuators
Virtual actuators give a more high level interaction with the actuators. For instance AutoMiny provides modules to map from steering angle radian values to the corresponding control signals for the motor. Virtual actuators are provided for normalized steering angle, steering angle, normalized speed and speed.
### Actuators
These modules interact with the actuators. These modules are on the hardware level and in case of AutoMiny this means the actuators expect control signals or voltages to send to the actuators.