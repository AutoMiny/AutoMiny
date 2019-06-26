---
title: "Realsense"
permalink: /docs/realsense/
excerpt: "Realsense"
toc: true
---
We support up to two realsense D435 cameras.
### Configuration
We configure the realsense camera with the following parameters:

| Name                        | Value | Explanation                                                                                                                                                |
|:----------------------------|:------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| align_depth                 | false | Color image alignment is disabled by default due to performance reasons. The depth image is aligned onto the infrared camera 1 image in our configuration. |
| depth_registered_processing | false | Do not register depth image onto other cameras                                                                                                             |
| enable_color                | true  | Enable color camera                                                                                                                                        |
| enable_infra1               | true  | Enable infrared camera 1                                                                                                                                   |
| enable_infra2               | false | Infrared camera 2 is not enabled because the image is very similar to infrared camera 1                                                                    |
| depth_fps                   | 15    | Depth images are sent at 15 hz. Use the `rs-enumerate-devices` tool to find valid resolution/framerate pairs.                                              |
| color_fps                   | 15    | Color images are sent at 15 hz. Use the `rs-enumerate-devices` tool to find valid resolution/framerate pairs.                                              |
| infra_fps                   | 15    | Infrared images are sent at 15 hz. Use the `rs-enumerate-devices` tool to find valid resolution/framerate pairs.                                           |

The default size of the images is 640x480. The color camera supports up to 1920x1080@30Hz. Depth and infrared is supported up to 1280x800@30Hz. Use the `rs-enumerate-devices` tool on the car to find valid combinations of framerate and resolution.

### Two camera mode
In two camera mode we need the serial numbers of the cameras to distinguish between the front and rear camera. You can retrieve the different serial numbers using the `rs-enumerate-devices` tool. Two camera mode is configured using the following environment variables. These can be added to `~/ros-config.sh`

| Name                      | Value                      |
|:--------------------------|:---------------------------|
| ROS_CAMERA_SERIAL_NO      | Front camera serial number |
| ROS_REAR_CAMERA_SERIAL_NO | Rear camera serial number  |

### Publishers

| Topic                   | Type                   | Information                   |
|:------------------------|:-----------------------|:------------------------------|
| ~color/camera_info      | sensor_msgs/CameraInfo | Color camera parameter        |
| ~color/image_raw        | sensor_msgs/Image      | Color image                   |
| ~color/image_rect_color | sensor_msgs/Image      | Color image rectified         |
| ~depth/camera_info      | sensor_msgs/CameraInfo | Depth image camera parameters |
| ~depth/image_rect_raw   | sensor_msgs/Image      | Depth image rectified         |
| ~infra1/camera_info     | sensor_msgs/CameraInfo | Infrared camera 1 parameters  |
| ~infra1/image_rect_raw  | sensor_msgs/Image      | Infrared camera 1 image       |

For more information on the realsense driver refer to the [realsense documentation](https://github.com/IntelRealSense/realsense-ros).
