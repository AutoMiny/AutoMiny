---
title: "LiDAR pose estimation"
permalink: /docs/lidar-pose-estimation/
excerpt: "LiDAR pose estimation"
toc: true
---
The LiDAR pose estimation works by finding the two poles mounted in the rear of the car for the camera. The position of these two poles is fixed and known. The pose estimation collects data samples from multiple scans, clusters them and extracts the largest two ones. The resulting clusters are then checked for plausibilty and the LiDAR pose in 2D is calculated. Since the poles are black it is important to put some white tape to increase the visibility of the poles for the LiDAR.

### Configuration
Configuration of the `lidar_pose_estimation` package is done through rqt dynamic reconfigure. It offers the following settings:

| Name                             | Default | Information                                          |
|:---------------------------------|:--------|:-----------------------------------------------------|
| max_dist                         | 0.25    | Maximum distance of points to be used for clustering |
| execution_frequency              | 1.0     | Execution frequency in seconds                       |
| cluster_tolerance                | 0.02    | Cluster tolerance                                    |
| min_cluster_size                 | 10      | Minimum amount of points needed in a cluster         |
| max_cluster_size                 | 100     | Maximum amount of points needed in a cluster         |
| max_reference_distance_deviation | 0.01    | Maximum distance deviation for clustering            |
| roll                             | 0.0     | Constant roll                                        |
| pitch                            | 0.0     | Constant pitch                                       |
| z                                | 0.07    | Constant height                                      |
| p_ref_1_x                        | -0.02   | Pole 1 x coordinate in car coordinate system         |
| p_ref_1_y                        | 0.0525  | Pole 1 y coordinate in car coordinate system         |
| p_ref_2_x                        | -0.02   | Pole 2 x coordinate in car coordinate system         |
| p_ref_2_y                        | 0.0525  | Pole 2 y coordinate in car coordinate system         |

### Subscribers

| Topic           | Type                          | Information                    |
|:----------------|:------------------------------|:-------------------------------|
| ~scan           | sensor_msgs/LaserScan         | Laser scan from LiDAR          |

### Publishers

| Topic  | Type                    | Information                         |
|:-------|:------------------------|:------------------------------------|
| ~poles | sensor_msgs/PointCloud2 | Detected poles from pose estimation |

### Transforms

| From         | To              | Information                                                              |
|:-------------|:----------------|:-------------------------------------------------------------------------|
| sensor_board | lidar_base_link | Transformation from sensor_board to lidar_base_link from pose estimation |
