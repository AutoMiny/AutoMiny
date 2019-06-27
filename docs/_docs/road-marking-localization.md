---
title: "Road Marking Localization"
permalink: /docs/road-marking-localization/
excerpt: "Road Marking Localization"
toc: true
road_gallery:
  - image_path: /assets/images/road_marking_localization.png
    url: /assets/images/road_marking_localization.png
    alt: "Alignment during road marking localization"

---

### Algorithm
The road marking localization node uses ICP (iterative closest point) algorithm to align road markings seen by the camera to a given map. Given an initial position estimate it calculates the correction for the car's current position. We assume that road markings are white. We find the road markings by thresholding the camera's infrared image. For each white pixel we then find the corresponding point (x, y, z) in the pointcloud. We then project the points into the map frame using the current estimated position. Next, we apply a box filter onto the point cloud to cut off any points that are too far away (due to noise) or are not near the ground plane (z = 0). In the next step we randomly select a low amount of points out of the previous filtered point cloud. Random sampling cuts down the point cloud size significantly while still keeping the overall structure of the road markings intact.

The predefined map is taken from an occupancy grid that is being broadcast by the `map_publisher` node through ROS. The map exists as an image file on the car and represents the ground plane. The white pixels inside the map are converted into a pointcloud also known as the reference point cloud.

Now that we have both the current road markings seen by the camera (projected into the map frame at the current proposed position) and also the reference pointcloud we can use ICP to calculate the transformation matrix between both pointclouds that aligns them. ICP works by creating a kd-tree for the reference point cloud. kd-trees offer a cheap nearest neighbour lookup with computation cost of `O(log(n))`. The kd-tree is only constructed once because our reference map point cloud does not change over time. Now, for every point in the proposed point cloud we find the nearest neighbour in the reference frame and get the squared distance between them. The sum of all distances is the cost of the current transform and the goal is to minimize this cost. We apply a Levenberg-Marquardt optimization to minimize this cost using a transformation. For the translation we only allow changes in the 2D space which is a translation of x and y and a rotation around the z-axis.

{% include gallery id="road_gallery" caption="Road marking localization alignment" %}

### Configuration
Configuration of the road marking localization package is done through rqt dynamic reconfigure. It offers the following settings.

| Name                                   | Default | Information                                                            |
|:---------------------------------------|:--------|:-----------------------------------------------------------------------|
| blur_kernel_size                       | 1       | Kernel size for gaussian blur in infrared image before thresholding    |
| crop_top_pixels                        | 0       | Amount of pixels to crop from the top of the infrared image            |
| x_box                                  | 2.0     | Size of bounding box x around the car for cropping                     |
| y_box                                  | 2.0     | Size of bounding box y around the car for cropping                     |
| minimum_z                              | -0.02   | Minimum height of points                                               |
| maximum_z                              | 0.02    | Maximum height of points                                               |
| threshold                              | 160     | Minimum amount of intensity for a pixel to be detected as road marking |
| icp_max_iterations                     | 5       | Number of iterations for the ICP algorithm                             |
| icp_RANSAC_outlier_rejection_threshold | 0.0     | RANSAC outlier rejection threshold                                     |
| icp_RANSAC_iterations                  | 0.0     | Number of RANSAC iterations                                            |
| icp_max_correspondence_distance        | 0.10    | Maximum distance to a point of a map to be considered during alignment |
| icp_sample_size                        | 750     | Random sampling size                                                   |
| minimum_points                         | 250     | Minimum number of points needed for alignment                          |
| maximum_x_correction                   | 0.3     | Maximum amount of acceptable correction in x                           |
| maximum_y_correction                   | 0.3     | Maximum amount of acceptable correction in y                           |
| maximum_yaw_correction                 | 0.5     | Maximum amount of acceptable correction in yaw                         |
| debug                                  | True    | Publish debug topics like images and intermediate pointclouds          |

### Subscribers

| Topic                         | Type                        | Information                                                              |
|:------------------------------|:----------------------------|:-------------------------------------------------------------------------|
| ~map                          | nav_msgs/OccupancyGrid      | Road marking map. A value > 0 means that there is a road marking.        |
| ~initialpose                  | nav_msgs/PoseWithCovariance | Initial position. Publishing on this topic resets the internal position. |
| ~camera/infra1/image_rect_raw | sensor_msgs/Image           | Infrared image                                                           |
| ~camera/infra1/camera_info    | sensor_msgs/CameraInfo      | Infrared camera parameters                                               |
| ~camera/depth/image_rect_raw  | sensor_msgs/Image           | Depth image                                                              |
| ~camera/depth/camera_info     | sensor_msgs/CameraInfo      | Depth image parameters                                                   |

### Publishers

| Topic                  | Type                       | Information                                  |
|:-----------------------|:---------------------------|:---------------------------------------------|
| ~corrected_odom        | nav_msgs/Odometry          | Result from road marking localization        |
| ~threshold             | sensor_msgs/Image          | Infrared image after thresholding            |
| ~raw_pcl               | sensor_msgs/PointCloud2    | Raw pointcloud from the thresholded pixels   |
| ~cropped_pcl           | sensor_msgs/PointCloud2    | Pointcloud after cropping too distant points |
| ~random_sampled_pcl    | sensor_msgs/PointCloud2    | Pointcloud after random sampling             |
| ~aligned_pcl           | sensor_msgs/PointCloud2    | Pointcloud after alignment to map            |
| ~map_pcl               | sensor_msgs/PointCloud2    | Map converted from occupancy grid to map     |
| ~transformation_matrix | std_msgs/Float64MultiArray | Transformation matrix from alignment         |
