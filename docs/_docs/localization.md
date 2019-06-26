## Localization
Localization for the car is done using a combination of wheel odometry and road marking localization.

### Odometry
The car uses the IMU, steering angle and speed motor encoder ticks to roughly estimate how the car moved. This estimation however is not very accurate as it is subject to slip and IMU drift. Therefore localization cannot rely on wheel odometry alone. Although wheel odometry has its downsides, it is available at a high rate and can thus be used to interpolate localization using a filter.

### Road marking localization
The road marking localization node uses ICP (iterative closest point) algorithm to align road markings seen by the camera to a given map. Given an initial position estimate it calculates the correction for the car's current position. We assume that road markings are white. We find the road markings by thresholding the camera's infrared image. For each white pixel we then find the corresponding point (x, y, z) in the pointcloud. We then project the points into the map frame using the current estimated position. Next, we apply a box filter onto the point cloud to cut off any points that are too far away (due to noise) or are not near the ground plane (z = 0). In the next step we randomly select a low amount of points out of the previous filtered point cloud. Random sampling cuts down the point cloud size significantly while still keeping the overall structure of the road markings intact.

The predefined map is taken from an occupancy grid that is being broadcast by the `map_publisher` node through ROS. The map exists as an image file on the car and represents the ground plane. The white pixels inside the map are converted into a pointcloud also known as the reference point cloud.

Now that we have both the current road markings seen by the camera (projected into the map frame at the current proposed position) and also the reference pointcloud we can use ICP to calculate the transformation matrix between both pointclouds that aligns them. ICP works by creating a kd-tree for the reference point cloud. kd-trees offer a cheap nearest neighbour lookup with computation cost of `O(log(n))`. The kd-tree is only constructed once because our reference map point cloud does not change over time. Now, for every point in the proposed point cloud we find the nearest neighbour in the reference frame and get the squared distance between them. The sum of all distances is the cost of the current transform and the goal is to minimize this cost. We apply a Levenberg-Marquardt optimization to minimize this cost using a transformation. For the translation we only allow changes in the 2D space which is a translation of x and y and a rotation around the z-axis.

#### Creating the map

### Kalman filter
We use the `robot_localization` package to fuse both wheel odometry and the localization from the `road_marking_localization` package.