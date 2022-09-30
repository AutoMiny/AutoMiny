cmake_minimum_required(VERSION 3.22)
project(lidar_pose_estimation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)

include_directories(SYSTEM
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        include
        )

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
        src/LidarPoseEstimationNodelet.cpp
        )

target_link_libraries(${PROJECT_NAME}_nodelet
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/LidarPoseEstimationNode.cpp
        )

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_ros pcl_ros laser_geometry)
ament_target_dependencies(${PROJECT_NAME}_nodelet rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_ros pcl_ros laser_geometry)

## Mark executables and/or libraries for installation
install(
        TARGETS
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()