cmake_minimum_required(VERSION 3.22)
project(sim_car_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(autominy_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a cpp library
add_library(${PROJECT_NAME} src/AutominySimController.cpp
                            include/AutominySimController.h)

ament_target_dependencies(${PROJECT_NAME} autominy_msgs rclcpp urdf controller_interface hardware_interface control_toolbox controller_manager xacro geometry_msgs nav_msgs std_msgs sensor_msgs tf2 tf2_ros)

ament_package()