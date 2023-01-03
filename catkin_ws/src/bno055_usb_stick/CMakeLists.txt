cmake_minimum_required(VERSION 3.22)
project(bno055_usb_stick)

find_package(bno055_usb_stick_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(
        Boost REQUIRED COMPONENTS
        system
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
)

## Declare a C++ executable
add_executable(
        bno055_usb_stick_node
        src/bno055_usb_stick_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
        bno055_usb_stick_node
        ${Boost_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(
        TARGETS
        bno055_usb_stick_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS bno055_usb_stick_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(bno055_usb_stick_node bno055_usb_stick_msgs geometry_msgs rclcpp sensor_msgs tf2_ros rclcpp)

#############
## Install ##
#############

ament_export_include_directories(include)
ament_export_dependencies(bno055_usb_stick_msgs geometry_msgs rclcpp sensor_msgs tf2_ros rclcpp)
ament_package()
