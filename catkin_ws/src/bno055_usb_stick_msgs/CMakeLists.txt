cmake_minimum_required(VERSION 3.22)
project(bno055_usb_stick_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


## Generate messages in the 'msg' folder
set(msg_files
    msg/CalibrationStatus.msg
    msg/EulerAngles.msg
    msg/Output.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(std_msgs message_geometry_msgs)
ament_package()