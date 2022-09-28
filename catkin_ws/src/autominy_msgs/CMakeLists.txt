cmake_minimum_required(VERSION 3.22)
project(autominy_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

set(msg_files
        msg/NormalizedSpeedCommand.msg
        msg/NormalizedSteeringCommand.msg
        msg/Plot.msg
        msg/Speed.msg
        msg/SpeedCommand.msg
        msg/SpeedPWMCommand.msg
        msg/SteeringAngle.msg
        msg/SteeringCommand.msg
        msg/SteeringPWMCommand.msg
        msg/SteeringFeedback.msg
        msg/Tick.msg
        msg/Trajectory.msg
        msg/TrajectoryPoint.msg
        msg/Voltage.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(std_msgs geometry_msgs)
ament_package()