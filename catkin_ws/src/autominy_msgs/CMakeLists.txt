cmake_minimum_required(VERSION 2.8.3)
project(autominy_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation
        )


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
        FILES
        NormalizedSpeedCommand.msg
        NormalizedSteeringCommand.msg
        SteeringAngle.msg
        Speed.msg
        SpeedCommand.msg
        SteeringAngle.msg
        SteeringCommand.msg
        SteeringFeedback.msg
        Tick.msg
        Trajectory.msg
        TrajectoryPoint.msg
        Voltage.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        std_msgs
        message_runtime
)

