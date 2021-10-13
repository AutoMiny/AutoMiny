cmake_minimum_required(VERSION 3.0.2)
project(autominy_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation
        nav_msgs
        )


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
        FILES
        NormalizedSpeedCommand.msg
        NormalizedSteeringCommand.msg
        Plot.msg
        SteeringAngle.msg
        Speed.msg
        SpeedCommand.msg
        SpeedPWMCommand.msg
        SteeringAngle.msg
        SteeringCommand.msg
        SteeringPWMCommand.msg
        SteeringFeedback.msg
        Tick.msg
        Trajectory.msg
        TrajectoryPoint.msg
        Obstacle.msg
        Obstacles.msg
        Voltage.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        std_msgs
        message_runtime
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
        )
