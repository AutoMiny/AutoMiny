cmake_minimum_required(VERSION 3.0.2)
project(lanelet_force_navigation)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
        visualization_msgs
)