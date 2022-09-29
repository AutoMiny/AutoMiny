cmake_minimum_required(VERSION 3.22)
project(emergency_stop)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autominy_msgs REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
        include)


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## determines target name as lib[name].so, here: libemergency_stop_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
        src/EmergencyStop.cpp
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/EmergencyStopNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components autominy_msgs sensor_msgs)
ament_target_dependencies(${PROJECT_NAME}_nodelet rclcpp rclcpp_components autominy_msgs sensor_msgs)

#############
## Install ##
#############

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

rclcpp_components_register_nodes(emergency_stop_nodelet "emergency_stop::EmergencyStopNodelet")
ament_export_dependencies(rclcpp rclcpp_components autominy_msgs sensor_msgs)
ament_package()