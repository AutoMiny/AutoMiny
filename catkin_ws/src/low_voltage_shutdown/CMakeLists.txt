cmake_minimum_required(VERSION 3.22)
project(low_voltage_shutdown)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autominy_msgs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
        include
        )


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet SHARED
        src/LowVoltageShutdownNodelet.cpp
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/LowVoltageShutdownNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components autominy_msgs)
ament_target_dependencies(${PROJECT_NAME}_nodelet rclcpp rclcpp_components autominy_msgs)

## Mark executables and/or libraries for installation
install(
        TARGETS
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

rclcpp_components_register_nodes(low_voltage_shutdown_nodelet "low_voltage_shutdown::LowVoltageShutdownNodelet")
ament_package()