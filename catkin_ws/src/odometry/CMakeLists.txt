cmake_minimum_required(VERSION 3.22)
project(odometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autominy_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(SYSTEM
        include
)


add_library(${PROJECT_NAME}_nodelet SHARED
        src/OdometryNodelet.cpp
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/OdometryNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

ament_target_dependencies(${PROJECT_NAME}_node autominy_msgs rclcpp rclcpp_components nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(${PROJECT_NAME}_nodelet autominy_msgs rclcpp rclcpp_components nav_msgs tf2 tf2_ros tf2_geometry_msgs)

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

install(
        TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

rclcpp_components_register_nodes(odometry_nodelet "odometry::OdometryNodelet")
ament_package()