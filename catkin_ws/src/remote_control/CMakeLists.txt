cmake_minimum_required(VERSION 3.22)
project(remote_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autominy_msgs REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(SYSTEM
        ${SDL2_INCLUDE_DIRS}
        include
        )


#####################################################
## Build library, node and nodelet of this package ##
#####################################################


add_library(${PROJECT_NAME}_nodelet SHARED
        src/RemoteControlNodelet.cpp
        )

target_link_libraries(${PROJECT_NAME}_nodelet
        ${SDL2_LIBRARIES}
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/RemoteControlNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        )

ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components autominy_msgs)
ament_target_dependencies(${PROJECT_NAME}_nodelet rclcpp rclcpp_components autominy_msgs)

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

rclcpp_components_register_nodes(${PROJECT_NAME}_nodelet "remote_control::RemoteControlNodelet")
ament_package()