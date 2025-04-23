cmake_minimum_required(VERSION 3.22)
project(car_description)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()