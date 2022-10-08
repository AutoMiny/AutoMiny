cmake_minimum_required(VERSION 3.22)
project(autominy_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME})

ament_package()