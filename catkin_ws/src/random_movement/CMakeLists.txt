cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(random_movement)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(random_movement_node src/random_movement_node.cpp)
add_dependencies(random_movement_node ${PROJECT_NAME})
target_link_libraries(random_movement_node ${catkin_LIBRARIES})


