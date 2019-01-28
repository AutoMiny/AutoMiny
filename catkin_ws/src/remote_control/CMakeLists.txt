cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)

project(remote_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  autominy_msgs
  sensor_msgs
  geometry_msgs
)

#find_package(Boost COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

include_directories(
  include
#  /usr/include/websocketpp/
#  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(Sources 
  nodes/remote_control.cpp
  src/controller.cpp
 # src/httpcommunicator.cpp
  src/monitor.cpp
  src/xboxcommunicator.cpp
)

add_executable(${PROJECT_NAME}_node ${Sources})
add_dependencies(remote_control_node ${PROJECT_NAME})
target_link_libraries(remote_control_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS remote_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
