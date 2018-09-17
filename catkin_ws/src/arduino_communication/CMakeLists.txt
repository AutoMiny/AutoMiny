cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)
project(arduino_communication)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        serial
)

catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(arduino_communication_node src/ArduinoCommunication.cpp src/ArduinoCommunicationNode.cpp src/MessageType.h)
target_link_libraries(arduino_communication_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS arduino_communication_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
