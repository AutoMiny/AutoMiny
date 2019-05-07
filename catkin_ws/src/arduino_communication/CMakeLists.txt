cmake_minimum_required(VERSION 2.8.3)
project(arduino_communication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        autominy_msgs
        geometry_msgs
        serial
        std_srvs
)

catkin_package(
    CATKIN_DEPENDS autominy_msgs geometry_msgs std_srvs
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(arduino_communication_node src/ArduinoCommunication.cpp src/ArduinoCommunicationNode.cpp src/MessageType.h src/COBS.h)
target_link_libraries(arduino_communication_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(arduino_communication_node ${catkin_EXPORTED_TARGETS})


install(TARGETS arduino_communication_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
