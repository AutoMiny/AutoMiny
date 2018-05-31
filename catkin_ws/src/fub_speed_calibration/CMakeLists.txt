cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)
project(fub_speed_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS
        serialization
)

catkin_package(
#	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs geometry_msgs
    DEPENDS Boost
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(speed_calibration_node src/SpeedCalibration.cpp src/SpeedCalibrationNode.cpp src/SpeedCalibrationData.cpp src/SpeedCalibrationData.h)
target_link_libraries(speed_calibration_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS speed_calibration_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
