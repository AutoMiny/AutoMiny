cmake_minimum_required(VERSION 2.8.3)
project(hd_map)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    roslib
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        visualization_msgs
)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include/
)

#############################
## Build node(let) example ##
#############################

## Declare a cpp executable
add_executable(${PROJECT_NAME}_visualization
         src/Visualization.cpp src/Visualization.h)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_visualization
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${PROTOBUF_LIBRARIES}
)

# determines target name as lib[name].so, here: libfub_dummy_example_nodelet.so
# the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}
        src/HDMap.cpp src/Road.cpp src/Lane.cpp src/Polyline2D.cpp src/Polyline3D.cpp
        src/LaneGroup.cpp src/Util.h src/LaneGroupConnector.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(
    TARGETS
        ${PROJECT_NAME}
        ${PROJECT_NAME}_visualization
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/HERE/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(
#    FILES nodelet_plugins.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
