cmake_minimum_required(VERSION 2.8.3)
project(visualization_converter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
        nodelet
        roscpp
        autominy_msgs
        nav_msgs
        sensor_msgs
        )

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#    FILES
#    visualization_converter.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    fub_other_package_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_lib
        CATKIN_DEPENDS
        nodelet
        autominy_msgs
)
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        include
        )


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
        src/VisualizationConverterNodelet.cpp
        include/visualization_converter/VisualizationConverterNodelet.h)

add_dependencies(${PROJECT_NAME}_nodelet
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/VisualizationConverterNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
)

target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
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
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
        FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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
