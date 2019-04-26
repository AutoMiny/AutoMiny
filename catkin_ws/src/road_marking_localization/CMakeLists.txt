cmake_minimum_required(VERSION 2.8.3)
project(road_marking_localization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O3")

include(TargetArch.cmake)
determine_target_architecture(TARGET_ARCH)
if ("${TARGET_ARCH}" STREQUAL "arm")
    message("Disabling Eigen vectorization due to bugs with pcl on ARM")
    add_definitions(-DEIGEN_DONT_ALIGN)
    add_definitions(-DEIGEN_DONT_VECTORIZE)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)
    add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
        #     ${PROJECT_NAME}_msgs
        dynamic_reconfigure
        nodelet
        roscpp
        image_transport
        nav_msgs
        sensor_msgs
        cv_bridge
        image_transport
        image_geometry
        tf2
        tf2_ros
        tf2_eigen
        tf2_msgs
        tf2_geometry_msgs
        pcl_ros
        eigen_conversions
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

#######################################
## dynamic_reconfigure configuration ##
#######################################

generate_dynamic_reconfigure_options(
        cfg/RoadMarkingLocalization.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#    FILES
#    road_marking_localization.srv
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
        dynamic_reconfigure
        nodelet
        eigen_conversions
)
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${cpd_INCLUDE_DIRS}
        include
        )


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
        src/RoadMarkingLocalization.cpp
        src/crop_box.cpp include/road_marking_localization/crop_box.h include/road_marking_localization/crop_box.hpp src/random_sample.cpp include/road_marking_localization/random_sample.hpp include/road_marking_localization/random_sample.h)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_lib
        ${PROJECT_NAME}_gencfg
        )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
        )

## determines target name as lib[name].so, here: libfub_dummy_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
        src/RoadMarkingLocalizationNodelet.cpp
        )

add_dependencies(${PROJECT_NAME}_nodelet
        #    ${PROJECT_NAME}_generate_messages_cpp
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/RoadMarkingLocalizationNode.cpp
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
        ${PROJECT_NAME}_lib
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
