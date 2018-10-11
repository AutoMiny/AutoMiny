cmake_minimum_required(VERSION 2.8.3)
project(fub_modelcar_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
)
find_package(Boost REQUIRED COMPONENTS
    serialization
)
find_package(OpenCV REQUIRED COMPONENTS
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS
    DEPENDS Boost OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#It is neccessary to be able to be linked from other packages
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Opencv_INCLUDE_DIRS}
)



###################
## Build library ##
###################

add_library(${PROJECT_NAME}
        src/xml_write_read.cpp
)



############################
## Build node(let)        ##
############################

## Declare a cpp executable
add_executable(${PROJECT_NAME}_test
        example/test.cpp

)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_test
	${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
    TARGETS
        ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
