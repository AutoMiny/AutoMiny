cmake_minimum_required(VERSION 3.22)
project(remote_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        nodelet
        roscpp
        autominy_msgs
        )

find_package(SDL2 REQUIRED)

#######################################
## dynamic_reconfigure configuration ##
#######################################

generate_dynamic_reconfigure_options(
        cfg/RemoteControl.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#    FILES
#    RemoteControlFwd.srv
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
        #    ${PROJECT_NAME}_msgs
        dynamic_reconfigure
        nodelet
        autominy_msgs
        #    DEPENDS ... (e.g. eigen opencv)
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        include
        )


#####################################################
## Build library, node and nodelet of this package ##
#####################################################

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
        src/RemoteControl.cpp
        )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_lib
        #    ${PROJECT_NAME}_generate_messages_cpp
        ${PROJECT_NAME}_gencfg
        )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
        ${SDL2_LIBRARIES}
        )

## determines target name as lib[name].so, here: libremote_control_nodelet.so
## the library path in nodelet_plugins.xml refers to this file
add_library(${PROJECT_NAME}_nodelet
        src/RemoteControlNodelet.cpp
        )

add_dependencies(${PROJECT_NAME}_nodelet
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/RemoteControlNode.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ${catkin_EXPORTED_TARGETS}
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
#   DESTINATION bin
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

## Mark executables and/or libraries for installation
install(
        TARGETS
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_nodelet
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        FILES nodelet_plugins.xml
        DESTINATION share/${PROJECT_NAME}
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
