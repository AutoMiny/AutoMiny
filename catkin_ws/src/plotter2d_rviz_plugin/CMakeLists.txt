cmake_minimum_required(VERSION 2.8.3)
project(plotter2d_rviz_plugin)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        autominy_msgs
        rviz
        image_geometry
        cv_bridge
        )

###################################
## catkin specific configuration ##
###################################

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        autominy_msgs
        rviz
        image_geometry
        cv_bridge
)


###########
## Build ##
###########

## Specify additional locations of header file
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        )

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if ("${rviz_QT_VERSION}" VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else ()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif ()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)


add_library(${PROJECT_NAME}
        src/plotter_2d_display.cpp
        src/overlay_utils.cpp
        src/camera_info_display.cpp
        src/tf_trajectory_display.cpp
        src/overlay_image_display.cpp
        src/overlay_camera_display.cpp
        src/twist_stamped_display.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME}
        ${QT_LIBRARIES}
        ${catkin_LIBRARIES}
        )


#############
## Install ##
#############

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})