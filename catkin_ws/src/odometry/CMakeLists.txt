cmake_minimum_required(VERSION 2.8.3)
project(odometry)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS nav_msgs std_msgs roscpp tf fub_modelcar_tools autominy_msgs)

catkin_package(
    CATKIN_DEPENDS
        nav_msgs
        std_msgs
        autominy_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

# Build the executable
add_executable(odometry_publisher src/odometry_publisher.cpp)

# Link against the catkin libraries
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})

# Install the executable
install(TARGETS odometry_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## Mark cpp header files for installation
install(DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
