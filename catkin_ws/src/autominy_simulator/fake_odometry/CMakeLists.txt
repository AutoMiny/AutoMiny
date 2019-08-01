cmake_minimum_required(VERSION 2.8.3)
project(fake_odometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS nav_msgs std_msgs roscpp tf autominy_msgs)

catkin_package(
    CATKIN_DEPENDS
        nav_msgs
        std_msgs
        autominy_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

# Build the executable
add_executable(fake_odometry src/odometry_publisher.cpp)
add_dependencies(fake_odometry ${catkin_EXPORTED_TARGETS})

# Link against the catkin libraries
target_link_libraries(fake_odometry ${catkin_LIBRARIES})

# Install the executable
install(TARGETS fake_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## Mark cpp header files for installation
install(DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
