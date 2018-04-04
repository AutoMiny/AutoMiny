cmake_minimum_required(VERSION 2.8.3)
project(car_robot_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#	LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS std_msgs sensor_msgs 
)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)


#add_executable(${PROJECT_NAME}_node src/node.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


#install(TARGETS ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
