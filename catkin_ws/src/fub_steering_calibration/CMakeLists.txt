cmake_minimum_required(VERSION 3.22)

project(fub_steering_calibration)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(
#     TARGETS
#         ${PROJECT_NAME}
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     PATTERN ".svn" EXCLUDE
# )
