cmake_minimum_required(VERSION 3.22)
project(arduino_communication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(autominy_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
)

add_executable(arduino_communication_node src/ArduinoCommunication.cpp src/ArduinoCommunicationNode.cpp src/MessageType.h src/COBS.h)
target_link_libraries(arduino_communication_node ${Boost_LIBRARIES})
ament_target_dependencies(arduino_communication_node rclcpp autominy_msgs geometry_msgs serial std_srvs)

install(TARGETS arduino_communication_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(std_msgs autominy_msgs serial geometry_msgs std_srvs rclcpp)
ament_package()