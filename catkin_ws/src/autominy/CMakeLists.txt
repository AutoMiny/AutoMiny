cmake_minimum_required(VERSION 3.22)
project(autominy)
 
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})

ament_package()
