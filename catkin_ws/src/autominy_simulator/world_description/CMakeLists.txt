cmake_minimum_required(VERSION 3.22)
project(world_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
