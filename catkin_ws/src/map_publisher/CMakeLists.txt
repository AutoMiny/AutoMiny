cmake_minimum_required(VERSION 3.22)
project(map_publisher)

find_package(ament_cmake REQUIRED)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY map
        DESTINATION share/${PROJECT_NAME}
)

ament_package()