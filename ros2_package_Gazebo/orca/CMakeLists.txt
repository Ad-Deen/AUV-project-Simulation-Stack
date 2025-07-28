cmake_minimum_required(VERSION 3.5)
project(orca)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds scripts
    DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
