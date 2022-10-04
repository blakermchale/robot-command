cmake_minimum_required(VERSION 3.5)
project(robot_command)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
