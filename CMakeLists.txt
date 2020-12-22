cmake_minimum_required(VERSION 2.8.3)
project(odrive_cpp_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  roslaunch
)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
  INCLUDE_DIRS include
  LIBRARIES odrive_cpp_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(odrive_cpp_ros
  src/odrive_cpp_ros.cpp
)

target_link_libraries(odrive_cpp_ros ${catkin_LIBRARIES} ${LIBUSB1_LIBRARIES})

add_dependencies(odrive_cpp_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(robot_odrive_interface src/robot_interface.cpp src/odrive_cpp_ros.cpp)
target_link_libraries(robot_odrive_interface ${catkin_LIBRARIES} ${LIBUSB1_LIBRARIES})
add_dependencies(robot_odrive_interface odrive_cpp_ros)
