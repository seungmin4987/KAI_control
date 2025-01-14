cmake_minimum_required(VERSION 3.0.2)
project(can_bridge_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_msg_pkg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs custom_msg_pkg
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare the C++ executable
add_executable(can_bridge_node src/can_bridge.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(can_bridge_node
  ${catkin_LIBRARIES}
)

