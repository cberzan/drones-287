cmake_minimum_required(VERSION 2.8.3)
project(lander)

find_package(catkin REQUIRED COMPONENTS
  	genmsg
    roscpp
    std_msgs
)

## Declare ROS messages, services and actions

## Generate added messages and services
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

## catkin specific configuration
catkin_package(
)

## Build

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Lander
    Lander.cpp
    Controller.h
    Controller.cpp
)
target_link_libraries(Lander ${catkin_LIBRARIES})

set( CMAKE_BUILD_TYPE Debug )
