cmake_minimum_required(VERSION 2.8.3)
project(roscopter_pose_estimator)

## Find catkin macros and libraries
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

add_executable(PoseEstimator PoseEstimator.cpp)
target_link_libraries(PoseEstimator ${catkin_LIBRARIES})
