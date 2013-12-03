cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator)

find_package(OpenCV REQUIRED)
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

add_executable(PoseEstimator
    PoseEstimator.cpp
    Corners.h
    Corners.cpp
    Geometry.h
    Geometry.cpp
)
target_link_libraries(PoseEstimator ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(TestGeometry
    TestGeometry.cpp
    Geometry.h
    Geometry.cpp
)
target_link_libraries(TestGeometry ${OpenCV_LIBS})

add_executable(TestCornersFromImages
    TestCornersFromImages.cpp
    Corners.h
    Corners.cpp
)
target_link_libraries(TestCornersFromImages ${OpenCV_LIBS})

add_executable(TestCornersFromCamera
    TestCornersFromCamera.cpp
    Corners.h
    Corners.cpp
)
target_link_libraries(TestCornersFromCamera ${OpenCV_LIBS})

add_executable(TestPoseFromImages
    TestPoseFromImages.cpp
    Corners.h
    Corners.cpp
    Geometry.h
    Geometry.cpp
)
target_link_libraries(TestPoseFromImages ${OpenCV_LIBS})

add_executable(TestPoseFromCamera
    TestPoseFromCamera.cpp
    Corners.h
    Corners.cpp
    Geometry.h
    Geometry.cpp
)
target_link_libraries(TestPoseFromCamera ${OpenCV_LIBS})

set( CMAKE_BUILD_TYPE Debug )
