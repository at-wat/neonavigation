cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs geometry_msgs tf)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs tf
)

## Build 
include_directories(${catkin_INCLUDE_DIRS})

add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})

