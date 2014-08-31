cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs geometry_msgs tf interactive_markers)

add_service_files(FILES ChangePath.srv)
add_message_files(FILES TrajectoryTrackerStatus.msg TrajectoryServerStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf interactive_markers
)

## Build 
include_directories(${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(trajectory_tracker trajectory_tracker_gencpp)

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES})

add_executable(trajectory_server src/trajectory_server.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES})
add_dependencies(trajectory_server trajectory_tracker_gencpp)

