cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    roscpp

    geometry_msgs
    interactive_markers
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros

    neonavigation_common
    trajectory_tracker_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp

    geometry_msgs
    interactive_markers
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros

    neonavigation_common
    trajectory_tracker_msgs
)

## Build
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(trajectory_recorder ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
add_dependencies(trajectory_saver ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_server src/trajectory_server.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES})
add_dependencies(trajectory_server ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  add_subdirectory(test)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS
    trajectory_recorder
    trajectory_saver
    trajectory_server
    trajectory_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
