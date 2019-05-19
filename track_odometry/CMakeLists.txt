cmake_minimum_required(VERSION 2.8.3)
project(track_odometry)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cmake_modules

    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs

    neonavigation_common
)

catkin_package(
  CATKIN_DEPENDS
    roscpp

    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs

    neonavigation_common
)

find_package(Eigen3 REQUIRED)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(track_odometry src/track_odometry.cpp)
target_link_libraries(track_odometry ${catkin_LIBRARIES})
add_dependencies(track_odometry ${catkin_EXPORTED_TARGETS})

add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)
target_link_libraries(tf_projection ${catkin_LIBRARIES})
add_dependencies(tf_projection ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  add_subdirectory(test)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS
    tf_projection
    track_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

