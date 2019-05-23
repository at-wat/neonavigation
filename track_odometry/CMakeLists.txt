cmake_minimum_required(VERSION 3.0.2)
project(track_odometry)

set(CATKIN_DEPENDS
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

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(Eigen3 REQUIRED)
catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


add_compile_options(-std=c++11)
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
