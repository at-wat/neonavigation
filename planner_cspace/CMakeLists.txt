cmake_minimum_required(VERSION 3.0.2)
project(planner_cspace)

set(CATKIN_DEPENDS
  roscpp

  actionlib
  diagnostic_updater
  geometry_msgs
  move_base_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs

  costmap_cspace
  costmap_cspace_msgs
  neonavigation_common
  planner_cspace_msgs
  trajectory_tracker_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(Boost REQUIRED COMPONENTS chrono)
find_package(OpenMP REQUIRED)
catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


add_compile_options(-std=c++11 -funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(planner_3d
  src/planner_3d.cpp
  src/costmap_bbf.cpp
  src/motion_cache.cpp
  src/path_interpolator.cpp
  src/rotation_cache.cpp
)
target_link_libraries(planner_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(planner_3d ${catkin_EXPORTED_TARGETS})

add_executable(planner_2dof_serial_joints src/planner_2dof_serial_joints.cpp)
target_link_libraries(planner_2dof_serial_joints ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(planner_2dof_serial_joints ${catkin_EXPORTED_TARGETS})

add_executable(dummy_robot src/dummy_robot.cpp)
target_link_libraries(dummy_robot ${catkin_LIBRARIES})
add_dependencies(dummy_robot ${catkin_EXPORTED_TARGETS})

add_executable(patrol src/patrol.cpp)
target_link_libraries(patrol ${catkin_LIBRARIES})
add_dependencies(patrol ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  add_subdirectory(test)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(TARGETS
    dummy_robot
    patrol
    planner_2dof_serial_joints
    planner_3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
