cmake_minimum_required(VERSION 3.1.3)
project(trajectory_tracker)

set(CATKIN_DEPENDS
  roscpp

  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros

  neonavigation_common
  trajectory_tracker_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TrajectoryTracker.cfg
)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

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
