cmake_minimum_required(VERSION 2.8.3)
project(planner_cspace)

find_package(catkin REQUIRED
  COMPONENTS
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

find_package(Boost REQUIRED COMPONENTS chrono)
find_package(OpenMP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -finline-functions ${OpenMP_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#set(CMAKE_LD_FLAGS "${CMAKE_CXX_FLAGS} -pg")

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(planner_3d src/planner_3d.cpp)
target_link_libraries(planner_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planner_3d ${catkin_EXPORTED_TARGETS})

add_executable(planner_2dof_serial_joints src/planner_2dof_serial_joints.cpp)
target_link_libraries(planner_2dof_serial_joints ${catkin_LIBRARIES} ${Boost_LIBRARIES})
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
