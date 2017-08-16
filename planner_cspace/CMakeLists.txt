cmake_minimum_required(VERSION 2.8.3)
project(planner_cspace)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    actionlib
    sensor_msgs
    geometry_msgs
    move_base_msgs
    tf
    message_generation
    costmap_cspace
    rostest
    roslint
)

add_message_files(FILES PlannerStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    move_base_msgs
    tf
    costmap_cspace
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -finline-functions")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#set(CMAKE_LD_FLAGS "${CMAKE_CXX_FLAGS} -pg")

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(planner_3d src/planner_3d.cpp)
target_link_libraries(planner_3d ${catkin_LIBRARIES})
add_dependencies(planner_3d costmap_cspace_generate_messages planner_cspace_generate_messages)

add_executable(planner_2dof_serial_joints src/planner_2dof_serial_joints.cpp)
target_link_libraries(planner_2dof_serial_joints ${catkin_LIBRARIES})
add_dependencies(planner_2dof_serial_joints planner_cspace_generate_messages)

add_executable(dummy_robot src/dummy_robot.cpp)
target_link_libraries(dummy_robot ${catkin_LIBRARIES})


roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

install(TARGETS
    planner_3d
    planner_2dof_serial_joints
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

