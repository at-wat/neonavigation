cmake_minimum_required(VERSION 2.8.3)
project(planner_cspace)

find_package(catkin REQUIRED COMPONENTS
	roscpp sensor_msgs geometry_msgs tf message_generation message_runtime costmap_cspace
)

add_message_files(FILES PlannerStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf costmap_cspace
	DEPENDS system_lib
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

include_directories(${catkin_INCLUDE_DIRS})

add_executable(planner_3d src/planner_3d.cpp src/grid_astar.hpp)
target_link_libraries(planner_3d ${catkin_LIBRARIES})
add_dependencies(planner_3d costmap_gencpp planner_gencpp planner_generate_messages_cpp)

