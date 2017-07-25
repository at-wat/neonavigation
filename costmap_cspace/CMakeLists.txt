cmake_minimum_required(VERSION 2.8.3)
project(costmap_cspace)

find_package(catkin REQUIRED COMPONENTS
	roscpp sensor_msgs geometry_msgs message_generation laser_geometry tf tf2_sensor_msgs rostest roslint
)

add_message_files(
	FILES CSpace3D.msg CSpace3DUpdate.msg MapMetaData3D.msg
)
generate_messages(
	DEPENDENCIES std_msgs geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs message_runtime laser_geometry tf tf2_sensor_msgs
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


include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(costmap_3d src/costmap_3d.cpp)
target_link_libraries(costmap_3d ${catkin_LIBRARIES})
add_dependencies(costmap_3d costmap_cspace_generate_messages)

add_executable(laserscan_to_map src/laserscan_to_map.cpp)
target_link_libraries(laserscan_to_map ${catkin_LIBRARIES})

add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
target_link_libraries(pointcloud2_to_map ${catkin_LIBRARIES})


roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

install(TARGETS
    costmap_3d
    laserscan_to_map
    pointcloud2_to_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

