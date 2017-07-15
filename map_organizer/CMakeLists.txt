cmake_minimum_required(VERSION 2.8.3)
project(map_organizer)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp geometry_msgs tf sensor_msgs nav_msgs eigen_conversions pcl_ros tf_conversions message_generation map_server rostest roslint)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

add_message_files(FILES OccupancyGridArray.msg)
generate_messages(DEPENDENCIES nav_msgs)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp geometry_msgs tf sensor_msgs nav_msgs eigen eigen_conversions pcl_ros map_server
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_executable(pointcloud_to_maps src/pointcloud_to_maps.cpp)
target_link_libraries(pointcloud_to_maps ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_to_maps map_organizer_gencpp)

add_executable(tie_maps src/tie_maps.cpp)
target_link_libraries(tie_maps yaml-cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(tie_maps map_organizer_gencpp)

add_executable(save_maps src/save_maps.cpp)
target_link_libraries(save_maps ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(save_maps map_organizer_gencpp)

add_executable(select_map src/select_map.cpp)
target_link_libraries(select_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(select_map map_organizer_gencpp)

add_executable(pose_transform src/pose_transform.cpp)
target_link_libraries(pose_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})


roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()


install(TARGETS
    pointcloud_to_maps
    tie_maps
    save_maps
    select_map
    pose_transform
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

