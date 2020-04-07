cmake_minimum_required(VERSION 3.1.3)
project(costmap_cspace)

set(CATKIN_DEPENDS
  roscpp

  geometry_msgs
  laser_geometry
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  costmap_cspace_msgs
  neonavigation_common
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(xmlrpcpp REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(costmap_3d src/costmap_3d.cpp src/costmap_3d_layers.cpp)
target_link_libraries(costmap_3d ${catkin_LIBRARIES})
add_dependencies(costmap_3d ${catkin_EXPORTED_TARGETS})
set_property(TARGET costmap_3d PROPERTY PUBLIC_HEADER
  include/costmap_cspace/pointcloud_accumulator.h
)

add_executable(laserscan_to_map src/laserscan_to_map.cpp)
target_link_libraries(laserscan_to_map ${catkin_LIBRARIES})
add_dependencies(laserscan_to_map ${catkin_EXPORTED_TARGETS})

add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
target_link_libraries(pointcloud2_to_map ${catkin_LIBRARIES})
add_dependencies(pointcloud2_to_map ${catkin_EXPORTED_TARGETS})

add_executable(largemap_to_map src/largemap_to_map.cpp)
target_link_libraries(largemap_to_map ${catkin_LIBRARIES})
add_dependencies(largemap_to_map ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(TARGETS
    costmap_3d
    largemap_to_map
    laserscan_to_map
    pointcloud2_to_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
