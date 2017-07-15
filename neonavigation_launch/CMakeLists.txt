cmake_minimum_required(VERSION 2.8.3)
project(neonavigation_launch)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
	CATKIN_DEPENDS costmap_cspace planner_cspace
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

