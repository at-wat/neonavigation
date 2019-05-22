cmake_minimum_required(VERSION 3.1.3)
project(neonavigation_launch)

find_package(catkin REQUIRED)
catkin_package(
	CATKIN_DEPENDS
)


install(DIRECTORY launch config map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
