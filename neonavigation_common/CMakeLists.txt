cmake_minimum_required(VERSION 3.0.2)
project(neonavigation_common)

set(CATKIN_DEPENDS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


add_compile_options(-std=c++11)
include_directories(include ${catkin_INCLUDE_DIRS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
