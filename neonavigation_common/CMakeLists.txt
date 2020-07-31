cmake_minimum_required(VERSION 3.1.3)
project(neonavigation_common)

set(CATKIN_DEPENDS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  add_subdirectory(test)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
