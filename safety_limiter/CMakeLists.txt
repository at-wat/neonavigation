cmake_minimum_required(VERSION 2.8.3)
project(safety_limiter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp

    geometry_msgs
    pcl_ros
    sensor_msgs
    tf
    tf_conversions
)
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package(
  DEPENDS
    roscpp

    geometry_msgs
    pcl_ros
    sensor_msgs
    tf
    tf_conversions
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Workaround for the bug in PCL(<1.8.1) https://github.com/PointCloudLibrary/pcl/issues/1406
remove_definitions(-DDISABLE_LIBUSB-1.0)

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

add_executable(safety_limiter src/safety_limiter.cpp)
target_link_libraries(safety_limiter ${catkin_LIBRARIES} ${PCL_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS
    safety_limiter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

