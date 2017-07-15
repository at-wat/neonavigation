cmake_minimum_required(VERSION 2.8.3)
project(joystick_interrupt)

find_package(catkin REQUIRED COMPONENTS
	roscpp sensor_msgs geometry_msgs topic_tools
)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs topic_tools
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


include_directories(${catkin_INCLUDE_DIRS})

add_executable(joystick_interrupt src/joystick_interrupt.cpp)
target_link_libraries(joystick_interrupt ${catkin_LIBRARIES})

add_executable(joystick_mux src/joystick_mux.cpp)
target_link_libraries(joystick_mux ${catkin_LIBRARIES})

install(TARGETS
    joystick_interrupt
    joystick_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

