cmake_minimum_required(VERSION 3.0.2)
project(joystick_interrupt)

set(CATKIN_DEPENDS
  roscpp

  geometry_msgs
  sensor_msgs
  topic_tools

  neonavigation_common
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)


add_compile_options(-std=c++11)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(joystick_interrupt src/joystick_interrupt.cpp)
target_link_libraries(joystick_interrupt ${catkin_LIBRARIES})

add_executable(joystick_mux src/joystick_mux.cpp)
target_link_libraries(joystick_mux ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  add_subdirectory(test)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(TARGETS
    joystick_interrupt
    joystick_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
