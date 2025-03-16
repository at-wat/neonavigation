cmake_minimum_required(VERSION 3.5)
project(trajectory_tracker)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${Eigen_INCLUDE_DIRS})
ament_auto_add_executable(trajectory_tracker src/trajectory_tracker.cpp)
# ament_auto_add_executable(trajectory_recorder src/trajectory_recorder.cpp)
# ament_auto_add_executable(trajectory_saver src/trajectory_saver.cpp)
# ament_auto_add_executable(trajectory_server src/trajectory_server.cpp)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  include_directories(test/include)

  ament_auto_add_executable(time_source test/src/time_source.cpp)
  ament_auto_add_executable(test_trajectory_tracker test/src/test_trajectory_tracker.cpp)
  target_link_libraries(test_trajectory_tracker ${GTEST_LIBRARIES})
  ament_auto_add_executable(test_trajectory_tracker_with_odom test/src/test_trajectory_tracker_with_odom.cpp)
  target_link_libraries(test_trajectory_tracker_with_odom ${GTEST_LIBRARIES})
  ament_auto_add_executable(test_trajectory_tracker_overshoot test/src/test_trajectory_tracker_overshoot.cpp)
  target_link_libraries(test_trajectory_tracker_overshoot ${GTEST_LIBRARIES})

  install(DIRECTORY
    test/configs
    DESTINATION share/${PROJECT_NAME}/test/
  )
  add_launch_test(test/test/test_trajectory_tracker_launch.py
    TARGET test_trajectory_tracker
    ARGS use_sim_time:=true
    TIMEOUT 120
  )
  add_launch_test(test/test/test_trajectory_tracker_launch.py
    TARGET test_trajectory_tracker_with_odom_delay
    ARGS use_sim_time:=true odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
    TIMEOUT 120
  )
  add_launch_test(test/test/test_trajectory_tracker_launch.py
    TARGET test_trajectory_tracker_without_time_optimal_control
    ARGS use_sim_time:=true odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
    TIMEOUT 120
  )
  add_launch_test(test/test/test_trajectory_tracker_overshoot_launch.py
    TARGET test_trajectory_tracker_overshoot
    ARGS test_executable:=test_trajectory_tracker_overshoot
    TIMEOUT 60
  )
  add_launch_test(test/test/test_trajectory_tracker_overshoot_launch.py
    TARGET test_trajectory_tracker_with_odom
    ARGS test_executable:=test_trajectory_tracker_with_odom
    TIMEOUT 60
  )
endif()

ament_auto_package()
