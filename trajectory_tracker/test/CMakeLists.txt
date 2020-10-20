include_directories(include)

catkin_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter ${catkin_LIBRARIES})

catkin_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_link_libraries(test_${PROJECT_NAME}_path2d ${catkin_LIBRARIES})

add_rostest_gtest(test_trajectory_recorder test/trajectory_recorder_rostest.test
    src/test_trajectory_recorder.cpp)
target_link_libraries(test_trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(test_trajectory_recorder trajectory_recorder)

add_rostest_gtest(test_trajectory_tracker test/trajectory_tracker_rostest.test
    src/test_trajectory_tracker.cpp)
target_link_libraries(test_trajectory_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_trajectory_tracker trajectory_tracker)

add_rostest(test/trajectory_tracker_rostest.test
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
  DEPENDENCIES test_trajectory_tracker
)

add_rostest(test/trajectory_tracker_rostest.test
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
  DEPENDENCIES test_trajectory_tracker
)

add_rostest_gtest(test_trajectory_tracker_with_odom test/trajectory_tracker_with_odom_rostest.test
    src/test_trajectory_tracker_with_odom.cpp)
target_link_libraries(test_trajectory_tracker_with_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_trajectory_tracker_with_odom trajectory_tracker)
