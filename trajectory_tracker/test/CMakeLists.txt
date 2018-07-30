catkin_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter ${catkin_LIBRARIES})

add_rostest_gtest(test_trajectory_recorder test/trajectory_recorder_rostest.test
    src/test_trajectory_recorder.cpp)
target_link_libraries(test_trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(test_trajectory_recorder trajectory_recorder)
