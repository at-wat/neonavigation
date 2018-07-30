catkin_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter ${catkin_LIBRARIES})
