add_rostest_gtest(test_compat test/compat_rostest.test
    src/test_compat.cpp)
target_link_libraries(test_compat ${catkin_LIBRARIES})
