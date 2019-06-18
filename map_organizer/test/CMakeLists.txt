add_rostest_gtest(test_map_organizer
  test/map_organizer_rostest.test
  src/test_map_organizer.cpp
)
target_link_libraries(test_map_organizer ${catkin_LIBRARIES})
