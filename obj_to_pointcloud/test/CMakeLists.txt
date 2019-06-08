add_rostest_gtest(test_obj_to_pointcloud
  test/obj_to_pointcloud_rostest.test
  src/test_obj_to_pointcloud.cpp
)
target_link_libraries(test_obj_to_pointcloud ${catkin_LIBRARIES})
