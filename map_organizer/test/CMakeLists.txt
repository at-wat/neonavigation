add_rostest_gtest(test_map_organizer
  test/map_organizer_rostest.test
  src/test_map_organizer.cpp
)
target_link_libraries(test_map_organizer ${catkin_LIBRARIES})
add_dependencies(test_map_organizer tie_maps save_maps select_map)

add_rostest_gtest(test_pointcloud_to_maps
  test/pointcloud_to_maps_rostest.test
  src/test_pointcloud_to_maps.cpp
)
target_link_libraries(test_pointcloud_to_maps ${catkin_LIBRARIES})
add_dependencies(test_pointcloud_to_maps pointcloud_to_maps)
