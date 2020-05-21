add_rostest_gtest(test_safety_limiter
  test/safety_limiter_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter ${catkin_LIBRARIES} ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})
target_include_directories(test_safety_limiter PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
include_directories(include)

add_dependencies(test_safety_limiter safety_limiter)

add_rostest_gtest(test_safety_limiter2
  test/safety_limiter2_rostest.test
  src/test_safety_limiter2.cpp
)
target_link_libraries(test_safety_limiter2 ${catkin_LIBRARIES} ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})
target_include_directories(test_safety_limiter2 PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
add_dependencies(test_safety_limiter2 safety_limiter)

add_rostest_gtest(test_safety_limiter_compat
  test/safety_limiter_compat_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter_compat ${catkin_LIBRARIES} ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})
target_include_directories(test_safety_limiter_compat PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
add_dependencies(test_safety_limiter_compat safety_limiter)
