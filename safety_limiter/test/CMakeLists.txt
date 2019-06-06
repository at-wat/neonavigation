add_rostest_gtest(test_safety_limiter
  test/safety_limiter_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter ${catkin_LIBRARIES})
include_directories(include)

add_dependencies(test_safety_limiter safety_limiter)

add_rostest_gtest(test_safety_limiter2
  test/safety_limiter2_rostest.test
  src/test_safety_limiter2.cpp
)
target_link_libraries(test_safety_limiter2 ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter2 safety_limiter)

add_rostest_gtest(test_safety_limiter_compat
  test/safety_limiter_compat_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter_compat ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter_compat safety_limiter)
