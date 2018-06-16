cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rostest
    geometry_msgs
    sensor_msgs
    tf
)

add_rostest_gtest(test_safety_limiter
  test/safety_limiter_rostest.test
  src/test_safety_limiter.cpp
)
add_rostest_gtest(test_safety_limiter_compat
  test/safety_limiter_compat_rostest.test
  src/test_safety_limiter.cpp
)

target_link_libraries(test_safety_limiter ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter safety_limiter)
