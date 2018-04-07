cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp rostest nav_msgs tf)

catkin_add_gtest(test_cyclic_vec src/test_cyclic_vec.cpp)
target_link_libraries(test_cyclic_vec ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_blockmem_gridmap src/test_blockmem_gridmap.cpp)
target_link_libraries(test_blockmem_gridmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(PLANNER_CSPACE_PERFORMANCE_TESTS)

  catkin_add_gtest(test_blockmem_gridmap_performance src/test_blockmem_gridmap_performance.cpp)
  target_link_libraries(test_blockmem_gridmap_performance ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}") # Force release build for performance test.

else(PLANNER_CSPACE_PERFORMANCE_TESTS)

  message("To perform performance tests for planner_cspace, use -DPLANNER_CSPACE_PERFORMANCE_TESTS:=ON")

endif(PLANNER_CSPACE_PERFORMANCE_TESTS)

add_rostest_gtest(test_navigate
  test/navigation_rostest.test
  src/test_navigate.cpp
)
target_link_libraries(test_navigate ${catkin_LIBRARIES})
add_dependencies(test_navigate planner_3d)

add_rostest_gtest(test_abort
  test/abort_rostest.test
  src/test_abort.cpp
)
target_link_libraries(test_abort ${catkin_LIBRARIES})
add_dependencies(test_abort planner_3d)
