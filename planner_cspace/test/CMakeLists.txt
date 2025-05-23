include_directories(include)

# planner_3d node with assertion
add_executable(planner_3d_debug
  ../src/costmap_bbf.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/planner_3d.cpp
  ../src/distance_map.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_compile_definitions(planner_3d_debug PUBLIC DEBUG)
target_link_libraries(planner_3d_debug ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(planner_3d_debug ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

catkin_add_gtest(test_blockmem_gridmap src/test_blockmem_gridmap.cpp)
target_link_libraries(test_blockmem_gridmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_grid_astar src/test_grid_astar.cpp)
target_link_libraries(test_grid_astar ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})

catkin_add_gtest(test_cyclic_vec src/test_cyclic_vec.cpp)
target_link_libraries(test_cyclic_vec ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_distance_map
  src/test_distance_map.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})

catkin_add_gtest(test_distance_map_fast_update
  src/test_distance_map_fast_update.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map_fast_update ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})

catkin_add_gtest(test_grid_metric_converter src/test_grid_metric_converter.cpp)
target_link_libraries(test_grid_metric_converter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_planner_3d_cost
  src/test_planner_3d_cost.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_planner_3d_cost ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_costmap_bbf
  src/test_costmap_bbf.cpp
  ../src/costmap_bbf.cpp
)
target_link_libraries(test_costmap_bbf ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_motion_cache
  src/test_motion_cache.cpp
  ../src/motion_cache.cpp
)
target_link_libraries(test_motion_cache ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_motion_primitive_builder
  src/test_motion_primitive_builder.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_motion_primitive_builder ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_start_pose_predictor
  src/test_start_pose_predictor.cpp
  ../src/motion_cache.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_link_libraries(test_start_pose_predictor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_rostest_gtest(test_debug_outputs
  test/debug_outputs_rostest.test
  src/test_debug_outputs.cpp
)
target_link_libraries(test_debug_outputs ${catkin_LIBRARIES})
add_dependencies(test_debug_outputs planner_3d)

add_rostest_gtest(test_navigate
  test/navigation_rostest.test
  src/test_navigate.cpp
)
target_link_libraries(test_navigate ${catkin_LIBRARIES})
add_dependencies(test_navigate planner_3d patrol)

add_rostest(test/navigation_rostest.test
  ARGS antialias_start:=true
  DEPENDENCIES test_navigate
)

add_rostest(test/navigation_rostest.test
  ARGS antialias_start:=true fast_map_update:=true
  DEPENDENCIES test_navigate
)

add_rostest(test/navigation_rostest.test
  ARGS with_tolerance:=true
  DEPENDENCIES test_navigate
)

add_rostest(test/navigation_rostest.test
  ARGS enable_crowd_mode:=true
  DEPENDENCIES test_navigate
)

add_rostest_gtest(test_navigate_remember
  test/navigation_remember_rostest.test
  src/test_navigate_remember.cpp
)
target_link_libraries(test_navigate_remember ${catkin_LIBRARIES})
add_dependencies(test_navigate_remember planner_3d patrol)

add_rostest(test/navigation_compat_rostest.test
  DEPENDENCIES test_navigate
)

add_rostest_gtest(test_navigate_boundary
  test/navigation_boundary_rostest.test
  src/test_navigate_boundary.cpp
)
target_link_libraries(test_navigate_boundary ${catkin_LIBRARIES})
add_dependencies(test_navigate_boundary planner_3d_debug)

add_rostest_gtest(test_abort
  test/abort_rostest.test
  src/test_abort.cpp
)
target_link_libraries(test_abort ${catkin_LIBRARIES})
add_dependencies(test_abort planner_3d)

add_rostest_gtest(test_preempt
  test/preempt_rostest.test
  src/test_preempt.cpp
)
target_link_libraries(test_preempt ${catkin_LIBRARIES})
add_dependencies(test_preempt planner_3d)

add_rostest_gtest(test_costmap_watchdog
  test/costmap_watchdog_rostest.test
  src/test_costmap_watchdog.cpp
)
target_link_libraries(test_costmap_watchdog ${catkin_LIBRARIES})
add_dependencies(test_costmap_watchdog planner_3d)

add_rostest_gtest(test_planner_2dof_serial_joints
  test/planner_2dof_serial_joints_rostest.test
  src/test_planner_2dof_serial_joints.cpp
)
target_link_libraries(test_planner_2dof_serial_joints ${catkin_LIBRARIES})
add_dependencies(test_planner_2dof_serial_joints planner_2dof_serial_joints)

add_rostest_gtest(test_planner_3d_map_size
  test/planner_3d_map_size_rostest.test
  src/test_planner_3d_map_size.cpp
)
target_link_libraries(test_planner_3d_map_size ${catkin_LIBRARIES})
add_dependencies(test_planner_3d_map_size planner_3d)

add_rostest_gtest(test_tolerant_action
  test/tolerant_action_rostest.test
  src/test_tolerant_action.cpp
)
target_link_libraries(test_tolerant_action ${catkin_LIBRARIES})
add_dependencies(test_tolerant_action planner_3d)

add_rostest_gtest(test_dynamic_parameter_change
  test/dynamic_parameter_change_rostest.test
  src/test_dynamic_parameter_change.cpp
)
target_link_libraries(test_dynamic_parameter_change ${catkin_LIBRARIES})
add_dependencies(test_dynamic_parameter_change planner_3d)

if(NEONAVIGATION_EXTRA_TESTS)

  catkin_add_gtest(test_blockmem_gridmap_performance
    src/test_blockmem_gridmap_performance.cpp
  )
  target_link_libraries(test_blockmem_gridmap_performance ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
  # Force release build for performance test.
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

endif()
