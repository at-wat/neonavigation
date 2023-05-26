include_directories(include)

catkin_add_gtest(test_distance_map_fast_update
  src/test_distance_map_fast_update.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map_fast_update ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
