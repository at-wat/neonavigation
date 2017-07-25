cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp rostest nav_msgs tf)

catkin_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
target_link_libraries(test_costmap_3d ${catkin_LIBRARIES})
