cmake_minimum_required(VERSION 3.5)
project(safety_limiter)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(${PCL_DEFINITIONS})

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)

# Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
list(REMOVE_ITEM PCL_LIBRARIES
  "vtkGUISupportQt"
  "vtkGUISupportQtOpenGL"
  "vtkGUISupportQtSQL"
  "vtkGUISupportQtWebkit"
  "vtkViewsQt"
  "vtkRenderingQt"
)

# Workaround for the bug in PCL(<1.8.1) https://github.com/PointCloudLibrary/pcl/issues/1406
remove_definitions(-DDISABLE_LIBUSB-1.0)

include_directories(${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
ament_auto_add_executable(safety_limiter src/safety_limiter.cpp)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  include_directories(test/include)
  ament_auto_add_executable(test_safety_limiter test/src/test_safety_limiter.cpp)
  target_link_libraries(test_safety_limiter ${GTEST_LIBRARIES})
  ament_auto_add_executable(test_safety_limiter2 test/src/test_safety_limiter2.cpp)
  target_link_libraries(test_safety_limiter2 ${GTEST_LIBRARIES})

  install(DIRECTORY
    test/configs
    DESTINATION share/${PROJECT_NAME}/test/
  )
  add_launch_test(test/test/test_safety_limiter_launch.py
    TARGET test_safety_limiter
    TIMEOUT 120
  )
  add_launch_test(test/test/test_safety_limiter2_launch.py
    TARGET test_safety_limiter2
    TIMEOUT 30
  )
endif()

ament_auto_package()
