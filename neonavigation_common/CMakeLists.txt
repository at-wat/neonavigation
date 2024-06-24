cmake_minimum_required(VERSION 3.5)
project(neonavigation_common)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
ament_auto_add_library(neonavigation_common SHARED
  src/neonavigation_node.cpp)

if(BUILD_TESTING)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
