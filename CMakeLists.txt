cmake_minimum_required(VERSION 3.22)
project(ad_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ---- 核心库 ----
add_library(adc_core
  src/vehicle_model.cpp
  src/trajectory.cpp
  src/metrics.cpp
  src/sim_loop.cpp
)
target_include_directories(adc_core PUBLIC include)

# ---- 可执行 ----
add_executable(adc_sim src/main.cpp)
target_link_libraries(adc_sim PRIVATE adc_core)

# ---- 测试 ----
enable_testing()
find_package(GTest REQUIRED)

add_executable(adc_test
  test/test_vehicle_model.cpp
  test/test_trajectory.cpp
)
target_link_libraries(adc_test PRIVATE adc_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(adc_test)
