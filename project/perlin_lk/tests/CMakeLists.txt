cmake_minimum_required(VERSION 3.10)

project(perlin_lk_tests)

set(CMAKE_CXX_STANDARD 11)


find_package(GTest REQUIRED)


# Source files
add_executable(${PROJECT_NAME} perlin_lk_tests)

# Link with gtest library
target_link_libraries(${PROJECT_NAME} gtest_main)