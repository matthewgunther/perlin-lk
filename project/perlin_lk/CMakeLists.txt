cmake_minimum_required(VERSION 3.22)

project(perlin_lk)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)

include_directories(${CMAKE_SOURCE_DIR}/inc)

set(INC_FILES ${CMAKE_SOURCE_DIR}/inc/engine.h ${CMAKE_SOURCE_DIR}/inc/bubble.h)
set(SRC_FILES ${CMAKE_SOURCE_DIR}/src/engine.cpp
              ${CMAKE_SOURCE_DIR}/src/bubble.cpp)

# Create library for executable
add_library(${PROJECT_NAME}_lib ${INC_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS} ${Boost_LIBRARIES})

# Create executable
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
