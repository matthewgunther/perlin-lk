cmake_minimum_required(VERSION 3.22)

project(perlin_lk)

set(OTHER_SRC
    ${CMAKE_SOURCE_DIR}/src/engine.cpp
    ${CMAKE_SOURCE_DIR}/src/flow_field.cpp
    ${CMAKE_SOURCE_DIR}/src/particle.cpp
)

# find_package(OpenCV REQUIRED)

# include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc ${OpenCV_INCLUDE_DIRS} )
include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/../opencv-4.10.0/include)

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/lk.cpp)
target_sources(${PROJECT_NAME} PRIVATE ${OTHER_SRC})
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


# target_include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc)

# set(SRC_FILES)
