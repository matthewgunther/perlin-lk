cmake_minimum_required(VERSION 3.22)

project(perlin_lk)


find_package( OpenCV REQUIRED )
# add_executable( DisplayImage DisplayImage.cpp )

message("opencv libs") 
# message(${OpenCV_LIBS})
# add_subdirectory(tests)

# set(OTHER_SRC
#     ${CMAKE_SOURCE_DIR}/src/engine.cpp
#     ${CMAKE_SOURCE_DIR}/src/flow_field.cpp
#     ${CMAKE_SOURCE_DIR}/src/particle.cpp
# )

# # find_package(OpenCV REQUIRED)

# # include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc ${OpenCV_INCLUDE_DIRS} )
# include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/../opencv-4.10.0/include)
include_directories(${CMAKE_SOURCE_DIR}/inc)


add_library(${PROJECT_NAME}_lib 
${CMAKE_SOURCE_DIR}/inc/engine.h ${CMAKE_SOURCE_DIR}/inc/flow_field.h ${CMAKE_SOURCE_DIR}/inc/particle.h 
${CMAKE_SOURCE_DIR}/src/engine.cpp ${CMAKE_SOURCE_DIR}/src/flow_field.cpp ${CMAKE_SOURCE_DIR}/src/particle.cpp )
# target_include_directories(${PROJECT_NAME}_lib  ${CMAKE_SOURCE_DIR}/inc)
target_link_libraries( ${PROJECT_NAME}_lib ${OpenCV_LIBS})
# target_link_libraries( ${PROJECT_NAME}_lib OpenCV::OpenCV)


add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/lk.cpp)
target_link_libraries( ${PROJECT_NAME} ${PROJECT_NAME}_lib)



# target_sources(${PROJECT_NAME} PRIVATE ${OTHER_SRC})
# # target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


# # target_include_directories(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/inc)

# # set(SRC_FILES)
