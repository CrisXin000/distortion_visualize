cmake_minimum_required(VERSION 2.8)
project(distortion_visualize)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} visualize.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} yaml-cpp)
