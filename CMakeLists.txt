cmake_minimum_required(VERSION 2.8)
project(people_detection)
find_package(PCL 1.7 REQUIRED)

add_definitions("--std=c++11")

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(people_detector src/main.cpp)
target_link_libraries(people_detector ${PCL_LIBRARIES})

