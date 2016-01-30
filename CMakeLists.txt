cmake_minimum_required(VERSION 2.8)
project(people_detection)
find_package(PCL 1.7 REQUIRED)

# https://github.com/PointCloudLibrary/pcl/issues/619
# add_definitions("--std=c++11")

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(people_detector src/main.cpp)
add_executable(extract_faces src/extract_faces.cpp)
add_executable(sample_app apps/main_ground_based_people_detection.cpp)
target_link_libraries(people_detector ${PCL_LIBRARIES})
target_link_libraries(extract_faces ${PCL_LIBRARIES})
target_link_libraries(sample_app ${PCL_LIBRARIES})
