cmake_minimum_required(VERSION 3.1)

project(project)

include_directories(/usr/include/eigen3/)
include_directories(/usr/include/pcl-1.8)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(transform transform.cpp)

find_package(PCL REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(transform ${PCL_LIBRARIES})
