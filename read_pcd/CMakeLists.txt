cmake_minimum_required(VERSION 3.1)

project(read_pcd)

include_directories("/usr/include/eigen3/")
include_directories("/usr/include/pcl-1.8")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(read_pcd read_pcd.cpp)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(read_pcd ${PCL_LIBRARIES})
