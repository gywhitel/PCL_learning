cmake_minimum_required(VERSION 2.8)
project(pcd_write)

find_package(PCL 1.8 QUIET REQUIRED)

include_directories(/usr/include/eigen3/)
include_directories(/usr/include/pcl-1.8)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(pcd_write pcd_write.cpp)
target_link_libraries(pcd_write ${PCL_LIBRARIES})