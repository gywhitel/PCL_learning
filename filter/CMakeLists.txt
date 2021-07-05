cmake_minimum_required(VERSION 3.1)

project(filter)
set(TARGET outlierRemoval)
set(SRC downsampling.cpp)

add_executable(${TARGET} ${SRC})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(/usr/include/eigen3/)
include_directories(/usr/include/pcl-1.8)

find_package(PCL 1.8 REQUIRED QUIET)    #do not use COMPONENTS here, otherwise only components would be imported
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


target_link_libraries(${TARGET} ${PCL_LIBRARIES})