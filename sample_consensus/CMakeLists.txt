cmake_minimum_required(VERSION 3.1)

project(project)
set(TARGET sphere)
set(SRC sphere_recognition.cpp)

include_directories(/usr/include/eigen3/)
include_directories(/usr/include/pcl-1.8)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
find_package(PCL REQUIRED QUIET)

add_executable(${TARGET} ${SRC})
target_link_libraries(${TARGET} ${PCL_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
