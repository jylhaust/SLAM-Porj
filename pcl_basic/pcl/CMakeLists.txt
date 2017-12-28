cmake_minimum_required (VERSION 2.8)
project(pcl_map)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(OpenCV_DIR "/home/yuanlin/opencv-2.4.13.4/build")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

# pcl
find_package( PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_map pcl_map.cpp)

target_link_libraries(pcl_map ${OpenCV_LIBS} ${PCL_LIBRARIES})
