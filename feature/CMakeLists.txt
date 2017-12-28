cmake_minimum_required( VERSION 2.8 )

project(VO)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( OpenCV_DIR "/home/yuanlin/opencv-2.4.13.4/build" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( features features.cpp)
target_link_libraries(features ${OpenCV_LIBS})