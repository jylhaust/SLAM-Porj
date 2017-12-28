cmake_minimum_required( VERSION 2.8 )

project(VO)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( OpenCV_DIR "/home/yuanlin/opencv-2.4.13.4/build")

find_package( OpenCV REQUIRED )
# find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( pose2d2d poseEstimate2d2d.cpp )

target_link_libraries( pose2d2d
        ${OpenCV_LIBS}
)