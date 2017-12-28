PROJECT(image)
cmake_minimum_required(VERSION 2.8)
# C++11
set(CMAKE_CXX_FLAGS "-std=c++11")

#find opencv 2.4.13

#find_package(OpenCV 2.4.13.4 EXACT REQUIRED)
#set(OpenCV_DIR "/home/yuanlin/opencv-2.4.13.4/build")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(image img.cpp)

# link opencv
#set(OpenCV_DIR "/usr/local/lib)   

target_link_libraries(image ${OpenCV_LIBS})