cmake_minimum_required(VERSION 2.8)
project(stereotest)
find_package(OpenCV REQUIRED)
add_executable(stereo depth.cpp)
target_link_libraries(stereo ${OpenCV_LIBS})

