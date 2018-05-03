cmake_minimum_required(VERSION 3.9)
project(code)
include_directories( "/usr/include/eigen3" )
set(CMAKE_CXX_STANDARD 11)

add_executable(qr_cholesty qr_cholesty.cpp)

add_executable( pose pose.cpp )