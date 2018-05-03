cmake_minimum_required(VERSION 3.9)
project(L6)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( Pangolin REQUIRED )
find_package( OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/Sophus/sophus )

set( Sophus_LIBRARIES libSophus.so )

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow  ${Sophus_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES} )

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${Sophus_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES})