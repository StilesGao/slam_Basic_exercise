
cmake_minimum_required(VERSION 3.9)
project(L5)

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

add_executable( computeORB computeORB.cpp )
target_link_libraries( computeORB ${OpenCV_LIBRARIES} )

add_executable( E2Rt E2Rt.cpp )
target_link_libraries( E2Rt ${Sophus_LIBRARIES}
        ${Eigen_LIBRARIES} )

add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA  ${Sophus_LIBRARIES}
        ${Eigen_LIBRARIES})
add_executable(rmse rmse.cpp)
target_link_libraries(rmse ${Sophus_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBRARIES})

