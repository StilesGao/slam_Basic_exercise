cmake_minimum_required( VERSION 2.8 )
project( l4 )

set ( CMAKE_CXX_COMPILER "g++" )
set ( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( Pangolin REQUIRED )
find_package( OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable( disparity disparity.cpp )
target_link_libraries(disparity ${Pangolin_LIBRARIES}
                                ${OpenCV_LIBRARIES})

add_executable( gaussnewton gaussnewton.cpp )
target_link_libraries(gaussnewton ${Pangolin_LIBRARIES}
                                    ${OpenCV_LIBRARIES})

add_executable( undistort_image undistort_image.cpp )
target_link_libraries( undistort_image ${Pangolin_LIBRARIES}
                                        ${OpenCV_LIBRARIES})