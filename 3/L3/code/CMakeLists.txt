cmake_minimum_required( VERSION 2.8 )
project( draw_trajectory )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( Pangolin REQUIRED )
#find_package( Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
#include_directories( ${Sophus_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/Sophus/sophus )

set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

set( Sophus_LIBRARIES libSophus.so )

add_executable( draw_trajectory draw_trajectory.cpp )
target_link_libraries( draw_trajectory ${Pangolin_LIBRARIES}
                                        ${Sophus_LIBRARIES})

add_executable( RMSE rmse.cpp )
target_link_libraries( RMSE ${Pangolin_LIBRARIES}
                            ${Sophus_LIBRARIES})