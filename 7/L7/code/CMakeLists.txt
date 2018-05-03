cmake_minimum_required(VERSION 3.9)
project(L7)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#Pangilin
find_package( Pangolin REQUIRED )
include_directories(${Pangolin_INCLUDE_DIRS})

#opencv
find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Eigen
include_directories("/usr/include/eigen3")

#Sophus
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/Sophus/sophus )
set( Sophus_LIBRARIES libSophus.so )

#g2o
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
		g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
		g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
		g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )



add_executable(directBA directBA.cpp)
target_link_libraries(directBA
		${G2O_LIBS}
		${CHOLMOD_LIBRARIES}
		${Sophus_LIBRARIES}
		${Eigen_LIBRARIES}
		${OpenCV_LIBRARIES}
		${Pangolin_LIBRARIES}
		)

