cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

include_directories(include)

set(LIBS glog gflags ${CERES_LIBRARIES})

add_library(ut_vslam src/vslam_io.cpp src/pairwise_2d_feature_cost_functor.cpp src/reprojection_cost_functor.cpp src/slam_backend_solver.cpp)
target_compile_features(ut_vslam PRIVATE cxx_std_17)    # 17 required for file iteration 

add_executable(main src/main.cpp)
target_link_libraries(main ut_vslam ${LIBS} stdc++fs)