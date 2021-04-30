cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


find_package(Ceres REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(include 
                    ${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS})

set(LIBS stdc++fs
         glog 
         gflags 
         ${CERES_LIBRARIES} 
         ${OpenCV_LIBS}
         ${Pangolin_LIBRARIES})

add_library(ut_vslam src/vslam_io.cpp 
                     src/pairwise_2d_feature_cost_functor.cpp 
                     src/reprojection_cost_functor.cpp
                     src/slam_backend_solver.cpp 
                     src/structureless_ceres_visualization_callback.cpp 
                     src/vslam_util.cpp
                     src/pose_viewer.cpp
                     src/draw_epipolar_lines.cpp)
                     
target_compile_features(ut_vslam PRIVATE cxx_std_17)    # 17 required for file iteration 

add_executable(main src/main.cpp)
target_link_libraries(main ut_vslam ${LIBS} )

add_executable(playground src/playground.cpp)
target_link_libraries(playground ut_vslam ${LIBS} )