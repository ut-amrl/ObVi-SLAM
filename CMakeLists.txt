cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Ceres REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS})

option(TESTING "Build tests" ON)

set(LIBS
        roslib
        roscpp
        rosbag
        stdc++fs
        glog
        gflags
        amrl_shared_lib
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES})

include_directories(src/shared)
add_subdirectory(src/shared)
include_directories(src)

add_library(ut_vslam src/vslam_io.cpp
        src/pairwise_2d_feature_cost_functor.cpp
        src/reprojection_cost_functor.cpp
        src/slam_backend_solver.cpp
        src/pose_viewer.cpp
        src/vslam_util.cpp
        src/shape_prior_factor.cpp
        src/bounding_box_factor.cpp
        src/object_slam_backend_solver.cpp
        src/synthetic_problem/noise_addition_utils.cpp
        src/synthetic_problem/synthetic_problem_generator.cpp
        src/synthetic_problem/synthetic_problem_construction_utils.cpp
        )

target_compile_features(ut_vslam PRIVATE cxx_std_17)

ROSBUILD_ADD_EXECUTABLE(main src/main.cpp)
target_link_libraries(main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(structured_main src/structured_main.cpp)
target_link_libraries(structured_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(ellipsoid_estimator_main src/ellipsoid_estimator_main.cpp)
target_link_libraries(ellipsoid_estimator_main ut_vslam ${LIBS})