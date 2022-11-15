cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

#set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-std=c++17 -lstdc++fs")
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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
#find_package(darknet_ros_msgs)
#find_package(catkin REQUIRED COMPONENTS darknet_ros_msgs)

include_directories(include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src/yaml-cpp/include)

option(TESTING "Build tests" ON)

set(LIBS
        roslib
        roscpp
        rosbag
        #        darknet_ros_msgs
        stdc++fs
        glog
        gflags
        cv_bridge
        amrl_shared_lib
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/src/yaml-cpp/build/libyaml-cpp.so)

include_directories(src/shared)
add_subdirectory(src/shared)
include_directories(src)

add_library(ut_vslam src/vslam_io.cpp
        src/pairwise_2d_feature_cost_functor.cpp
        src/reprojection_cost_functor.cpp
        src/slam_backend_solver.cpp
        src/pose_viewer.cpp
        src/shape_prior_factor.cpp
        src/bounding_box_factor.cpp
        src/object_slam_backend_solver.cpp
        src/vslam_type_conversion_util.cpp
        src/vslam_types_math_util.cpp
        src/vslam_noise_util.cpp
        src/ellipsoid_utils.cpp
        src/synthetic_problem/noise_addition_utils.cpp
        src/synthetic_problem/synthetic_problem_generator.cpp
        src/synthetic_problem/synthetic_problem_construction_utils.cpp
        src/refactoring/factors/bounding_box_factor.cpp
        src/refactoring/factors/pairwise_2d_feature_cost_functor.cpp
        src/refactoring/factors/independent_object_map_factor.cpp
        src/refactoring/factors/reprojection_cost_functor.cpp
        src/refactoring/factors/shape_prior_factor.cpp
        src/refactoring/types/vslam_obj_opt_types_refactor.cpp
        )

target_compile_features(ut_vslam PRIVATE cxx_std_17)

#ROSBUILD_ADD_EXECUTABLE(main src/main.cpp)
#target_link_libraries(main ut_vslam ${LIBS})
#
#ROSBUILD_ADD_EXECUTABLE(structured_main src/structured_main.cpp)
#target_link_libraries(structured_main ut_vslam ${LIBS})
#
ROSBUILD_ADD_EXECUTABLE(ellipsoid_estimator_main src/ellipsoid_estimator_main.cpp)
target_link_libraries(ellipsoid_estimator_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(ellipsoid_estimator_real_data_main src/ellipsoid_estimator_real_data_main.cpp)
target_link_libraries(ellipsoid_estimator_real_data_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(offline_object_visual_slam_main src/refactoring/offline_object_visual_slam_main.cpp)
target_link_libraries(offline_object_visual_slam_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(iv_bbox_main src/iv_bbox/main.cpp)
target_link_libraries(iv_bbox_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(iv_bbox_sync_main src/iv_bbox/sync_main.cpp)
target_link_libraries(iv_bbox_sync_main ut_vslam ${LIBS})

#
#ROSBUILD_ADD_EXECUTABLE(approx_depth_bounding_box_extractor src/data_preprocessing_utils/approx_depth_bounding_box_extractor.cpp)
#target_link_libraries(approx_depth_bounding_box_extractor ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(localization_rosbag_extraction_and_interpolation_for_bounding_boxes src/data_preprocessing_utils/localization_rosbag_extraction_and_interpolation_for_bounding_boxes.cpp)
target_link_libraries(localization_rosbag_extraction_and_interpolation_for_bounding_boxes ut_vslam ${LIBS})