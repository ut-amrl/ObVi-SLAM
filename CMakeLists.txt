cmake_minimum_required(VERSION 3.1.0)
project(ut_vslam)

#set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-std=c++17 -lstdc++fs")
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(CONSTRAIN_ELLIPSOID_ORIENTATION "Constrain ellipsoid orientation to yaw only" ON)
#option(CONSTRAIN_ELLIPSOID_ORIENTATION "Constrain ellipsoid orientation to yaw only" OFF)
if (CONSTRAIN_ELLIPSOID_ORIENTATION)
    MESSAGE("Constraining ellipsoid orientation")
    add_definitions(-DCONSTRAIN_ELLIPSOID_ORIENTATION)
else ()
    MESSAGE("Not constraining ellipsoid orientation")
endif ()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

OPTION(GENERATE_SHARED_LIB_UNITTESTS,
        "Option to enable generation of shared library unit tests"
        OFF)

find_package(PCL 1.3 REQUIRED)
# Recommended to use Ceres 1.14
# find_package(Ceres 1.14 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4.2 EXACT REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)

#find_package(darknet_ros_msgs)
#find_package(catkin REQUIRED COMPONENTS darknet_ros_msgs)

include_directories(include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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
        ${Boost_FILESYSTEM_LIBRARY})

include_directories(src/shared)
add_subdirectory(src/shared)
include_directories(src)

add_library(ut_vslam
        src/evaluation/evaluation_utils.cpp
        src/evaluation/trajectory_interpolation_utils.cpp
        src/refactoring/bounding_box_frontend/pending_object_estimator.cpp
        src/refactoring/factors/bounding_box_factor.cpp
        src/refactoring/factors/pairwise_2d_feature_cost_functor.cpp
        src/refactoring/factors/independent_object_map_factor.cpp
        src/refactoring/factors/relative_pose_factor.cpp
        src/refactoring/factors/reprojection_cost_functor.cpp
        src/refactoring/factors/shape_prior_factor.cpp
        src/refactoring/long_term_map/long_term_object_map_extraction.cpp
        src/refactoring/optimization/jacobian_extraction.cpp
        src/refactoring/types/vslam_obj_opt_types_refactor.cpp
        src/refactoring/visual_feature_processing/orb_output_low_level_feature_reader.cpp
        src/types/timestamped_data_to_frames_utils.cpp
        )

target_compile_features(ut_vslam PRIVATE cxx_std_17)

ROSBUILD_ADD_EXECUTABLE(offline_object_visual_slam_main src/refactoring/offline_object_visual_slam_main.cpp)
target_link_libraries(offline_object_visual_slam_main ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(initialize_traj_and_feats_from_orb_out src/data_preprocessing_utils/unproject_main.cpp)
target_link_libraries(initialize_traj_and_feats_from_orb_out ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(orb_trajectory_sparsifier src/data_preprocessing_utils/orb_trajectory_sparsifier.cpp)
target_link_libraries(orb_trajectory_sparsifier ut_vslam ${LIBS})

#
#ROSBUILD_ADD_EXECUTABLE(approx_depth_bounding_box_extractor src/data_preprocessing_utils/approx_depth_bounding_box_extractor.cpp)
#target_link_libraries(approx_depth_bounding_box_extractor ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(localization_rosbag_extraction_and_interpolation_for_bounding_boxes src/data_preprocessing_utils/localization_rosbag_extraction_and_interpolation_for_bounding_boxes.cpp)
target_link_libraries(localization_rosbag_extraction_and_interpolation_for_bounding_boxes ut_vslam ${LIBS})


ROSBUILD_ADD_EXECUTABLE(test_det_call src/testing/test_obj_det_service.cpp)
target_link_libraries(test_det_call ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(visualize_jacobian_problem_feats src/refactoring/visualize_jacobian_problem_feats.cpp)
target_link_libraries(visualize_jacobian_problem_feats ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(write_configuration src/refactoring/configuration/write_configuration.cpp)
target_link_libraries(write_configuration ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(interpolate_poses_with_required_nodes src/evaluation/interpolate_poses_with_required_nodes.cpp)
target_link_libraries(interpolate_poses_with_required_nodes ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(oa_slam_data_generator src/evaluation/oa_slam/oa_slam_data_generator.cpp)
target_link_libraries(oa_slam_data_generator ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(trajectory_metrics_generator src/evaluation/trajectory_metrics_generator.cpp)
target_link_libraries(trajectory_metrics_generator ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(utvslam_out_to_poses_by_timestamp src/evaluation/trajectory_formatting/utvslam_out_to_poses_by_timestamp.cpp)
target_link_libraries(utvslam_out_to_poses_by_timestamp ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(debug_jacobian_hessian_diagonal src/debugging_utils/debug_jacobian_hessian_diagonal.cpp)
target_link_libraries(debug_jacobian_hessian_diagonal ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(ltm_extraction_only src/refactoring/ltm_extraction_only.cpp)
target_link_libraries(ltm_extraction_only ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(display_ltm src/refactoring/display_ltm.cpp)
target_link_libraries(display_ltm ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(waypoint_timestamp_extractor src/data_preprocessing_utils/waypoint_timestamp_extractor.cpp)
target_link_libraries(waypoint_timestamp_extractor ut_vslam ${LIBS})

ROSBUILD_ADD_EXECUTABLE(sequence_file_writer src/data_preprocessing_utils/sequence_file_writer.cpp)
target_link_libraries(sequence_file_writer ut_vslam ${LIBS})

IF (GENERATE_UNITTESTS)
    MESSAGE("Adding test executable")
    SET(UT_VSLAM_UNITTEST_NAME "ut_vslam_tests"
            CACHE STRING "Name of compiled unit test executable")
    ADD_EXECUTABLE(${UT_VSLAM_UNITTEST_NAME}
            test/file_io/cv_file_storage/config_file_storage_io_tests.cc
            test/file_io/cv_file_storage/sequence_file_storage_io_tests.cc
            test/file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io_tests.cc)
    TARGET_LINK_LIBRARIES(${UT_VSLAM_UNITTEST_NAME}
            gtest
            gtest_main
            ${LIBS})
ENDIF ()