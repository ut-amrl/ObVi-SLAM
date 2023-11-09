#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
bounding_box_post_process_base_directory=${root_data_dir}bounding_box_data/

odometry_topic="/jackal_velocity_controller/odom"

#sequence_file_base_name="end_of_may_demo_v1"
#sequence_file_base_name="_2023_06_22_21_51_46"
#sequence_file_base_name="evaluation_2023_07_v1"
sequence_file_base_name="short_sequence_for_example"
#config_file_base_name="tentative_11"
#config_file_base_name="tentative_11_wider_trees_lamps"
#config_file_base_name="base7a_2_fallback_lower_rot_cov"
#config_file_base_name="base7a_2_fallback_lower_rot_cov"
config_file_base_name="base7a_1_fallback_a_2_first_100_wide_shape_prior_for_second_no_feats_red_ltm"

python3 src/evaluation/ltm_trajectory_sequence_executor.py \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --odometry_topic ${odometry_topic} \
    --output_jacobian_debug --output_bb_assoc --run_rviz --record_viz_rosbag --log_to_file \
    --output_checkpoints \
    --bounding_box_post_process_base_directory ${bounding_box_post_process_base_directory} \
#    --output_ellipsoid_debug
#