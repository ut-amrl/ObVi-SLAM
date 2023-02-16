#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
calibration_file_directory=${root_data_dir}zed_calib/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

sequence_file_base_name="3059_9589"
config_file_base_name="1"
#config_file_base_name="test_higher_obj_obs_for_opt"
#config_file_base_name="1_first_25"

python3 convenience_scripts/updated/ltm_trajectory_sequence_executor.py \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --output_ellipsoid_debug --output_jacobian_debug --output_bb_assoc --run_rviz --record_viz_rosbag --log_to_file