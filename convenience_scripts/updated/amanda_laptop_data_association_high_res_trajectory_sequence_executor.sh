#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_flir/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

sequence_file_base_name="1678401418_only"
config_file_base_name="7_data_assoc_only_flir_freq"

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
    --output_jacobian_debug --output_bb_assoc --run_rviz --record_viz_rosbag --log_to_file
