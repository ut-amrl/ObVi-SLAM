#!/bin/bash

SLAM_DIR=/root/LTOV-SLAM-Evaluation/ut_vslam/

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
calibration_file_directory=${root_data_dir}calibration/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=${SLAM_DIR}sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

odometry_topic="/jackal_velocity_controller/odom"

config_file_base_name="base4"
sequence_file_base_name="amazon_0523_v0"

make && python3 src/evaluation/compute_metrics_for_ut_vslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --orb_post_process_base_directory=${orb_post_process_base_directory} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory} \
    --odometry_topic ${odometry_topic} \
#    --force_rerun_interpolator \
#    --force_rerun_metrics_generator \
#    --force_rerun_trajectory_formatter