#!/bin/bash

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
rosbag_file_directory=${root_data_dir}original_data/
# TODO change me
trajectory_sequence_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/amazon_0523/
orb_post_process_base_directory=${root_data_dir}orb_post_process
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
results_root_directory=${root_data_dir}ut_vslam_results/
odometry_topic="/jackal_velocity_controller/odom"

config_file_base_name="amazon_0523_base"
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
    --odometry_topic ${odometry_topic}
#    --force_rerun_interpolator \
#    --force_rerun_metrics_generator \
#    --force_rerun_trajectory_formatter