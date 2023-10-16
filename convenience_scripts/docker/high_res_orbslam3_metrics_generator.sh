#!/bin/bash

root_data_dir=/root/ObVi-SLAM-Evaluation/data/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/root/ObVi-SLAM-Evaluation/ObVi-SLAM/sequences/
orb_slam_3_out_root_dir=${root_data_dir}orb_slam_3_out/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
odometry_topic="/jackal_velocity_controller/odom"

sequence_file_base_name="evaluation_2023_07_v1"

make && python3 src/evaluation/compute_metrics_for_orbslam3.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --orb_slam_3_out_root_dir=${orb_slam_3_out_root_dir} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --odometry_topic ${odometry_topic} \
    --force_rerun_metrics_generator \
#    --force_rerun_interpolator \

