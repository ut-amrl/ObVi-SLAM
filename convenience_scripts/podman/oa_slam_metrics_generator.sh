#!/bin/bash

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/
oa_slam_out_root_dir=${root_data_dir}oa_slam_out/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/

sequence_file_base_name="high_res_20230218_1a_7326"

make -j4 && python3 src/evaluation/compute_metrics_for_oaslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --oa_slam_out_root_dir=${oa_slam_out_root_dir} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --odometry_topic ${odometry_topic} \
    --force_rerun_metrics_generator \
#    --force_rerun_interpolator \

