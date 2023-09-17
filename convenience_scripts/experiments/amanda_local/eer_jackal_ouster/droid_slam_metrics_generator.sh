#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
droid_slam_out_root_dir=${root_data_dir}droid_slam_out/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
odometry_topic="/jackal_velocity_controller/odom"

#sequence_file_base_name="20230218_1a_4a"
#sequence_file_base_name="evaluation_2023_07_v1_backup_2023_07_28_21_18"
#sequence_file_base_name="evaluation_2023_07_v1_2023_08_12_14_21"
#sequence_file_base_name="evaluation_2023_07_v1_2023_08_12_15_44"
sequence_file_base_name="evaluation_2023_07_v1"

cp sequences/evaluation_2023_07_v1.json sequences/${sequence_file_base_name}.json

make && python3 src/evaluation/compute_metrics_for_droidslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --droid_slam_out_root_dir=${droid_slam_out_root_dir} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --odometry_topic ${odometry_topic} \
    --force_rerun_metrics_generator \
#    --force_rerun_interpolator \

