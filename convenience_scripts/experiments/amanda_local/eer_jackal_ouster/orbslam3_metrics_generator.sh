#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
orb_slam_3_out_root_dir=${root_data_dir}orb_slam_3_out/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
odometry_topic="/jackal_velocity_controller/odom"

#sequence_file_base_name="20230218_1a_4a"
sequence_file_base_name="orb_final_probably"

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

