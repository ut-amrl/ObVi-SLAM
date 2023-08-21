#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
orb_post_process_base_directory=${root_data_dir}orb_post_process
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
results_root_directory=${root_data_dir}oa_slam_out/


gt_ellipsoids_file=${rosbag_file_directory}readable_labeling__2023_05_12_10_40_16.csv

sequence_file_base_name="evaluation_2023_07_v1_0819"
cp sequences/evaluation_2023_07_v1.json sequences/${sequence_file_base_name}.json

make && python3 src/evaluation/compute_object_metrics_for_oa_slam.py \
    --gt_ellipsoids_file=${gt_ellipsoids_file} \
    --results_root_directory=${results_root_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --calibration_file_directory=${calibration_file_directory} \
    --force_rerun_metrics_generator

