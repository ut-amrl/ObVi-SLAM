#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
calibration_file_directory=/robodata/taijing/object-slam/calibration/husky_zed_resolution_1_scale_0_5/
config_file_directory=/home/tiejean/projects/ut_semantic_vslam/config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=/robodata/taijing/object-slam/
orb_post_process_base_directory=${root_data_dir}vslam_in_sparse/
results_root_directory=${root_data_dir}vslam_out/
trajectory_sequence_file_directory=/home/tiejean/projects/ut_semantic_vslam/sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

sequence_file_base_name="high_res_20230218_1a_7326"
config_file_base_name="7"

python3 src/evaluation/compute_metrics_for_ut_vslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --orb_post_process_base_directory=${orb_post_process_base_directory} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory}