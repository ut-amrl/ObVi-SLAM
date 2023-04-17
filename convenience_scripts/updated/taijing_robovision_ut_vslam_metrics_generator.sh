#!/bin/bash

root_data_dir=/robodata/taijing/object-slam/oslam/
config_file_directory=/home/tiejean/projects/ut_semantic_vslam/config/taijing/
trajectory_sequence_file_directory=/home/tiejean/projects/ut_semantic_vslam/sequences/

calibration_file_directory=${root_data_dir}calibration/
rosbag_file_directory=${root_data_dir}original_data/
orb_slam_out_directory=${root_data_dir}orb_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

sequence_file_base_name="taijing_20230218_1a"
# config_file_base_name="7"
# config_file_base_name="7_epipolar"
config_file_base_name="8_replication"

python3 src/evaluation/compute_metrics_for_ut_vslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --orb_post_process_base_directory=${orb_post_process_base_directory} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory}