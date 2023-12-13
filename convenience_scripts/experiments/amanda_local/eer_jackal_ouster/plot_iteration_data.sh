#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
results_root_directory=${root_data_dir}ut_vslam_results/

config_file_base_name="revision_speed_up_factors2"
sequence_file_base_name="evaluation_2023_07_v1"

make && python3 src/evaluation/iteration_plotter.py \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory} \
