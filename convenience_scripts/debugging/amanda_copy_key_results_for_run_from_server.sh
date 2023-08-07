#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
results_root_directory=${root_data_dir}ut_vslam_results/
copied_from_server_dest_dir=${results_root_directory}copied_from_server/

server_results_dir=/robodata/aaadkins/ellipsoid_slam/eer_jackal/ut_vslam_results/

config_file_base_name="best_so_far_base7a_2"
sequence_file_base_name="evaluation_2023_07_v1"

make && python3 src/evaluation/copy_key_results_from_server.py \
    --server_results_dir ${server_results_dir} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --copied_from_server_dest_dir ${copied_from_server_dest_dir} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory} \
#    --force_rerun_interpolator \
#    --force_rerun_trajectory_formatter