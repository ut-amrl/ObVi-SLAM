#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
rosbag_file_directory=${root_data_dir}original_data/
bounding_box_post_process_base_directory=${root_data_dir}bounding_box_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

sequence_file_base_name="evaluation_2023_07_v1"
config_file_base_name="base7b"

python3 src/evaluation/write_bounding_boxes_to_file_for_sequence.py \
    --config_file_directory ${config_file_directory} \
    --bounding_box_post_process_base_directory ${bounding_box_post_process_base_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
