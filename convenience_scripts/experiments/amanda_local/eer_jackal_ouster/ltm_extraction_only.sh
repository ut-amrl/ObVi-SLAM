#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

sequence_file_base_name="jacobian_debug2"
config_file_base_name="base"
number_in_sequence=0
#min_frame_id=394
#max_frame_id=547

make
python3 src/evaluation/ltm_extractor_only.py \
    --config_file_directory ${config_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --number_in_sequence_arg_name ${number_in_sequence} \
    --output_jacobian_debug --log_to_file \
#    --min_frame_id ${min_frame_id} --max_frame_id ${max_frame_id}
