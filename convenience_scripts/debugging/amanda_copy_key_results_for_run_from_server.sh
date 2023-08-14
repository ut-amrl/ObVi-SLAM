#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
results_root_directory=${root_data_dir}ut_vslam_results/
copied_from_server_dest_dir=${results_root_directory}copied_from_server/

server_results_dir=/robodata/aaadkins/ellipsoid_slam/eer_jackal/ut_vslam_results/


#config_file_base_name="base7a_2_v3"
#config_file_base_name="base7b_1_fallback"
#config_file_base_name="base7b_2"

#config_file_base_name="base7a_4_fallback"
sequence_file_base_name="evaluation_2023_07_v1"

#config_file_strings=("base7a_1_fallback_a_2" "base7a_1_fallback_a_3" "base7a_1_fallback_b_2" "base7a_1_fallback_c_2" "base7a_2_fallback_b_2" "base7a_2_fallback_c_2" "base7a_2_v3" "base7b_1_fallback_b_2" "base7b_2_fallback_a_2" "base7b_2_fallback_a_3" "base7b_2_fallback_b_1" "base7b_2_fallback_c_1" "base7b_fallback" "base7_fallback_enabled" "base7_fallback_enabled_v2")

#config_file_strings=("base7a_1_fallback_a_2_v2" "base7a_1_fallback_b_2_v2" "base7a_1_fallback_c_2_v2" "base7a_2_fallback_a_2" "base7a_2_fallback_b_1" "base7a_2_fallback_b_2_v2" "base7a_2_fallback_b_3" "base7a_2_fallback_c_2_v2")
config_file_strings=("base7a_2_fallback_b_2_v2" "base7a_2_fallback_b_3" "base7a_2_fallback_c_2_v2")

for config_file_base_name in ${config_file_strings[@]}; do
  echo ${config_file_base_name}
  python3 src/evaluation/copy_key_results_from_server.py \
    --server_results_dir ${server_results_dir} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --copied_from_server_dest_dir ${copied_from_server_dest_dir} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory} \
#    --force_rerun_interpolator \
#    --force_rerun_trajectory_formatter
done

#make && python3 src/evaluation/copy_key_results_from_server.py \
#    --server_results_dir ${server_results_dir} \
#    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
#    --sequence_file_base_name=${sequence_file_base_name} \
#    --copied_from_server_dest_dir ${copied_from_server_dest_dir} \
#    --config_file_base_name=${config_file_base_name} \
#    --results_root_directory=${results_root_directory} \
##    --force_rerun_interpolator \
##    --force_rerun_trajectory_formatter