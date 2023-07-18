#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

#sequence_file_base_name="end_of_may_demo_v1"
sequence_file_base_name="amazon_0523_v0_first_2"
#config_file_base_name="tentative_11"
#config_file_base_name="tentative_11_wider_trees_lamps"
#config_file_base_name="base4_old_single_thread_higher_jacobian_threshold"
config_file_base_name="base4_old_single_thread_higher_jacobian_threshold_wide_merge"


relevant_results_dir=${results_root_directory}amazon_0523_v0_bag2_only/base4_old_single_thread_higher_jacobian_threshold/0__2023_05_12_11_00_35/
reprocessing_dir=${relevant_results_dir}reprocessing/

param_prefix=""
intrinsics_file=${calibration_file_directory}camera_matrix.txt
extrinsics_file=${calibration_file_directory}extrinsics.txt
reprocessing_ut_vslam_out_dir=${reprocessing_dir}ut_vslam_out/
long_term_map_output=${reprocessing_ut_vslam_out_dir}long_term_map.json
#long_term_map_input=""
ltm_opt_jacobian_info_directory=${reprocessing_dir}jacobian_debugging_out/
params_config_file=${config_file_directory}${config_file_base_name}.json
logs_directory=${reprocessing_dir}logs/
input_checkpoint_file=${relevant_results_dir}/checkpoints/pose_graph_state_checkpoint_pre_optimization_643_attempt_0.json
poses_by_node_id_file=${root_data_dir}orb_post_process/sparsified_ut_vslam_in/base4_old_single_thread_higher_jacobian_threshold/_2023_05_12_11_00_35/poses/initial_robot_poses_by_node.txt

mkdir -p ${logs_directory}
mkdir -p ${reprocessing_ut_vslam_out_dir}
mkdir -p ${ltm_opt_jacobian_info_directory}

make

./bin/run_opt_from_pg_state --param_prefix=${param_prefix} \
--intrinsics_file=${intrinsics_file} \
--extrinsics_file=${extrinsics_file} \
--long_term_map_output=${long_term_map_output} \
--ltm_opt_jacobian_info_directory=${ltm_opt_jacobian_info_directory} \
--params_config_file=${params_config_file} \
--logs_directory=${logs_directory} \
--input_checkpoint_file=${input_checkpoint_file} \
--poses_by_node_id_file=${poses_by_node_id_file}
#--long_term_map_input=${long_term_map_input} \


