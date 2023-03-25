#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
rosbag_file_directory=${root_data_dir}original_data/
results_root_directory=${root_data_dir}ut_vslam_results/

#rosbag_base_name="1677097326"
#num_in_sequence="1"
#num_in_sequence="old_old_1"
#num_in_sequence="old_1"
rosbag_base_name="1676766718"
num_in_sequence="0"
#num_in_sequence="actual_0"
sequence_file_base_name="high_res_20230218_1a_7326"
#config_file_base_name="4_higher_obj_thresh_lower_conf_two_phase_off"
config_file_base_name="7_two_phase_off"
bag_suffix=".bag"

rosbag_file=${orig_data_dir}${rosbag_base_name}${bag_suffix}
top_level_results_dir=${results_root_directory}${sequence_file_base_name}/${config_file_base_name}/${num_in_sequence}_${rosbag_base_name}/
ltm_opt_jacobian_info_directory=${top_level_results_dir}jacobian_debugging_out/


jacobian_residual_info_file=${ltm_opt_jacobian_info_directory}ordered_jacobian_residual_info.json
hessian_diag_entries_file=${ltm_opt_jacobian_info_directory}hessian_diag.csv
data_association_file=${top_level_results_dir}ut_vslam_out/data_association_results.json

echo ${jacobian_residual_info_file}

make && ./bin/debug_jacobian_hessian_diagonal \
  --jacobian_residual_info_file ${jacobian_residual_info_file} \
  --hessian_diag_entries_file ${hessian_diag_entries_file} \
  --data_association_file ${data_association_file}


