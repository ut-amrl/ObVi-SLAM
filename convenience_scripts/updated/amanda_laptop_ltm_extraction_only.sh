#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

#sequence_file_base_name="1676766718_only"
#config_file_base_name="two_phase_off"
#sequence_file_base_name="20230218_1a_4a"
#sequence_file_base_name="high_res_7326_7619"
#sequence_file_base_name="1675896005"
#sequence_file_base_name="high_res_7326_20230218_1a"
sequence_file_base_name="high_res_20230218_1a_7326"
config_file_base_name="10_two_phase_off_first_100"
#config_file_base_name="9_two_phase_off"
#config_file_base_name="8_two_phase_off_first_250"
#config_file_base_name="4_higher_obj_thresh_lower_conf_fix_traj"
#config_file_base_name="4_higher_obj_two_off_pgo_off"
#config_file_base_name="test_higher_obj_obs_for_opt"
#config_file_base_name="1_first_25"
number_in_sequence=0

python3 src/evaluation/ltm_extractor_only.py \
    --config_file_directory ${config_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --number_in_sequence_arg_name ${number_in_sequence} \
    --output_jacobian_debug --log_to_file
