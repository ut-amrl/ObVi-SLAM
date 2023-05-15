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
#config_file_base_name="10_two_phase_off_first_100"
config_file_base_name="8_two_phase_off"
#config_file_base_name="9_two_phase_off"
#config_file_base_name="8_two_phase_off_first_250"
#config_file_base_name="4_higher_obj_thresh_lower_conf_fix_traj"
#config_file_base_name="4_higher_obj_two_off_pgo_off"
#config_file_base_name="test_higher_obj_obs_for_opt"
#config_file_base_name="1_first_25"
number_in_sequence=0
#min_frame_id=394
#max_frame_id=547

# Tried min/max combos for 8_two_phase_off
# 500   516   No problem
# 503   516   No problem
# 504   515   No problem
# 0     515   Problem
# 504   550   Problem
# 250   516   Problem
# 375   516   No problem
# 350   516   No problem
# 340   516   No problem
# 335   516   No problem
# 332   516   No problem
# 331   516   No problem
# 312   516   Problem
# 330   516   Problem
# 504   517   No problem
# 504   518   No problem
# 504   519   No problem
# 504   520   No problem
# 504   521   No problem
# 504   522   No problem
# 504   523   No problem
# 504   524   No problem
# 504   525   No problem
# 504   526   No problem
# 504   527   No problem
# 504   528   No problem
# 504   529   No problem
# 504   530   No problem
# 504   540   No problem
# 504   545   No problem
# 504   546   No problem
# 504   547   No problem
# 504   548   Problem
# 360   548   Problem
# 360   547   Problem
# 390   547   Problem (2 problem objects 29 and 45)
# 392   547   Problem (2 problem objects 29 and 45)
# 389   547   No problem
# 394   547   No problem
# 391   547   Problem
# 393   547   Problem
# 395   547   Problem
# 331   547   No problem


python3 src/evaluation/ltm_extractor_only.py \
    --config_file_directory ${config_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --number_in_sequence_arg_name ${number_in_sequence} \
    --output_jacobian_debug --log_to_file \
#    --min_frame_id ${min_frame_id} --max_frame_id ${max_frame_id}
