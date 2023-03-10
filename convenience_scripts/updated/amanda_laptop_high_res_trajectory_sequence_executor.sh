#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=/home/amanda/workspaces/ut_vslam/config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

#sequence_file_base_name="1676766718_only"
#config_file_base_name="two_phase_off"
#sequence_file_base_name="20230218_1a_4a"
#sequence_file_base_name="high_res_7326_7619"
#sequence_file_base_name="1675896005"
#sequence_file_base_name="high_res_7326_20230218_1a"
sequence_file_base_name="high_res_20230218_1a_7326"
config_file_base_name="6_two_phase_off"
#config_file_base_name="4_higher_obj_thresh_lower_conf_fix_traj"
#config_file_base_name="4_higher_obj_two_off_pgo_off"
#config_file_base_name="test_higher_obj_obs_for_opt"
#config_file_base_name="1_first_25"

python3 src/evaluation/ltm_trajectory_sequence_executor.py \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --output_ellipsoid_debug --output_jacobian_debug --output_bb_assoc --run_rviz --record_viz_rosbag --log_to_file
