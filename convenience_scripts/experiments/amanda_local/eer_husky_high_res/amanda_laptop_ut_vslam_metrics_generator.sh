#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
ut_vslam_out_root_dir=${root_data_dir}orb_slam_3_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
results_root_directory=${root_data_dir}ut_vslam_results/

#config_file_base_name="3_higher_obj_thresh_lower_conf"
#sequence_file_base_name="20230218_1a_4a"
#config_file_base_name="4_higher_obj_thresh_lower_conf_two_phase_off"
config_file_base_name="8_two_phase_off"
sequence_file_base_name="high_res_20230218_1a_7326"

make && python3 src/evaluation/compute_metrics_for_ut_vslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --orb_post_process_base_directory=${orb_post_process_base_directory} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory} \
#    --force_rerun_interpolator \
#    --force_rerun_metrics_generator \
#    --force_rerun_trajectory_formatter