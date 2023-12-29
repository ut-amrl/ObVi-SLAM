#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
orb_post_process_base_directory=${root_data_dir}orb_post_process
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
results_root_directory=${root_data_dir}ut_vslam_results/

odometry_topic="/jackal_velocity_controller/odom"


#config_file_base_name="update_rev_single_phase_manual_vf_reduced_tracking"
sequence_file_base_name="evaluation_2023_07_v1"

config_file_base_name="update_rev_lowerererish_conv_thresholds_manual_feat_adj_tighterer_vo_v4"

python3 src/evaluation/timing_analysis.py \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --config_file_base_name=${config_file_base_name} \
    --results_root_directory=${results_root_directory}
#    --force_rerun_interpolator \
#    --force_rerun_trajectory_formatter