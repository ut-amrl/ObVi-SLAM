#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/
ut_vslam_out_root_dir=${root_data_dir}orb_slam_3_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
calibration_file_directory=${root_data_dir}calibration/
results_root_directory=${root_data_dir}ut_vslam_results/

odometry_topic="/jackal_velocity_controller/odom"

#config_file_base_name="3_higher_obj_thresh_lower_conf"
#sequence_file_base_name="20230218_1a_4a"
#config_file_base_name="4_higher_obj_thresh_lower_conf_two_phase_off"
#config_file_base_name="8_two_phase_off"
#config_file_base_name="amazon_0523_base"
#sequence_file_base_name="amazon_0523_v0"
#config_file_base_name="base7_x_y_merge"
#config_file_base_name="base7a_2_v2"

#config_file_base_name=base7a_1
#config_file_base_name=base7a_1_fallback
#config_file_base_name=base7a_1_fallback_v2
#config_file_base_name=base7a_2
#config_file_base_name=base7a_2_fallback
#config_file_base_name=base7a_2_fallback_v2
#config_file_base_name=base7a_2_v2
#config_file_base_name=base7a_3
#config_file_base_name=base7a_3_fallback
#config_file_base_name=base7a_4_fallback
#config_file_base_name=base7b
#config_file_base_name=base7b_1_fallback
#config_file_base_name=base7b_2
#config_file_base_name=base7b_2_fallback
#config_file_base_name=base7_vis_feat_only
#config_file_base_name=base7_x_y_merge

# Best one I think....
config_file_base_name=base7a_1_fallback_a_2


#config_file_base_name=base7a_1_fallback_a_3
#config_file_base_name=base7a_1_fallback_b_2
#config_file_base_name=base7a_1_fallback_c_2
#config_file_base_name=base7a_2_fallback_b_2
#config_file_base_name=base7a_2_fallback_c_2
#config_file_base_name=base7a_2_v3
#config_file_base_name=base7b_1_fallback_b_2
#config_file_base_name=base7b_2_fallback_a_2
#config_file_base_name=base7b_2_fallback_a_3
#config_file_base_name=base7b_2_fallback_b_1
#config_file_base_name=base7b_2_fallback_c_1
#config_file_base_name=base7b_fallback
#config_file_base_name=base7_fallback_enabled
#config_file_base_name=base7_fallback_enabled_v2



#config_file_base_name="base7a_1_fallback_a_2_v2"
#config_file_base_name="base7a_1_fallback_b_2_v2"
#config_file_base_name="base7a_1_fallback_c_2_v2"
#config_file_base_name="base7a_2_fallback_a_2"
#config_file_base_name="base7a_2_fallback_b_1"
#config_file_base_name="base7a_2_fallback_b_2_v2"
#config_file_base_name="base7a_2_fallback_b_3"
#config_file_base_name="base7a_2_fallback_c_2_v2"

#config_file_base_name="no_shape_prior_base7a_1_fallback_a_2"
#config_file_base_name="no_vis_feats_base7a_1_fallback_a_2"
config_file_base_name="no_ltm_base7a_1_fallback_a_2"

sequence_file_base_name="evaluation_2023_07_v1"

make && python3 src/evaluation/compute_metrics_for_ut_vslam.py \
    --rosbag_file_directory=${rosbag_file_directory} \
    --trajectory_sequence_file_directory=${trajectory_sequence_file_directory} \
    --sequence_file_base_name=${sequence_file_base_name} \
    --lego_loam_out_root_dir=${lego_loam_out_root_dir} \
    --calibration_file_directory=${calibration_file_directory} \
    --orb_post_process_base_directory=${orb_post_process_base_directory} \
    --config_file_base_name=${config_file_base_name} \
    --odometry_topic ${odometry_topic} \
    --results_root_directory=${results_root_directory} \
    --force_rerun_metrics_generator \
#    --force_rerun_interpolator \
#    --force_rerun_trajectory_formatter