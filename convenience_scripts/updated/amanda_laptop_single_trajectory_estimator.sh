#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
calibration_file_directory=${root_data_dir}zed_calib/
config_file_directory=${root_data_dir}configs/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/

rosbag_base_name="1669743059"
sequence_file_base_name="test"
config_file_base_name="test2"
results_for_bag_dir_prefix="1_"

python3 src/evaluation/single_trajectory_estimator.py \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --rosbag_base_name ${rosbag_base_name} \
    --results_for_bag_dir_prefix ${results_for_bag_dir_prefix} \
    --output_ellipsoid_debug --output_jacobian_debug --output_bb_assoc
#    --long_term_map_bag_dir_arg_name ${long_term_map_bag_dir_arg_name}


