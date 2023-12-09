#!/bin/bash
rosparam set /use_sim_time false

SLAM_DIR=/root/LTOV-SLAM-Evaluation/ut_vslam/

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
config_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/config/
trajectory_sequence_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/

calibration_file_directory=${root_data_dir}calibration/
rosbag_file_directory=${root_data_dir}original_data/
orb_slam_out_directory=${root_data_dir}orb_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/

sequence_file_base_name="evaluation_2023_07_v1"

cd $SLAM_DIR

config_file_base_name="base7a_1_fallback_a_2"
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
    --output_bb_assoc --record_viz_rosbag --log_to_file

kill -9 $yolo_pid
