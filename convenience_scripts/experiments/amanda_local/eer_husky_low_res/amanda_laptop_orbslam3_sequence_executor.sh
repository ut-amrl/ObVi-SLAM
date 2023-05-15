#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

orb_slam_configuration_file=/home/amanda/workspaces/ORB_SLAM3/Examples/Stereo/husky_zed_rectified.yaml
orb_slam_vocabulary_file=/home/amanda/workspaces/ORB_SLAM3/Vocabulary/ORBvoc.txt
orb_slam_3_out_root_dir=${root_data_dir}orb_slam_3_out/

sequence_file_base_name="3059_9589"

python3 src/evaluation/run_multi_session_orb_slam_3.py \
    --orb_slam_configuration_file=${orb_slam_configuration_file} \
    --orb_slam_vocabulary_file=${orb_slam_vocabulary_file} \
    --orb_slam_3_out_root_dir ${orb_slam_3_out_root_dir} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --generate_map_file