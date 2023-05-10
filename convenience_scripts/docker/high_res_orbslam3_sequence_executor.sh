#!/bin/bash

root_data_dir="${HOME}/data/"
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=${HOME}/ut_vslam/sequences/

orb_slam_configuration_file=${HOME}/ORB_SLAM3/Examples/Stereo/high_res_husky_zed_rectified.yaml
orb_slam_vocabulary_file=${HOME}/ORB_SLAM3/Vocabulary/ORBvoc.txt
orb_slam_3_out_root_dir=${root_data_dir}orb_slam_3_out/

sequence_file_base_name="high_res_20230218_1a"

python3 src/evaluation/run_multi_session_orb_slam_3.py \
    --orb_slam_configuration_file=${orb_slam_configuration_file} \
    --orb_slam_vocabulary_file=${orb_slam_vocabulary_file} \
    --orb_slam_3_out_root_dir ${orb_slam_3_out_root_dir} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --sequence_file_base_name ${sequence_file_base_name} \
#    --generate_map_file