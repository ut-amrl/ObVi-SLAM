#!/bin/bash

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
rosbag_file_directory=${root_data_dir}original_data/
trajectory_sequence_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/

orb_slam_configuration_file=/root/LTOV-SLAM-Evaluation/ORB_SLAM3/Examples/Stereo/high_res_jackal_zed_rectified.yaml
orb_slam_vocabulary_file=/root/LTOV-SLAM-Evaluation/ORB_SLAM3/Vocabulary/ORBvoc.txt
orb_slam_3_out_root_dir=${root_data_dir}orb_slam_3_out/

#sequence_file_base_name="20230218_1a_4a"
sequence_file_base_name="evaluation_2023_07_v1"


(rosrun image_transport republish compressed in:=/zed2i/zed_node/left/image_rect_color raw out:=/camera/left/image_raw) &
left_republish_pid=$!

(rosrun image_transport republish compressed in:=/zed2i/zed_node/right/image_rect_color raw out:=/camera/right/image_raw) &
right_republish_pid=$!

sleep 5

python3 src/evaluation/run_multi_session_orb_slam_3.py \
    --orb_slam_configuration_file=${orb_slam_configuration_file} \
    --orb_slam_vocabulary_file=${orb_slam_vocabulary_file} \
    --orb_slam_3_out_root_dir ${orb_slam_3_out_root_dir} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --generate_map_file

kill -9 $left_republish_pid
kill -9 $right_republish_pid
