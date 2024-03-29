#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
camera_topics_file=/home/amanda/workspaces/ut_vslam/config/zed2i_left_camera_topics.txt
waypoint_topic_trigger=/autonomy_arbiter/joystick_stamp
waypoint_file_version=1

#rosbag_base_name=_2023_05_13_19_03_07
#rosbag_base_name=_2023_05_13_21_34_26
#rosbag_base_name=_2023_05_13_21_51_39
rosbag_base_name=_2023_05_16_15_02_33

make && python3 src/evaluation/waypoint_timestamp_extractor.py \
    --waypoint_topic_trigger ${waypoint_topic_trigger} \
    --camera_topics_file ${camera_topics_file} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --rosbag_base_name ${rosbag_base_name} \
    --waypoint_file_version ${waypoint_file_version} \
    --force_waypoint_file_regeneration

