#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
rosbag_file_directory=${root_data_dir}original_data/
camera_topics_file=/home/amanda/workspaces/ut_vslam/config/zed_left_camera_topics.txt
waypoint_topic_trigger=/autonomy_arbiter/joystick_stamp

#rosbag_base_name=1676580043
#rosbag_base_name=1676766718
#rosbag_base_name=1676767081
#rosbag_base_name=1676767437
#rosbag_base_name=1676767833
#rosbag_base_name=1676768224
#rosbag_base_name=1676768545
rosbag_base_name=1677097326
#rosbag_base_name=1677097619

make && python3 src/evaluation/waypoint_timestamp_extractor.py \
    --waypoint_topic_trigger ${waypoint_topic_trigger} \
    --camera_topics_file ${camera_topics_file} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --rosbag_base_name ${rosbag_base_name} \
    --force_waypoint_file_regeneration

