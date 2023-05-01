#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
rosbag_file_directory=${root_data_dir}original_data/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

#rosbag_base_name="1669743059"
#rosbag_base_name="1668019589"
sequence_file_base_name="20230218_all"

# NOTE: Before running this
# Source catkin setup for legoloam, but use the version that doesn't overwrite previous config: source ~/catkin_ws/devel/setup.sh --extend
# Make sure that the appropriate version of lego loam is built

python3 src/evaluation/run_lego_loam_velodyne.py \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --rosbag_file_directory ${rosbag_file_directory} \
     --force_run_lego_loam

