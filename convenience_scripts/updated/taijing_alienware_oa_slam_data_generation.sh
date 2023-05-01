#!/bin/bash
bagname=1676766718

root_data_dir=/home/tiejean/Documents/mnt/oslam/
nodes_by_timestamp_file=${root_data_dir}orb_out/${bagname}/timestamps/node_ids_and_timestamps.txt
rosbag_file_directory=${root_data_dir}original_data/
rosbag_file=${rosbag_file_directory}${bagname}.bag
oa_slam_data_output_directory=${root_data_dir}oa_slam_in/${bagname}

[ ! -d $oa_slam_data_output_directory ]         && mkdir -p ${oa_slam_data_output_directory}

./bin/oa_slam_data_generator --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
    --rosbag_file ${rosbag_file} --oa_slam_data_output_directory ${oa_slam_data_output_directory}