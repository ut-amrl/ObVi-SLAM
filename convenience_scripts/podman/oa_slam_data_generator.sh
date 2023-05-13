#!/bin/bash
bagname=1676766718

YOLO_DIR=/root/LTOV-SLAM-Evaluation/yolov5-ros/
SLAM_DIR=/root/LTOV-SLAM-Evaluation/ut_vslam/
yolo_weight=/root/LTOV-SLAM-Evaluation/data/yolo_models/outdoors-final-yolov5s-1.pt

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
nodes_by_timestamp_file=${root_data_dir}orb_out/${bagname}/timestamps/node_ids_and_timestamps.txt
rosbag_file_directory=${root_data_dir}original_data/
rosbag_file=${rosbag_file_directory}${bagname}.bag
oa_slam_data_output_directory=${root_data_dir}oa_slam_in/${bagname}/

[ ! -d $oa_slam_data_output_directory ]         && mkdir -p ${oa_slam_data_output_directory}

cd $YOLO_DIR
(python3 detect_ros.py --weights $yolo_weight --img 960 --conf 0.2) &
yolo_pid=$!
sleep 5

cd $SLAM_DIR

./bin/oa_slam_data_generator --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
    --rosbag_file ${rosbag_file} --oa_slam_data_output_directory ${oa_slam_data_output_directory}

python src/evaluation/oa_slam/oa_slam_input_formatter.py --data_path ${oa_slam_data_output_directory}