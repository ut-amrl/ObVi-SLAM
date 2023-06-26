#!/bin/bash

WORKDIR=/root/LTOV-SLAM-Evaluation/

YOLO_DIR=${WORKDIR}yolov5-ros/
SLAM_DIR=${WORKDIR}ut_vslam/
yolo_weight=${WORKDIR}data/yolo_models/outdoors-final-yolov5s-1.pt

# cd $YOLO_DIR
# (python3 detect_ros.py --weights $yolo_weight --img 960 --conf 0.2) &
# yolo_pid=$!
# sleep 5

cd $SLAM_DIR

root_data_dir=${WORKDIR}/data/

declare -a bagnames=("_2023_05_11_18_35_54")
# declare -a bagnames=("_2023_05_11_18_35_54" \
#                      "_2023_05_12_11_00_35" \
#                      "_2023_05_13_19_03_07" \
#                      "_2023_05_13_21_34_26" \
#                      "_2023_05_13_21_51_39" \
#                      "_2023_05_16_15_02_33")

for bagname in ${bagnames[@]}; do

    nodes_by_timestamp_file=${root_data_dir}orb_out/${bagname}/timestamps/node_ids_and_timestamps.txt
    rosbag_file_directory=${root_data_dir}original_data/
    rosbag_file=${rosbag_file_directory}${bagname}.bag
    cube_slam_data_output_directory=${root_data_dir}cube_slam_in/${bagname}/

    [ ! -d $cube_slam_data_output_directory ]         && mkdir -p ${cube_slam_data_output_directory}

    ./bin/cube_slam_data_generator --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
        --rosbag_file ${rosbag_file} --cube_slam_data_output_directory ${cube_slam_data_output_directory}
    sleep 3

done

kill -9 $yolo_pid