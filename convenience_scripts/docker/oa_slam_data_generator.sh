#!/bin/bash

WORKDIR=/root/ObVi-SLAM-Evaluation/

YOLO_DIR=${WORKDIR}yolov5/
SLAM_DIR=${WORKDIR}ObVi-SLAM/
yolo_weight=${WORKDIR}data/yolo_models/outdoors-final-yolov5s-1.pt

cd $YOLO_DIR
(python3 detect_ros.py --weights $yolo_weight --img 960 --conf 0.2) &
yolo_pid=$!
sleep 5

cd $SLAM_DIR

root_data_dir=${WORKDIR}/data/

declare -a bagnames=("_2023_05_11_18_35_54" \
                     "_2023_05_13_19_03_07" \
                     "_2023_05_13_21_51_39" \
                     "_2023_05_16_15_02_33" \
                     "_2023_06_23_10_22_43" \
                     "_2023_06_22_21_51_46" \
                     "_2023_06_28_11_02_23" \
                     "_2023_06_27_21_52_02" \
                     "_2023_06_21_10_32_23" \
                     "_2023_06_27_21_36_30" \
                     "_2023_05_17_12_13_10" \
                     "_2023_05_12_13_15_27" \
                     "_2023_06_30_11_27_42" \
                     "_2023_05_12_10_50_32" \
                     "_2023_06_22_22_12_29" \
                     "_2023_06_26_11_08_53")

for bagname in ${bagnames[@]}; do

    nodes_by_timestamp_file=${root_data_dir}orb_out/${bagname}/timestamps/node_ids_and_timestamps.txt
    rosbag_file_directory=${root_data_dir}original_data/
    rosbag_file=${rosbag_file_directory}${bagname}.bag
    oa_slam_data_output_directory=${root_data_dir}oa_slam_in/${bagname}/

    [ ! -d $oa_slam_data_output_directory ]         && mkdir -p ${oa_slam_data_output_directory}

    ./bin/oa_slam_data_generator --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
        --rosbag_file ${rosbag_file} --oa_slam_data_output_directory ${oa_slam_data_output_directory}
    sleep 3

    python src/evaluation/oa_slam/oa_slam_input_formatter.py --data_path ${oa_slam_data_output_directory}
    sleep 3

done

kill -9 $yolo_pid