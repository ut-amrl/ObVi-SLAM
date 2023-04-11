#!/bin/bash

YOLO_DIR="/home/tiejean/projects/yolov5-ros/"
SLAM_DIR="/home/tiejean/projects/ut_semantic_vslam/"
yolo_weight="/robodata/taijing/object-slam/yolov5-models/outdoors-final-yolov5s-0.pt"

root_data_dir=/robodata/taijing/object-slam/oslam/
calibration_file_directory=/robodata/taijing/object-slam/calibration/husky_zed_resolution_1_scale_0_5/
config_file_directory=/home/tiejean/projects/ut_semantic_vslam/config/

rosbag_file_directory=${root_data_dir}original_data/
orb_slam_out_directory=${root_data_dir}orb_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=/home/tiejean/projects/ut_semantic_vslam/sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

sequence_file_base_name="high_res_20230218_1a_7326"
config_file_base_name="7"

cd $YOLO_DIR
(python3 detect_ros.py --weights $yolo_weight --img 640 --conf 0.01) &
yolo_pid=$!
sleep 5

cd $SLAM_DIR

python3 src/evaluation/ltm_trajectory_sequence_executor.py \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --output_bb_assoc --record_viz_rosbag 
    --force_run_orb_post_process

kill -9 $yolo_pid