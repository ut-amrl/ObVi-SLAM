#!/bin/bash
rosparam set /use_sim_time false

YOLO_DIR=/root/LTOV-SLAM-Evaluation/yolov5-ros/
SLAM_DIR=/root/LTOV-SLAM-Evaluation/ut_vslam/
yolo_weight=/root/LTOV-SLAM-Evaluation/data/yolo_models/outdoors-final-yolov5s-1.pt

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
# TODO change me
config_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/config/amazon_0523/
trajectory_sequence_file_directory=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/amazon_0523/

calibration_file_directory=${root_data_dir}calibration/
rosbag_file_directory=${root_data_dir}original_data/
orb_slam_out_directory=${root_data_dir}orb_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

sequence_file_base_name="amazon_0523_v0"

# cd $YOLO_DIR
# (python3 detect_ros.py --weights $yolo_weight --img 960 --conf 0.01) &
# yolo_pid=$!
# sleep 5

cd $SLAM_DIR

config_file_base_name="amazon_0523_base"
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
    --record_viz_rosbag --log_to_file

kill -9 $yolo_pid
