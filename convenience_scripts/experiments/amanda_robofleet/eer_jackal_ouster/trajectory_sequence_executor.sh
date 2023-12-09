#!/bin/bash

rosparam set /use_sim_time false

YOLO_DIR=/root/LTOV-SLAM-Evaluation/yolov5-ros/
SLAM_DIR=/root/LTOV-SLAM-Evaluation/ut_vslam/
yolo_weight=/root/LTOV-SLAM-Evaluation/data/yolo_models/outdoors-final-yolov5s-1.pt

root_data_dir=/root/LTOV-SLAM-Evaluation/data/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=${SLAM_DIR}config/
orb_slam_out_directory=${root_data_dir}orb_out/
rosbag_file_directory=${root_data_dir}original_data/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
trajectory_sequence_file_directory=${SLAM_DIR}sequences/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
bounding_box_post_process_base_directory=${root_data_dir}bounding_box_data/

#cd $YOLO_DIR
#(python3 detect_ros.py --weights $yolo_weight --img 960 --conf 0.01 --device 5) &
#yolo_pid=$!
#sleep 5

odometry_topic="/jackal_velocity_controller/odom"

#sequence_file_base_name="end_of_may_demo_v1"
#sequence_file_base_name="amazon_0523_v0"
sequence_file_base_name="evaluation_2023_07_v1"
#config_file_base_name="tentative_11"
#config_file_base_name="tentative_11_wider_trees_lamps"
#config_file_base_name="no_vis_feats_base7a_2_fallback_c_2"

cd $SLAM_DIR

#config_file_strings=("base7a_1_fallback_a_2_fix_bb_vis_1" "base7a_1_fallback_a_2_fix_bb_vis_2" "base7a_1_fallback_a_2_fix_bb_vis_3" "base7a_1_fallback_a_2_fix_bb_vis_4")
config_file_strings=("base7a_1_fallback_a_2_short_phase1")

for config_file_base_name in ${config_file_strings[@]}; do
  echo ${config_file_base_name}
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
      --odometry_topic ${odometry_topic} \
      --bounding_box_post_process_base_directory ${bounding_box_post_process_base_directory} \
      --output_jacobian_debug --output_bb_assoc --record_viz_rosbag --log_to_file \
      --output_checkpoints --disable_log_to_stderr \
      --record_viz_rosbag \
  #    --output_ellipsoid_debug
done

#kill -9 $yolo_pid
