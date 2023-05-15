#!/bin/bash


run_optimization() {
  bag_time_str=$1

#  bb_type_variant=merged
#  bb_type_variant=yolov3
  bb_type_variant=yolov5

  intrinsics_file_base_name=cobot_kinect_intrinsics
  extrinsics_file_base_name=cobot_kinect_extrinsics_2022_03_16

  bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_09_30/
#  bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_08_25/
  bag_file_orig_dir=${bag_file_base_dir}original/
  bag_file_bb_dir=${bag_file_base_dir}yolo_output/${bb_type_variant}/
  bag_file_loc_dir=${bag_file_base_dir}localization/${bb_type_variant}/
  bag_file_ltm_dir=${bag_file_base_dir}ltms/${bb_type_variant}/

  robot_config_dir=/home/amanda/robot_config_info/

  bag_suffix=.bag
  txt_suffix=.txt
  json_suffix=.json

  spacing_suffix=spaced_out_0_1_
#  spacing_suffix=""

  bounding_box_file_prefix=bb_by_node_${spacing_suffix}
  poses_by_node_prefix=localization_est_by_node_id_${spacing_suffix}
  timestamps_by_node_prefix=timestamps_by_node_id_${spacing_suffix}
  ltm_prefix=long_term_map_${spacing_suffix}

  #rosbag_prefix=images_
  rosbag_prefix=""

  rosbag_file=${bag_file_orig_dir}${rosbag_prefix}${bag_time_str}${bag_suffix}
  bounding_boxes_by_node_id_file=${bag_file_bb_dir}${bounding_box_file_prefix}${bag_time_str}${txt_suffix}
  poses_by_node_id_file=${bag_file_loc_dir}${poses_by_node_prefix}${bag_time_str}${txt_suffix}
  nodes_by_timestamp_file=${bag_file_loc_dir}${timestamps_by_node_prefix}${bag_time_str}${txt_suffix}
  long_term_map_out_file=${bag_file_ltm_dir}${ltm_prefix}${bag_time_str}${json_suffix}

  intrinsics_file=${robot_config_dir}${intrinsics_file_base_name}${txt_suffix}
  extrinsics_file=${robot_config_dir}${extrinsics_file_base_name}${txt_suffix}

  if [ $# -gt 1 ];
  then
    ltm_bag_base_name=$2
    long_term_map_input_file=${bag_file_ltm_dir}${ltm_prefix}${ltm_bag_base_name}${json_suffix}
    echo "Running with LTM ${ltm_bag_base_name}"
    make && ./bin/offline_object_visual_slam_main --rosbag_file ${rosbag_file} --bounding_boxes_by_node_id_file \
    ${bounding_boxes_by_node_id_file} --poses_by_node_id_file ${poses_by_node_id_file} --intrinsics_file ${intrinsics_file} \
    --extrinsics_file ${extrinsics_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
    --long_term_map_output ${long_term_map_out_file} \
    --long_term_map_input ${long_term_map_input_file}
  else
    echo "Running without LTM"
    echo "rosbag_file ${rosbag_file}"
    echo "bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file}"
    echo "poses_by_node_id_file ${poses_by_node_id_file}"
    echo "intrinsics_file ${intrinsics_file}"
    echo "extrinsics_file ${extrinsics_file}"
    echo "nodes_by_timestamp_file ${nodes_by_timestamp_file}"
    echo "long_term_map_output ${long_term_map_out_file}"

    make && ./bin/offline_object_visual_slam_main --rosbag_file ${rosbag_file} --bounding_boxes_by_node_id_file \
    ${bounding_boxes_by_node_id_file} --poses_by_node_id_file ${poses_by_node_id_file} --intrinsics_file ${intrinsics_file} \
    --extrinsics_file ${extrinsics_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
    --long_term_map_output ${long_term_map_out_file}
  fi
}

cur_bag_time_str=$1
echo "Running optimization for ${cur_bag_time_str}, no LTM"
run_optimization ${cur_bag_time_str}
prev_bag_time_str=${cur_bag_time_str}

if [ $# -gt 1 ]; then
  for ((i=2; i <= $#; i=$i+1)); do
    cur_bag_time_str=${!i}
    echo "Running optimization for bag ${cur_bag_time_str} with LTM ${prev_bag_time_str}"
    run_optimization ${cur_bag_time_str} ${prev_bag_time_str}
    prev_bag_time_str=${cur_bag_time_str}
  done
fi
