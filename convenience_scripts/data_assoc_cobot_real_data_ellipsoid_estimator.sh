#!/bin/bash

#rosbag_base_name=cobot_orbit_blue_chair_2022-03-16-15-40-13
#rosbag_base_name=cobot_orbit_red_chair_slower_2022-03-16-15-26-05
#rosbag_base_name=2022-06-15-16-00-33
#rosbag_base_name=2022-06-15-16-32-59
rosbag_base_name=2022-06-15-16-36-13

intrinsics_file_base_name=cobot_kinect_intrinsics
extrinsics_file_base_name=cobot_kinect_extrinsics_2022_03_16

bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/orbit_two_chairs/
bag_file_orig_dir=${bag_file_base_dir}original/
bag_file_bb_dir=${bag_file_base_dir}bounding_box_data/
bag_file_loc_dir=${bag_file_base_dir}localization/
robot_config_dir=/home/amanda/robot_config_info/

bag_suffix=.bag
txt_suffix=.txt
#bounding_box_file_prefix=bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=localization_by_node_id_
#timestamps_by_node_prefix=timestamp_by_node_id_

#bounding_box_file_prefix=min_dedupe_bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=min_dedupe_localization_by_node_id_
#timestamps_by_node_prefix=min_dedupe_timestamp_by_node_id_


bounding_box_file_prefix=spread_out_obs_bounding_box_with_node_id_unfiltered_yolo_v3_
poses_by_node_prefix=spread_out_obs_localization_by_node_id_
timestamps_by_node_prefix=spread_out_obs_timestamp_by_node_id_

#bounding_box_file_prefix=empty_bbs_min_dedupe_bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=empty_bbs_min_dedupe_localization_by_node_id_
#timestamps_by_node_prefix=empty_bbs_min_dedupe_timestamp_by_node_id_

rosbag_prefix=images_

#rosbag_file=${bag_file_orig_dir}${rosbag_base_name}${bag_suffix}
rosbag_file=${bag_file_bb_dir}${rosbag_prefix}${rosbag_base_name}${bag_suffix}
bounding_boxes_by_node_id_file=${bag_file_bb_dir}${bounding_box_file_prefix}${rosbag_base_name}${txt_suffix}
poses_by_node_id_file=${bag_file_loc_dir}${poses_by_node_prefix}${rosbag_base_name}${txt_suffix}
nodes_by_timestamp_file=${bag_file_loc_dir}${timestamps_by_node_prefix}${rosbag_base_name}${txt_suffix}

intrinsics_file=${robot_config_dir}${intrinsics_file_base_name}${txt_suffix}
extrinsics_file=${robot_config_dir}${extrinsics_file_base_name}${txt_suffix}

#DEFINE_string(param_prefix, "", "param_prefix");
#DEFINE_string(rosbag_file,
#              "",
#              "ROS bag file name that contains the images for this run");
#DEFINE_string(bounding_boxes_by_node_id_file,
#              "",
#              "File with bounding box observations by node id");
#DEFINE_string(poses_by_node_id_file,
#              "",
#              "File with initial robot pose estimates");
#DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
#DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
#DEFINE_string(nodes_by_timestamp_file,
#              "",
#              "File containing the timestamp-node mapping");



make && ./bin/offline_object_visual_slam_main --rosbag_file ${rosbag_file} --bounding_boxes_by_node_id_file \
${bounding_boxes_by_node_id_file} --poses_by_node_id_file ${poses_by_node_id_file} --intrinsics_file ${intrinsics_file} \
 --extrinsics_file ${extrinsics_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file}

