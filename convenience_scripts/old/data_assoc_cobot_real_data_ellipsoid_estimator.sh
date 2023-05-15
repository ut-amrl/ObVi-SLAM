#!/bin/bash

#rosbag_base_name=cobot_orbit_blue_chair_2022-03-16-15-40-13
#rosbag_base_name=cobot_orbit_red_chair_slower_2022-03-16-15-26-05
#rosbag_base_name=2022-06-15-16-00-33
#rosbag_base_name=2022-06-15-16-32-59
#rosbag_base_name=2022-06-15-16-36-13

rosbag_base_name=2022-08-24-16-26-29
#rosbag_base_name=2022-08-24-16-28-27
#rosbag_base_name=2022-08-24-17-04-19
#rosbag_base_name=2022-08-24-17-06-14
#rosbag_base_name=2022-08-24-17-08-00
#rosbag_base_name=2022-08-24-17-08-56
#rosbag_base_name=2022-08-24-17-10-20
#rosbag_base_name=2022-08-24-17-11-48

#ltm_bag_base_name=2022-08-24-17-06-14
#ltm_bag_base_name=2022-08-24-17-08-56
ltm_bag_base_name=2022-08-24-17-11-48

bb_type_variant=merged
#bb_type_variant=yolov3
#bb_type_variant=yolov5

intrinsics_file_base_name=cobot_kinect_intrinsics
extrinsics_file_base_name=cobot_kinect_extrinsics_2022_03_16

#bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/orbit_two_chairs/
#bag_file_orig_dir=${bag_file_base_dir}original/
#bag_file_bb_dir=${bag_file_base_dir}bounding_box_data/
#bag_file_loc_dir=${bag_file_base_dir}localization/

bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_08_25/
bag_file_orig_dir=${bag_file_base_dir}original/
bag_file_bb_dir=${bag_file_base_dir}yolo_output/${bb_type_variant}/
bag_file_loc_dir=${bag_file_base_dir}/localization/${bb_type_variant}/
bag_file_ltm_dir=${bag_file_base_dir}/ltms/${bb_type_variant}/

robot_config_dir=/home/amanda/robot_config_info/

bag_suffix=.bag
txt_suffix=.txt
json_suffix=.json
#bounding_box_file_prefix=bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=localization_by_node_id_
#timestamps_by_node_prefix=timestamp_by_node_id_

#bounding_box_file_prefix=min_dedupe_bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=min_dedupe_localization_by_node_id_
#timestamps_by_node_prefix=min_dedupe_timestamp_by_node_id_


#bounding_box_file_prefix=spread_out_obs_bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=spread_out_obs_localization_by_node_id_
#timestamps_by_node_prefix=spread_out_obs_timestamp_by_node_id_

bounding_box_file_prefix=bb_by_node_
poses_by_node_prefix=localization_est_by_node_id_
timestamps_by_node_prefix=timestamps_by_node_id_
ltm_prefix=long_term_map_

#bounding_box_file_prefix=empty_bbs_min_dedupe_bounding_box_with_node_id_unfiltered_yolo_v3_
#poses_by_node_prefix=empty_bbs_min_dedupe_localization_by_node_id_
#timestamps_by_node_prefix=empty_bbs_min_dedupe_timestamp_by_node_id_

#rosbag_prefix=images_
rosbag_prefix=""

#rosbag_file=${bag_file_orig_dir}${rosbag_base_name}${bag_suffix}
#rosbag_file=${bag_file_bb_dir}${rosbag_prefix}${rosbag_base_name}${bag_suffix}
rosbag_file=${bag_file_orig_dir}${rosbag_prefix}${rosbag_base_name}${bag_suffix}
bounding_boxes_by_node_id_file=${bag_file_bb_dir}${bounding_box_file_prefix}${rosbag_base_name}${txt_suffix}
poses_by_node_id_file=${bag_file_loc_dir}${poses_by_node_prefix}${rosbag_base_name}${txt_suffix}
nodes_by_timestamp_file=${bag_file_loc_dir}${timestamps_by_node_prefix}${rosbag_base_name}${txt_suffix}
long_term_map_out_file=${bag_file_ltm_dir}${ltm_prefix}${rosbag_base_name}${json_suffix}
long_term_map_input_file=${bag_file_ltm_dir}${ltm_prefix}${ltm_bag_base_name}${json_suffix}

intrinsics_file=${robot_config_dir}${intrinsics_file_base_name}${txt_suffix}
extrinsics_file=${robot_config_dir}${extrinsics_file_base_name}${txt_suffix}

make && ./bin/offline_object_visual_slam_main --rosbag_file ${rosbag_file} --bounding_boxes_by_node_id_file \
${bounding_boxes_by_node_id_file} --poses_by_node_id_file ${poses_by_node_id_file} --intrinsics_file ${intrinsics_file} \
 --extrinsics_file ${extrinsics_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
 --long_term_map_output ${long_term_map_out_file} \
# --long_term_map_input ${long_term_map_input_file}
