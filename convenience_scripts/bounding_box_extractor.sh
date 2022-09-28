#!/bin/bash



#rosbag_base_name=2022-08-24-16-26-29
#rosbag_base_name=2022-08-24-16-28-27
#rosbag_base_name=2022-08-24-17-04-19
#rosbag_base_name=2022-08-24-17-06-14
#rosbag_base_name=2022-08-24-17-08-00
#rosbag_base_name=2022-08-24-17-08-56
#rosbag_base_name=2022-08-24-17-10-20
rosbag_base_name=2022-08-24-17-11-48


#bb_type_variant=merged
#bb_in_file_prefix=bb_merged_
#bb_type_variant=yolov3
#bb_in_file_prefix=bb_chairs_
bb_type_variant=yolov5
bb_in_file_prefix=bb_cones_

bag_file_base_dir=/home/amanda/rosbags/ellipsoid_slam/ltm_test_2022_08_25/
bag_file_orig_dir=${bag_file_base_dir}original/
bag_file_bb_dir=${bag_file_base_dir}yolo_output/${bb_type_variant}/
bag_file_loc_dir=${bag_file_base_dir}localization/${bb_type_variant}/

bag_suffix=.bag
txt_suffix=.txt

bounding_box_file_prefix=bb_by_node_
poses_by_node_prefix=localization_est_by_node_id_
timestamps_by_node_prefix=timestamps_by_node_id_

rosbag_file=${bag_file_orig_dir}${rosbag_base_name}${bag_suffix}
bb_by_timestamp_file=${bag_file_bb_dir}${bb_in_file_prefix}${rosbag_base_name}${txt_suffix}
bounding_boxes_by_node_id_file=${bag_file_bb_dir}${bounding_box_file_prefix}${rosbag_base_name}${txt_suffix} # TODO fix
poses_by_node_id_file=${bag_file_loc_dir}${poses_by_node_prefix}${rosbag_base_name}${txt_suffix} # TODO fix
nodes_by_timestamp_file=${bag_file_loc_dir}${timestamps_by_node_prefix}${rosbag_base_name}${txt_suffix} # TODO fix

make && ./bin/localization_rosbag_extraction_and_interpolation_for_bounding_boxes --rosbag_file_name ${rosbag_file} \
--bb_by_timestamp_file_no_association ${bb_by_timestamp_file} --bb_by_node_out_file \
${bounding_boxes_by_node_id_file} --localization_est_out_file ${poses_by_node_id_file} --timestamp_by_node_id_file ${nodes_by_timestamp_file}

