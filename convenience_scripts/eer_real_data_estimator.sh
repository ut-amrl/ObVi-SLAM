#!/bin/bash

#rosbag_base_name=cobot_orbit_blue_chair_2022-03-16-15-40-13
rosbag_base_name="1669743059"
#rosbag_base_name="1668019589"
#rosbag_base_name="1669743339"

ellipsoid_debug_version="v1"

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
calib_base_dir=${root_data_dir}zed_calib/
obj_det_base_dir=${root_data_dir}obj_det/
#ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in/

ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in_sparse/
ut_vslam_out_base_dir=${root_data_dir}ut_vslam_out/
orig_data_dir=${root_data_dir}/original_data/

debug_base_dir=${root_data_dir}debugging/${rosbag_base_name}/
mkdir -p ${debug_base_dir}
ellipsoid_debug_dir=${debug_base_dir}ellipsoids/${ellipsoid_debug_version}/
mkdir -p ${ellipsoid_debug_dir}
rm ${ellipsoid_debug_dir}*.png

bag_suffix=".bag"
json_suffix=".json"

intrinsics_file_base_name=camera_matrix.txt
extrinsics_file_base_name=extrinsics.txt
intrinsics_file=${calib_base_dir}${intrinsics_file_base_name}
extrinsics_file=${calib_base_dir}${extrinsics_file_base_name}

bounding_box_dir=${obj_det_base_dir}${rosbag_base_name}/
bounding_boxes_by_node_id_file=${bounding_box_dir}bounding_boxes_by_node.csv

ut_vslam_in_dir=${ut_vslam_in_base_dir}${rosbag_base_name}/
poses_by_node_id_file=${ut_vslam_in_dir}poses/initial_robot_poses_by_node.txt
nodes_by_timestamp_file=${ut_vslam_in_dir}timestamps/node_ids_and_timestamps.txt
rosbag_file=${orig_data_dir}${rosbag_base_name}${bag_suffix}
low_level_feats_dir=${ut_vslam_in_dir}
bb_associations_base_file="data_association_results_"

long_term_map_output_dir=${ut_vslam_out_base_dir}${rosbag_base_name}/
long_term_map_output_file=${long_term_map_output_dir}long_term_map.json
bb_associations_file=${long_term_map_output_dir}${bb_associations_base_file}${rosbag_base_name}${json_suffix}

make && ./bin/offline_object_visual_slam_main --intrinsics_file ${intrinsics_file} \
--extrinsics_file ${extrinsics_file} --bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file} \
--poses_by_node_id_file ${poses_by_node_id_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
--rosbag_file ${rosbag_file} --long_term_map_output ${long_term_map_output_file} \
--low_level_feats_dir ${low_level_feats_dir} \
--debug_images_output_directory ${ellipsoid_debug_dir}

