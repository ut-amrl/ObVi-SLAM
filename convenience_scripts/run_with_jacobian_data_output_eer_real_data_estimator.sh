#!/bin/bash

#rosbag_base_name=cobot_orbit_blue_chair_2022-03-16-15-40-13
#rosbag_base_name="1669743059"
#rosbag_base_name="1668019589"
#rosbag_base_name=1669743339
#rosbag_base_name=1669758673
#rosbag_base_name=1669938112
#rosbag_base_name=1669743629
#rosbag_base_name=1669743856
#rosbag_base_name=1669938357
#rosbag_base_name=1669938651
#rosbag_base_name=1669938899
rosbag_base_name=1669939150

#version=""
version="_vOnly"

ltm_opt_jacobian_info_directory=/home/amanda/rosbags/ellipsoid_slam/eer_bags/jacobian_debugging/${rosbag_base_name}/full_frames_visual_only_jacobian_info_ellipsoids${version}/
mkdir ${ltm_opt_jacobian_info_directory}

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
calib_base_dir=${root_data_dir}zed_calib/
obj_det_base_dir=${root_data_dir}obj_det/
#ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in/

ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in_sparse/
ut_vslam_out_base_dir=${root_data_dir}ut_vslam_out/
orig_data_dir=${root_data_dir}/original_data/

bag_suffix=".bag"

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

long_term_map_output_dir=${ut_vslam_out_base_dir}${rosbag_base_name}/
long_term_map_output_file=${long_term_map_output_dir}long_term_map.json
visual_feature_results_file=${long_term_map_output_dir}visual_feature_results.json

make && ./bin/offline_object_visual_slam_main --intrinsics_file ${intrinsics_file} \
--extrinsics_file ${extrinsics_file} --bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file} \
--poses_by_node_id_file ${poses_by_node_id_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
--rosbag_file ${rosbag_file} --long_term_map_output ${long_term_map_output_file} \
--low_level_feats_dir ${low_level_feats_dir} \
--ltm_opt_jacobian_info_directory ${ltm_opt_jacobian_info_directory} \
--visual_feature_results_file ${visual_feature_results_file}

