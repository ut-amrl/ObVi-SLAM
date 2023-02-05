#!/bin/bash

#rosbag_base_name=cobot_orbit_blue_chair_2022-03-16-15-40-13
#rosbag_base_name="1669743059"
rosbag_base_name="1668019589"
#version=""
#version="_v2"
version="_vOnly"

ltm_opt_jacobian_info_directory=/home/amanda/rosbags/ellipsoid_slam/eer_bags/jacobian_debugging/${rosbag_base_name}/full_frames_visual_only_jacobian_info_ellipsoids${version}/
#problem_feature_file_base_name=ProblemFeatureIndices_LT_1eNeg8.csv
problem_feature_file_base_name=ProblemFeatureIndices_LT_1eNeg2.csv

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
#ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in/

ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in_sparse/
orig_data_dir=${root_data_dir}/original_data/

bag_suffix=".bag"

ut_vslam_out_base_dir=${root_data_dir}ut_vslam_out/
ut_vslam_in_dir=${ut_vslam_in_base_dir}${rosbag_base_name}/
nodes_by_timestamp_file=${ut_vslam_in_dir}timestamps/node_ids_and_timestamps.txt
rosbag_file=${orig_data_dir}${rosbag_base_name}${bag_suffix}
low_level_feats_dir=${ut_vslam_in_dir}

jacobian_residual_info_file=${ltm_opt_jacobian_info_directory}ordered_jacobian_residual_info.json
problem_feats_matlab_file=${ltm_opt_jacobian_info_directory}${problem_feature_file_base_name}
jacobian_debugging_images_out_dir=${ltm_opt_jacobian_info_directory}debug_images/

long_term_map_output_dir=${ut_vslam_out_base_dir}${rosbag_base_name}/
visual_feature_results_file=${ltm_opt_jacobian_info_directory}visual_feature_results.json


make && ./bin/visualize_jacobian_problem_feats \
  --jacobian_residual_info_file ${jacobian_residual_info_file} \
  --problem_feats_matlab_file ${problem_feats_matlab_file} \
  --images_out_dir ${jacobian_debugging_images_out_dir} \
  --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
  --low_level_feats_dir ${low_level_feats_dir} \
  --rosbag_file ${rosbag_file} \
  --visual_feature_results_file ${visual_feature_results_file}


