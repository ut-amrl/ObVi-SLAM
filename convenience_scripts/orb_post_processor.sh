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

original_bag_base_name="1668019589"

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
calib_base_dir=${root_data_dir}zed_calib/
obj_det_base_dir=${root_data_dir}obj_det/
orb_out_base_dir=${root_data_dir}orb_out/
ut_vslam_in_base_dir=${root_data_dir}ut_vslam_in/
ut_vslam_in_sparse_base_dir=${root_data_dir}ut_vslam_in_sparse/
ut_vslam_out_base_dir=${root_data_dir}ut_vslam_out/

obj_det_dir_for_bag=${obj_det_base_dir}${rosbag_base_name}/
orb_out_dir_for_bag=${orb_out_base_dir}${rosbag_base_name}/
ut_vslam_in_dir_for_bag=${ut_vslam_in_base_dir}${rosbag_base_name}/
ut_vslam_in_sparse_dir_for_bag=${ut_vslam_in_sparse_base_dir}${rosbag_base_name}/
ut_vslam_out_dir_for_bag=${ut_vslam_out_base_dir}${rosbag_base_name}/

mkdir ${obj_det_dir_for_bag}
mkdir ${ut_vslam_in_dir_for_bag}
mkdir ${ut_vslam_in_sparse_dir_for_bag}
mkdir ${ut_vslam_out_dir_for_bag}

cp ${obj_det_base_dir}${original_bag_base_name}/bounding_boxes_by_node.csv ${obj_det_dir_for_bag}

make

python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i ${orb_out_dir_for_bag} \
 -o ${ut_vslam_in_dir_for_bag}
./bin/initialize_traj_and_feats_from_orb_out --raw_data_path ${orb_out_dir_for_bag} \
 --calibration_path ${calib_base_dir} --processed_data_path ${ut_vslam_in_dir_for_bag}
./bin/orb_trajectory_sparsifier --input_processed_data_path ${ut_vslam_in_dir_for_bag} \
--output_processed_data_path ${ut_vslam_in_sparse_dir_for_bag}


