#!/bin/bash
# bagname=_2023_05_11_18_35_54

# root_data_dir=/root/LTOV-SLAM-Evaluation/data/
# oa_slam_in_dir=${root_data_dir}oa_slam_in/${bagname}/
# path_to_image_sequence_for_cam_1=${oa_slam_in_dir}1/cam_1_images.txt
# path_to_image_sequence_for_cam_2=${oa_slam_in_dir}2/cam_2_images.txt
# detections_file_for_cam_1=${oa_slam_in_dir}1/detections/detections.json
# detections_file_for_cam_2=${oa_slam_in_dir}2/detections/detections.json

# oa_slam_dir=/root/LTOV-SLAM-Evaluation/OA-SLAM/
# camera_file=${oa_slam_dir}Cameras/husky_zed_rectified.yaml
# nodes_by_timestamp_file=${root_data_dir}orb_out/${bagname}/timestamps/node_ids_and_timestamps.txt

# oa_slam_out_dir=${root_data_dir}oa_slam_out/
# [ ! -d $oa_slam_out_dir ]         && mkdir -p ${oa_slam_out_dir}
# oa_slam_pose_path=${oa_slam_out_dir}${bagname}/
# [ ! -d $oa_slam_pose_path ]         && mkdir -p ${oa_slam_pose_path}
# oa_slam_pose_path=${oa_slam_pose_path}oa_slam_poses.csv


# cd ${oa_slam_dir}
# sh build.sh

# ./bin/oa-slam_stereo \
#     Vocabulary/ORBvoc.txt \
#     ${camera_file} \
#     ${oa_slam_in_dir} \
#     ${path_to_image_sequence_for_cam_1} ${path_to_image_sequence_for_cam_2} \
#     ${nodes_by_timestamp_file} \
#     ${detections_file_for_cam_1} ${detections_file_for_cam_2} \
#     null points+objects \
#     ${oa_slam_pose_path}

# TODO (Taijing) add a script that check if data is generated;
# If not, run the data generation script

WORKDIR=/root/LTOV-SLAM-Evaluation/

oa_slam_dir=${WORKDIR}OA-SLAM/
camera_file=${oa_slam_dir}/Cameras/jackal_zed_rectified.yaml
root_data_dir=${WORKDIR}data/

sequence_file_base_name="end_of_may_demo_v1"
# TODO fix me
sequence_file=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/${sequence_file_base_name}.json

cd ${oa_slam_dir}

./bin/oa-slam_stereo_multi_sessions \
    Vocabulary/ORBvoc.txt \
    ${camera_file} \
    ${root_data_dir} \
    ${sequence_file} \
    null \
    points+objects