#!/bin/bash
WORKDIR=/root/LTOV-SLAM-Evaluation/

root_data_dir=${WORKDIR}data/
rosbag_file_directory=${root_data_dir}original_data/

orb_slam_configuration_file=${WORKDIR}ORB_SLAM2/Examples/Stereo/high_res_jackal_zed_rectified.yaml
orb_slam_vocabulary_file=${WORKDIR}ORB_SLAM2/Vocabulary/ORBvoc.txt
orb_slam_2_out_root_dir=${root_data_dir}orb_out/

trajectory_sequence_file_directory=${WORKDIR}ut_vslam/sequences/single_bag/

declare -a sequence_file_base_names=("20230511183554_only" \
                                     "20230512110035_only" \
                                     "20230513190307_only" \
                                     "20230513213426_only" \
                                     "20230513215139_only" \
                                     "20230516150233_only")

for sequence_file_base_name in ${sequence_file_base_names[@]}; do
  python3 src/evaluation/run_orb_slam_2.py \
      --orb_slam_configuration_file=${orb_slam_configuration_file} \
      --orb_slam_vocabulary_file=${orb_slam_vocabulary_file} \
      --orb_slam_2_out_root_dir ${orb_slam_2_out_root_dir} \
      --rosbag_file_directory ${rosbag_file_directory} \
      --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
      --sequence_file_base_name ${sequence_file_base_name} \
      --force_run_orb_slam
  sleep 10
done
