#!/bin/bash

root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/
rosbag_file_directory=${root_data_dir}original_data/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
coda_parser_repo_root_dir=/home/amanda/workspaces/amrl_libs/coda
trajectory_sequence_file_directory=/home/amanda/workspaces/ut_vslam/sequences/

#rosbag_base_name="1669743059"
#rosbag_base_name="1668019589"
sequence_file_base_name="2023-05-12_morning"

ouster_branch=writeToFile

# NOTE: Before running this
# Activate conda environment (conda activate coda)
# Source catkin setup for legoloam, but use the version that doesn't overwrite previous config: source ~/catkin_ws/devel/setup.sh --extend
# Make sure that the appropriate version of lego loam is built

cwd=$(pwd)
cd /home/amanda/catkin_ws/src/LeGO-LOAM-1
git checkout ${ouster_branch}
cd ../../
catkin_make -j1
source devel/setup.sh --extend
cd ${cwd}

python3 src/evaluation/run_lego_loam.py \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --coda_parser_repo_root_dir ${coda_parser_repo_root_dir} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --rosbag_file_directory ${rosbag_file_directory} \
     --force_run_lego_loam

