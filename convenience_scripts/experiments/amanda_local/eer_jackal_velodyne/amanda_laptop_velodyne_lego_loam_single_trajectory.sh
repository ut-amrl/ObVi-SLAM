#!/bin/bash

#root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_bags/
#root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_higher_res/
root_data_dir=/home/amanda/rosbags/ellipsoid_slam/eer_jackal/

rosbag_file_directory=${root_data_dir}original_data/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/

velodyne_branch=writeResultsWithTimestampsToFileVelodyne

#rosbag_base_name="1669743059"
#rosbag_base_name="_2023-04-22-17-28-08_1a1"
#rosbag_base_name="_2023-04-22-17-32-43_2a1"
#rosbag_base_name="_2023-04-22-17-38-03_3a1"
rosbag_base_name="_2023-04-22-17-44-01_4a1"
#rosbag_base_name="_2023-04-22-17-49-18_5a1"
#rosbag_base_name="_2023-04-22-17-54-14_6a1"
#rosbag_base_name="_2023-04-22-17-58-23_2a2"
#rosbag_base_name="1668019589"

# NOTE: Before running this
# Source catkin setup for legoloam, but use the version that doesn't overwrite previous config: source ~/catkin_ws/devel/setup.sh --extend
# Make sure that the appropriate version of lego loam is built

cwd=$(pwd)
cd /home/amanda/catkin_ws/src/LeGO-LOAM-1
git checkout ${velodyne_branch}
cd ../../
catkin_make -j1
source devel/setup.bash
cd ${cwd}

python3 src/evaluation/run_lego_loam_velodyne.py \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --rosbag_base_name ${rosbag_base_name} --force_run_lego_loam

