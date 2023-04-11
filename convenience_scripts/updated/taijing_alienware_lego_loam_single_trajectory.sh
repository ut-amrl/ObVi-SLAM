#!/bin/bash

# trap "trap -SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

root_data_dir=/home/tiejean/Documents/mnt/vslam/
rosbag_file_directory=${root_data_dir}
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
coda_parser_repo_root_dir=/home/tiejean/Documents/projects/coda/

rosbag_base_name=1677097326

python3 src/evaluation/run_lego_loam.py \
    --lego_loam_out_root_dir ${lego_loam_out_root_dir} \
    --coda_parser_repo_root_dir ${coda_parser_repo_root_dir} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --rosbag_base_name ${rosbag_base_name} --force_run_lego_loam

# trap - SIGINT SIGTERM EXIT
