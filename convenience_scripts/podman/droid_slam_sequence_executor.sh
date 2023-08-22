#!/bin/bash

WORKDIR=/root/LTOV-SLAM-Evaluation/

droid_slam_dir=${WORKDIR}DROID-SLAM/
root_data_dir=${WORKDIR}/data/

sequence_file_base_name="evaluation_2023_07_v1"
sequence_file=/root/LTOV-SLAM-Evaluation/ut_vslam/sequences/${sequence_file_base_name}.json

cd $droid_slam_dir

python evaluation_scripts/test_obvi_seq.py --seqfile $sequence_file_base_name --datadir $root_data_dir --calib calib/obvi_slam.txt --stride 3