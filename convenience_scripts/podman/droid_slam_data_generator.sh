#!/bin/bash

WORKDIR=/root/LTOV-SLAM-Evaluation/
SLAM_UTILS_DIR=${WORKDIR}SLAMUtilsScripts/

cd $SLAM_UTILS_DIR

root_data_dir=${WORKDIR}/data/

declare -a bagnames=("_2023_05_11_18_35_54" \
                     "_2023_05_13_19_03_07" \
                     "_2023_05_13_21_51_39" \
                     "_2023_05_16_15_02_33" \
                     "_2023_06_23_10_22_43" \
                     "_2023_06_22_21_51_46" \
                     "_2023_06_28_11_02_23" \
                     "_2023_06_27_21_52_02" \
                     "_2023_06_21_10_32_23" \
                     "_2023_06_27_21_36_30" \
                     "_2023_05_17_12_13_10" \
                     "_2023_05_12_13_15_27" \
                     "_2023_06_30_11_27_42" \
                     "_2023_05_12_10_50_32" \
                     "_2023_06_22_22_12_29" \
                     "_2023_06_26_11_08_53")

for bagname in ${bagnames[@]}; do
    bagfile=${root_data_dir}original_data/${bagname}.bag
    droid_slam_data_output_directory=${root_data_dir}droid_slam_in/
    [ ! -d $droid_slam_data_output_directory ]         && mkdir -p ${droid_slam_data_output_directory}
    python3 data_processing/parse_bag_to_kitti.py --bagfile $bagfile --outputdir $droid_slam_data_output_directory
done