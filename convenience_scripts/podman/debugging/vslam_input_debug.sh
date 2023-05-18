#!/bin/bash

WORKDIR=/root/LTOV-SLAM-Evaluation/
SLAM_DIR=${WORKDIR}ut_vslam/

cd $SLAM_DIR
make -j8

root_data_dir=${WORKDIR}data/
calibration_file_directory=${root_data_dir}calibration/
config_file_directory=${WORKDIR}ut_vslam/config/
rosbag_file_directory=${root_data_dir}original_data/

config_version="dqa"
params_config_file=${config_file_directory}${config_version}.json


declare -a bagnames=("_2023-05-16-15-06-47" \
                     "_2023-05-16-15-19-50" \
                     "_2023-05-16-15-24-19")

for bagname in ${bagnames[@]}; do

    orb_out_dir=${root_data_dir}orb_out/${bagname}/
    orb_post_process_dir=${root_data_dir}orb_post_process/
    unsparsified_vslam_in_dir=${orb_post_process_dir}unsparsified_ut_vslam_in/${bagname}/
    [ ! -d $unsparsified_vslam_in_dir ]  && mkdir -p $unsparsified_vslam_in_dir

    # python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i $orb_out_dir -o $unsparsified_vslam_in_dir 
    # sleep 3
    ./bin/initialize_traj_and_feats_from_orb_out --raw_data_path $orb_out_dir --calibration_path $calibration_file_directory --processed_data_path $unsparsified_vslam_in_dir
    sleep 3

    sparsified_vslam_in_dir=${orb_post_process_dir}sparsified_ut_vslam_in/${config_version}/${bagname}/
    [ ! -d  sparsified_vslam_in_dir]  && mkdir -p $sparsified_vslam_in_dir

    ./bin/orb_trajectory_sparsifier -input_processed_data_path $unsparsified_vslam_in_dir --output_processed_data_path $sparsified_vslam_in_dir --params_config_file $params_config_file
    sleep 3


    intrinsics_file=${calibration_file_directory}camera_matrix.txt
    extrinsics_file=${calibration_file_directory}extrinsics.txt
    poses_by_node_id_file=${sparsified_vslam_in_dir}poses/initial_robot_poses_by_node.txt
    nodes_by_timestamp_file=${sparsified_vslam_in_dir}timestamps/node_ids_and_timestamps.txt
    rosbag_file=${rosbag_file_directory}${bagname}.bag
    low_level_feats_dir=${sparsified_vslam_in_dir}

    debug_output_directory=${root_data_dir}sparsified_ut_vslam_in_dqa/
    [ ! -d $debug_output_directory ]  && mkdir -p $debug_output_directory
    debug_output_directory=${debug_output_directory}${config_version}/
    [ ! -d $debug_output_directory ]  && mkdir -p $debug_output_directory
    debug_output_directory=${debug_output_directory}${bagname}/
    [ ! -d $debug_output_directory ]  && mkdir -p $debug_output_directory

    ./bin/visualize_vslam_inputs \
        --debug_output_directory ${debug_output_directory} \
        --intrinsics_file ${intrinsics_file} \
        --extrinsics_file ${extrinsics_file} \
        --poses_by_node_id_file ${poses_by_node_id_file} \
        --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
        --rosbag_file ${rosbag_file} \
        --low_level_feats_dir ${low_level_feats_dir} \
        --params_config_file ${params_config_file}
    echo "================="
    
done