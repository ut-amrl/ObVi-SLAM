# bagname=freiburg2_desk
# configname=base7a_2_fallback_a_2_tum

# SLAMDIR=/home/tiejean/Documents/projects/ut_vslam
# DARADIR=/home/tiejean/Documents/mnt/oslam
# BBOX_DIR=$DARADIR/bounding_box_data
# CALIB_DIR=$DARADIR/calibration_tum


# vslam_in=$DARADIR/orb_post_process/sparsified_ut_vslam_in/$configname/$bagname/

# configfile=$SLAMDIR/config/${base7a_2_fallback_a_2}.json
# intrinsics_file=$CALIB_DIR/camera_matrix.txt
# extrinsics_file=$CALIB_DIR/extrinsics.txt
# bounding_boxes_by_timestamp_file=$BBOX_DIR/bounding_boxes_by_timestamp_${bagname}.csv
# poses_by_node_id_file=$ORB_POST_PROCESS
# sparsified_orb_out=$ORB_POST_PROCESS/sparsified_ut_vslam_in/$configname/$bagname/
# poses_by_node_id_file=$vslam_in/poses/initial_robot_poses_by_node.txt
# nodes_by_timestamp_file=$vslam_in/timestamps/node_ids_and_timestamps.txt
# rosbag_file=$DARADIR/TUM/$bagname

# ./bin/offline_object_visual_slam_main \
#     --param_prefix $bagname \
#     --intrinsics_file $intrinsics_file \
#     --extrinsics_file $extrinsics_file \
#     --bounding_boxes_by_timestamp_file $bounding_boxes_by_timestamp_file \
#     --poses_by_node_id_file $poses_by_node_id_file \
#     --nodes_by_timestamp_file $nodes_by_timestamp_file \
#     --rosbag_file $rosbag_file \

rosparam set /use_sim_time false
SLAM_DIR=/home/tiejean/Documents/projects/ut_vslam
config_file_directory=$SLAM_DIR/config/
trajectory_sequence_file_directory=$SLAM_DIR/sequences/

root_data_dir=/home/tiejean/Documents/mnt/oslam/
calibration_file_directory=${root_data_dir}calibration_tum/
rosbag_file_directory=${root_data_dir}TUM/
orb_slam_out_directory=${root_data_dir}orb_out/
orb_post_process_base_directory=${root_data_dir}orb_post_process/
results_root_directory=${root_data_dir}ut_vslam_results/
lego_loam_out_root_dir=${root_data_dir}lego_loam_out/
bounding_box_post_process_base_directory=$root_data_dir/bounding_box_data/

sequence_file_base_name="tum_fr2_desk"

cd $SLAM_DIR

config_file_base_name="tum_fr2_desk"
python3 src/evaluation/ltm_trajectory_sequence_executor.py \
    --bounding_box_post_process_base_directory ${bounding_box_post_process_base_directory} \
    --config_file_directory ${config_file_directory} \
    --orb_slam_out_directory ${orb_slam_out_directory} \
    --rosbag_file_directory ${rosbag_file_directory} \
    --orb_post_process_base_directory ${orb_post_process_base_directory} \
    --calibration_file_directory ${calibration_file_directory} \
    --trajectory_sequence_file_directory ${trajectory_sequence_file_directory} \
    --results_root_directory ${results_root_directory} \
    --config_file_base_name ${config_file_base_name} \
    --sequence_file_base_name ${sequence_file_base_name} \
    --output_bb_assoc --record_viz_rosbag --log_to_file --run_rviz 
    # --lego_loam_out_root_dir ${lego_loam_out_root_dir} \



