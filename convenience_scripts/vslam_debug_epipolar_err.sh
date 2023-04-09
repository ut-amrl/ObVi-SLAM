#!/bin/bash

bagname="1677097326"
# CALIB_DIR="/robodata/taijing/object-slam/calibration/husky_zed_resolution_2_scale_0_5/"
CALIB_DIR="/robodata/taijing/object-slam/calibration/husky_zed_resolution_1_scale_0_5/"
SLAM_DIR="/home/tiejean/projects/ut_semantic_vslam/"
ORB_DIR="/home/tiejean/projects/ORB_SLAM2/"
DATA_DIR="/robodata/taijing/object-slam/vslam/"
BAG_DIR="/robodata/taijing/object-slam/"
YOLO_DIR="/home/tiejean/projects/yolov5-ros/"
yolo_weight="/robodata/taijing/object-slam/yolov5-models/outdoors-final-yolov5s-0.pt"

stage="0"
while getopts s:d:b:c:y:w:n flag
do
    case "${flag}" in
        s) stage=${OPTARG};;
        d) DATA_DIR=${OPTARG};;
        b) BAG_DIR=${OPTARG};;
        c) CALIB_DIR=${OPTARG};;
        y) YOLO_DIR=${OPTARG};;
        w) yolo_weight=${OPTARG};;
        n) bagname=${OPTARG};;
    esac
done

bagfile=${BAG_DIR}${bagname}.bag

ORB_OUT_DIR=${DATA_DIR}orb_out/${bagname}/
VSLAM_IN_DIR=${DATA_DIR}vslam_in/${bagname}/
VSLAM_IN_SPARSE_DIR=${DATA_DIR}vslam_in_sparse/${bagname}/
VSLAM_OUT_DIR=${DATA_DIR}vslam_out/${bagname}/
DEBUG_OUT_DIR=${DATA_DIR}debug/${bagname}/
[ ! -d $ORB_OUT_DIR ]         && mkdir -p $ORB_OUT_DIR
[ ! -d $VSLAM_IN_DIR ]        && mkdir -p $VSLAM_IN_DIR
[ ! -d $VSLAM_IN_SPARSE_DIR ] && mkdir -p $VSLAM_IN_SPARSE_DIR
[ ! -d $VSLAM_OUT_DIR ]       && mkdir -p $VSLAM_OUT_DIR
[ ! -d $DEBUG_OUT_DIR ]       && mkdir -p $DEBUG_OUT_DIR

echo "bagfile:        "$bagfile
echo "slam directory: "$SLAM_DIR
echo "ORB  directory: "$ORB_DIR
echo "ORB  Output directory: "$ORB_OUT_DIR
echo "SLAM Intput directory: "$VSLAM_IN_DIR
echo "SLAM Intput directory (Sparse): "$VSLAM_IN_SPARSE_DIR
echo "Debug Output directory: "$DEBUG_OUT_DIR

params_config_file=config/7.json
source ~/.bashrc
rosparam set /use_sim_time false

# (roslaunch /robodata/taijing/scripts/launch/decompress-images.launch) &
# decompress_pid=$!

# echo "start running slam...."
# cd $YOLO_DIR
# (python3 detect_ros.py --weights $yolo_weight --img 640 --conf 0.1) &
# yolo_pid=$!
# sleep 5

cd $SLAM_DIR
# (rosrun rviz rviz -d ovslam.rviz) &
# rviz_pid=$!
# sleep 3

# make -j8
# ./bin/orb_trajectory_sparsifier -input_processed_data_path $VSLAM_IN_DIR --output_processed_data_path $VSLAM_IN_SPARSE_DIR --params_config_file $params_config_file
# echo "finish sparsifying!"
# sleep 10

intrinsics_file=${CALIB_DIR}camera_matrix.txt
extrinsics_file=${CALIB_DIR}extrinsics.txt
bounding_boxes_by_node_id_file="/home/tiejean/Documents/mnt/vslam/dummy/1668019589/bounding_boxes_by_node.csv" # TODO FIXME
vslam_in_directory=${VSLAM_IN_SPARSE_DIR}
poses_by_node_id_file=${vslam_in_directory}poses/initial_robot_poses_by_node.txt
nodes_by_timestamp_file=${vslam_in_directory}timestamps/node_ids_and_timestamps.txt
rosbag_file=${bagfile}
long_term_map_output_file=${VSLAM_OUT_DIR}long_term_map.json
low_level_feats_dir=${vslam_in_directory}
debug_output_directory=${DEBUG_OUT_DIR}

echo "\n"
echo "intrinsics_file:           "$intrinsics_file
echo "extrinsics_file:           "$extrinsics_file
echo "poses_by_node_id_file:     "$poses_by_node_id_file
echo "nodes_by_timestamp_file:   "$nodes_by_timestamp_file
echo "rosbag_file:               "$rosbag_file
echo "long_term_map_output_file: "$long_term_map_output_file
echo "low_level_feats_dir:       "$low_level_feats_dir
echo "\n"

./bin/visualize_epipolar_error --target_frame_id_1 586 --target_frame_id_2 589 --debug_output_directory "/robodata/taijing/object-slam/vslam/debug/tmp/" --intrinsics_file ${intrinsics_file} \
    --extrinsics_file ${extrinsics_file} --bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file} \
    --poses_by_node_id_file ${poses_by_node_id_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
    --rosbag_file ${rosbag_file} --long_term_map_output ${long_term_map_output_file} \
    --low_level_feats_dir ${low_level_feats_dir} --params_config_file $params_config_file \
    --vslam_debugger_directory ${debug_output_directory}

echo "finish running slam!"

