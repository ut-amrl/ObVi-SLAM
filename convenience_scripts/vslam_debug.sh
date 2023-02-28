#!/bin/bash

bagname="1668019589"
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

params_config_file=config/3.json
source ~/.bashrc
rosparam set /use_sim_time false

# (roslaunch /robodata/taijing/scripts/launch/decompress-images.launch) &
# decompress_pid=$!

if [[ "$stage" < "1" ]]; then # run ORB_SLAM2
    echo "start running ORB_SLAM2...."
    cd $ORB_DIR
    echo "removing directory "$ORB_OUT_DIR"..."
    rm -rf $ORB_OUT_DIR/*
    sleep 5
    ./build.sh && ./build_ros.sh
    # (rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/husky_zed_rectified.yaml false $ORB_OUT_DIR false) &
    (rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/husky_zed_rectified_high_res.yaml false $ORB_OUT_DIR false) &
    orb_pid=$!
    echo "launching ORB_SLAM2"
    sleep 8 # make sure orb slam finish loading the vocabulary file
    # rosbag play --clock $bagfile -r .25 --duration=30
    rosbag play --clock $bagfile -r .25
    sleep 1
    kill -9 $orb_pid
    echo "finish running ORB_SLAM2!"
fi

if [[ "$stage" < "2" ]]; then # format vslam_in
    echo "start formating vslam_in...."
    cd $SLAM_DIR
    echo "removing directory "$VSLAM_IN_DIR"..."
    rm -rf $VSLAM_IN_DIR/*
    sleep 5
    python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i $ORB_OUT_DIR -o $VSLAM_IN_DIR 
    make -j4
    ./bin/initialize_traj_and_feats_from_orb_out --raw_data_path $ORB_OUT_DIR --calibration_path $CALIB_DIR --processed_data_path $VSLAM_IN_DIR
    echo "removing directory "$VSLAM_IN_SPARSE_DIR"..."
    rm -rf $VSLAM_IN_SPARSE_DIR/*
    sleep 5
   ./bin/orb_trajectory_sparsifier -input_processed_data_path $VSLAM_IN_DIR --output_processed_data_path $VSLAM_IN_SPARSE_DIR --params_config_file $params_config_file
    echo "finish formating vslam_in!"
fi

if [[ "$stage" < "3" ]]; then # running slam
    rosparam set /use_sim_time false
    echo "start running slam...."
    cd $YOLO_DIR
    (python3 detect_ros.py --weights $yolo_weight --img 640 --conf 0.1) &
    yolo_pid=$!
    sleep 5

    cd $SLAM_DIR
    # (rosrun rviz rviz -d ovslam.rviz) &
    rviz_pid=$!
    sleep 3
    
    make -j8
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
    echo "./bin/offline_object_visual_slam_main --intrinsics_file ${intrinsics_file} \
        --extrinsics_file ${extrinsics_file} --bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file} \
        --poses_by_node_id_file ${poses_by_node_id_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
        --rosbag_file ${rosbag_file} --long_term_map_output ${long_term_map_output_file} \
        --low_level_feats_dir ${low_level_feats_dir}  --params_config_file $params_config_file \
        --vslam_debugger_directory ${debug_output_directory}"
    echo "\n"

    ./bin/offline_object_visual_slam_main --intrinsics_file ${intrinsics_file} \
        --extrinsics_file ${extrinsics_file} --bounding_boxes_by_node_id_file ${bounding_boxes_by_node_id_file} \
        --poses_by_node_id_file ${poses_by_node_id_file} --nodes_by_timestamp_file ${nodes_by_timestamp_file} \
        --rosbag_file ${rosbag_file} --long_term_map_output ${long_term_map_output_file} \
        --low_level_feats_dir ${low_level_feats_dir} --params_config_file $params_config_file \
        --vslam_debugger_directory ${debug_output_directory}

    kill -9 $yolo_pid
    echo "finish running slam!"
fi

if [[ "$stage" < "4" ]]; then

source /home/tiejean/projects/my_venv/bin/activate
# python ./convenience_scripts/vslam_debug_viz.py
deactivate

fi
