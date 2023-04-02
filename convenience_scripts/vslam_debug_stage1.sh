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

echo "start formating vslam_in...."
cd $SLAM_DIR
echo "removing directory "$VSLAM_IN_DIR"..."
rm -rf $VSLAM_IN_DIR/*
sleep 5
python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i $ORB_OUT_DIR -o $VSLAM_IN_DIR 
# make -j4
# ./bin/initialize_traj_and_feats_from_orb_out --raw_data_path $ORB_OUT_DIR --calibration_path $CALIB_DIR --processed_data_path $VSLAM_IN_DIR
echo "finish formating vslam_in!"