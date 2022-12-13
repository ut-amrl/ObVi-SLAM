#!/bin/bash

stage="0"
while getopts s flag
do
    case "${flag}" in
        s) stage=${OPTARG};;
    esac
done

bagname="1668019589"
CALIB_DIR="/robodata/taijing/object-slam/calibration/husky_zed/"
SLAM_DIR="/home/tiejean/projects/ut_semantic_vslam/"
ORB_DIR="/home/tiejean/projects/ORB_SLAM2/"
DATA_DIR="/robodata/taijing/object-slam/vslam/"
BAG_DIR="/robodata/husky_logs/"

bagfile=${BAG_DIR}${bagname}.bag

ORB_OUT_DIR=${DATA_DIR}orb_out/${bagname}/
VSLAM_IN_DIR=${DATA_DIR}vslam_in/${bagname}/
VSLAM_IN_SPARSE_DIR=${DATA_DIR}vslam_in_sparse/${bagname}/
[ ! -d $ORB_OUT_DIR ]         && mkdir -p $ORB_OUT_DIR
[ ! -d $VSLAM_IN_DIR ]        && mkdir -p $VSLAM_IN_DIR
[ ! -d $VSLAM_IN_SPARSE_DIR ] && mkdir -p $VSLAM_IN_SPARSE_DIR

echo "bagfile:        "$bagfile
echo "slam directory: "$SLAM_DIR
echo "ORB  directory: "$ORB_DIR
echo "ORB  Output directory: "$ORB_OUT_DIR
echo "SLAM Intput directory: "$VSLAM_IN_DIR
echo "SLAM Intput directory (Sparse): "$VSLAM_IN_SPARSE_DIR

rosparam set /use_sim_time true

# (roslaunch /robodata/taijing/scripts/launch/decompress-images.launch) &
# decompress_pid=$!

if [[ "$stage" < "1" ]]; then # run ORB_SLAM2
    echo "start running ORB_SLAM2...."
    cd $ORB_DIR
    ./build.sh $$ ./build_ros.sh
    (rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/husky_zed_rectified.yaml false $ORB_OUT_DIR false) &
    orb_pid=$!
    sleep 5 # make sure orb slam finish loading the vocabulary file
    rosbag play --clock $bagfile -r .25
    sleep 1
    kill -9 $orb_pid
    echo "finish running ORB_SLAM2!"
fi

if [[ "$stage" < "2" ]]; then # format vslam_in
    echo "start formating vslam_in...."
    cd $SLAM_DIR
    python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i $ORB_OUT_DIR -o $VSLAM_IN_DIR 
    make -j4
    ./bin/initialize_traj_and_feats_from_orb_out --raw_data_path $ORB_OUT_DIR --calibration_path $CALIB_DIR --processed_data_path $VSLAM_IN_DIR
    ./bin/orb_trajectory_sparsifier -input_processed_data_path $VSLAM_IN_DIR --output_processed_data_path $VSLAM_IN_SPARSE_DIR
    echo "finish formating vslam_in!"
fi
