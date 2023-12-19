SLAMDIR=/home/tiejean/Documents/projects/ut_vslam
DARADIR=/home/tiejean/Documents/mnt/oslam
ORB_OUT=$DARADIR/orb_out
ORB_POST_PROCESS=$DARADIR/orb_post_process

bagname=freiburg2_desk
configname=base7a_2_fallback_a_2

configfile=$SLAMDIR/config/${base7a_2_fallback_a_2}.json
calibration_dir=$DARADIR/calibration/
orb_out_dir=$ORB_OUT/$bagname/
unsparsified_orb_out=$ORB_POST_PROCESS/unsparsified_ut_vslam_in/$bagname/
sparsified_orb_out=$ORB_POST_PROCESS/sparsified_ut_vslam_in/$configname/$bagname/

mkdir -p $orb_out_dir
mkdir -p $unsparsified_orb_out
mkdir -p $sparsified_orb_out

python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py \
    -i $orb_out_dir -o $unsparsified_orb_out

./bin/initialize_traj_and_feats_from_orb_out \
    --raw_data_path $orb_out_dir \
    --calibration_path $calibration_dir \
    --processed_data_path $unsparsified_orb_out

./bin/orb_trajectory_sparsifier \
    --param_prefix $bagname \
    --input_processed_data_path $unsparsified_orb_out \
    --output_processed_data_path $sparsified_orb_out \
    --params_config_file $configfile
