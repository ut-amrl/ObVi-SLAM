./bin/format_tum_gt \
    --gt_filepath_in /home/tiejean/Documents/mnt/oslam/TUM/freiburg2_desk/groundtruth.txt \
    --gt_filepath_out /home/tiejean/Documents/mnt/oslam/lego_loam_out/freiburg2_desk/poses/lego_loam_poses.csv \
    --time_filepath /home/tiejean/Documents/mnt/oslam/orb_post_process/sparsified_ut_vslam_in/tum_fr2_desk/freiburg2_desk/timestamps/timestamps_only.txt \
    --gt_postprocessed_filepath_out /home/tiejean/Documents/mnt/oslam/ut_vslam_results/tum_fr2_desk/tum_fr2_desk/0_freiburg2_desk/postprocessing/interpolated_lego_loam_poses.csv 

# python src/evaluation/tum/interpolate_tum_gt.py \
#     --inpath /home/tiejean/Documents/mnt/oslam/lego_loam_out/freiburg2_desk/poses/lego_loam_poses.csv \
#     --tpath /home/tiejean/Documents/mnt/oslam/orb_post_process/sparsified_ut_vslam_in/tum_fr2_desk/freiburg2_desk/timestamps/timestamps_only.txt \
#     --outpath /home/tiejean/Documents/mnt/oslam/ut_vslam_results/tum_fr2_desk/tum_fr2_desk/0_freiburg2_desk/postprocessing/interpolated_lego_loam_poses.csv