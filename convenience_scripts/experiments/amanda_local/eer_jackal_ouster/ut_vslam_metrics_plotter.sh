#!/bin/bash

config_file_base_name="update_rev_lower_conv_thresholds_manual_feat_true_single_phase_v2"

python3 metrics_plotting_configs/plotting_config_generator.py \
    --config_file_base_name ${config_file_base_name}

python src/evaluation/waypoint_consistency_cdf_plotter.py \
  --approaches_and_metrics_file_name ./metrics_plotting_configs/${config_file_base_name}_compare.csv \
  --error_types_and_savepaths_file_name ./metrics_plotting_configs/savepaths/comparisons_save_path_traj_${config_file_base_name}.csv

python3 src/evaluation/object_metrics_plotter.py \
  --approaches_and_metrics_file_name ./metrics_plotting_configs/object_metrics/${config_file_base_name}_compare.csv \
  --error_types_and_savepaths_file_name ./metrics_plotting_configs/savepaths/comparisons_save_path_obj_${config_file_base_name}.csv
