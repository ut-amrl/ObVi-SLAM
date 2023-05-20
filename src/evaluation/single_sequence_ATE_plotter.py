import argparse
import os

from plotter_utils import *

kMetricsFilename = "metrics.json"
# change this configarations accordingly
kATETranslErrorYLims = [(0, 3.5), (10, 11)]
kATEOrientErrorYLims = [(0, 13), (20,30), (38,40)]

# TODO add helper
# Example usage:
# python src/evaluation/single_sequence_ATE_plotter.py --approaches_and_results_root_dirs_filename metrics_plotting_configs/podman_results_root_dir.csv 
#   --sequence_path sequences/amazon_0523_v0.json --config_name amazon_0523_base  --error_types_and_savepaths_file_name metrics_plotting_configs/savepaths/test_ate_savepath.csv
def parseArgs():
    parser = argparse.ArgumentParser(description='Plot ATE for a single sequence.')
    parser.add_argument('--approaches_and_results_root_dirs_filename', required=True, default="", type=str)
    # Parse in sequence file in case we want to set x-axis to bagnames rather than bag indices
    parser.add_argument('--sequence_path', required=True, default="", type=str)
    parser.add_argument('--config_name', required=True, default="", type=str)
    parser.add_argument('--error_types_and_savepaths_file_name', required=False, default="", type=str)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    results_root_dirs_dict = parseCommonSeparatePlottingConfigCSV(cmdLineArgs.approaches_and_results_root_dirs_filename, checkPath=True)
    output_paths_dict = {}
    if cmdLineArgs.error_types_and_savepaths_file_name:
        output_paths_dict = parseCommonSeparatePlottingConfigCSV(cmdLineArgs.error_types_and_savepaths_file_name, checkPath=False)
    seq_id, bagnames = parseSeqIdAndBagnamesFromSequenceFile(cmdLineArgs.sequence_path)
    paths_dict = {}
    for approach_name, result_root_dir in results_root_dirs_dict.items():
        if approach_name not in kApproachNames:
            warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
            continue
        if approach_name == kGTApproachName:
            continue
        if approach_name == kObViSLAMApproachName:
            paths_dict[approach_name] = os.path.join(result_root_dir, seq_id, cmdLineArgs.config_name, kMetricsFilename)
        else:
            paths_dict[approach_name] = os.path.join(result_root_dir, seq_id, kMetricsFilename)
    transl_errs_dict, orient_errs_dict = load_approach_ATE_metrics(paths_dict)
    
    savepath = None
    if len(output_paths_dict) != 0:
        savepath = output_paths_dict[kATETranslErrorType]
    plot_single_sequence_rmse(transl_errs_dict, kATETranslErrorType, ylims=kATETranslErrorYLims, savepath=savepath)

    savepath = None
    if len(output_paths_dict) != 0:
        savepath = output_paths_dict[kATEOrientErrorType]
    plot_single_sequence_rmse(orient_errs_dict, kATEOrientErrorType, ylims=kATEOrientErrorYLims, savepath=savepath)