import argparse
import os
import warnings

from plotter_utils import *

kTrajectoryFilename = "trajectory.csv"
kGTFIlename = "lego_loam_poses.csv"

# TODO add helper
# Example usage: 
# python src/evaluation/sequence_trajectory_plotter.py --approaches_and_results_root_dirs_filename metrics_plotting_configs/podman_sequence_trajectory.csv --sequence_path sequences/amazon_0523_v0.json --config_name amazon_0523_base  --output_dir figs
def parseArgs():
    parser = argparse.ArgumentParser(description='Plot trajectories for a single sequence.')
    parser.add_argument('--approaches_and_results_root_dirs_filename', required=True, default="", type=str)
    parser.add_argument('--sequence_path', required=True, default="", type=str)
    parser.add_argument('--config_name', required=True, default="", type=str)
    parser.add_argument('--output_dir', required=False, default="", type=str)
    args = parser.parse_args()
    return args

def runPlotter(cmdLineArgs):
    results_root_dirs_dict = parseCommonSeparatePlottingConfigCSV(cmdLineArgs.approaches_and_results_root_dirs_filename, checkPath=True)
    seq_id, bagnames = parseSeqIdAndBagnamesFromSequenceFile(cmdLineArgs.sequence_path)
    for i, bagname in enumerate(bagnames):
        seq_bagname = str(i) + "_" + bagname
        savepath = None
        if (cmdLineArgs.output_dir) and (not os.path.exists(cmdLineArgs.output_dir)):
            warnings.warn("The specified output directory doesn't exist. Not saving files.")
        elif cmdLineArgs.output_dir:
            savepath = os.path.join(cmdLineArgs.output_dir, seq_bagname)
        paths_dict = {}
        for approach_name, results_root_dir in results_root_dirs_dict.items():
            if approach_name not in kApproachNames:
                warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
                continue
            if approach_name == kObViSLAMApproachName:
                paths_dict[approach_name] = os.path.join(results_root_dir, seq_id, cmdLineArgs.config_name, seq_bagname, "postprocessing", kTrajectoryFilename)
            elif approach_name == kGTApproachName:
                paths_dict[approach_name] = os.path.join(results_root_dir, bagname, "poses", kGTFIlename)
            else:
                paths_dict[approach_name] = os.path.join(results_root_dir, seq_id, seq_bagname, kTrajectoryFilename)
        plot_single_session_trajectory_2d(paths_dict, savepath=savepath)

if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    runPlotter(cmdLineArgs)
    