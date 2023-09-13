import pandas as pd
import argparse
import os
import warnings

from plotter_utils import *

kTrajWithWaypointsFilename = "traj_with_waypoints.csv"
kTrajWithWaypointsXlim = []
kTrajWithWaypointsYlim = []
kTrajWithWaypointsFigsize = []

def parseArgs():
    parser = argparse.ArgumentParser(description='Plot topdown view of the trajectories for a single sequence.')
    parser.add_argument('--datadir', required=True, default="", type=str)
    parser.add_argument('--sequence_path', required=True, default="", type=str)
    parser.add_argument('--savepath', required=False, default="", type=str)
    parser.add_argument('--xlim_min', required=False, default=None, type=int)
    parser.add_argument('--xlim_max', required=False, default=None, type=int)
    parser.add_argument('--ylim_min', required=False, default=None, type=int)
    parser.add_argument('--ylim_max', required=False, default=None, type=int)
    parser.add_argument('--figsize_x', required=False, default=None, type=float)
    parser.add_argument('--figsize_y', required=False, default=None, type=float)
    parser.add_argument('--is_obvi_slam', required=False, default=False, action='store_true')
    parser.add_argument('--no_duplicate_waypoints', required=False, default=False, action='store_true')
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    cmdLineArgs = parseArgs()
    if (cmdLineArgs.xlim_min is None) ^ (cmdLineArgs.xlim_max is None):
        warnings.warn("You must specify both xlim_min and xlim_max, or neither of them. Skipping your specified xlim")
    if (cmdLineArgs.ylim_min is None) ^ (cmdLineArgs.ylim_max is None):
        warnings.warn("You must specify both ylim_min and ylim_max, or neither of them. Skipping your specified ylim")
    if (cmdLineArgs.xlim_min is not None) and (cmdLineArgs.xlim_max is not None):
        kTrajWithWaypointsXlim = [cmdLineArgs.xlim_min, cmdLineArgs.xlim_max]
    if (cmdLineArgs.ylim_min is not None) and (cmdLineArgs.ylim_max is not None):
        kTrajWithWaypointsYlim = [cmdLineArgs.ylim_min, cmdLineArgs.ylim_max]
    if (cmdLineArgs.figsize_x is None) ^ (cmdLineArgs.figsize_y is None):
        warnings.warn("You must specify both figsize_x and figsize_y, or neither of them. Skipping your specified ylim")
    if (cmdLineArgs.figsize_x is not None) and (cmdLineArgs.figsize_y is not None):
        kTrajWithWaypointsFigsize = [cmdLineArgs.figsize_x, cmdLineArgs.figsize_y]
    
    seq_id, bagnames = parseSeqIdAndBagnamesFromSequenceFile(cmdLineArgs.sequence_path)
    poses_dict = {}
    for bad_idx, bagname in enumerate(bagnames):
        bag_uid = str(bad_idx)+ "_" + bagname
        if cmdLineArgs.is_obvi_slam:
            filepath = os.path.join(cmdLineArgs.datadir, bag_uid, "postprocessing", kTrajWithWaypointsFilename)
        else:
            filepath = os.path.join(cmdLineArgs.datadir, bag_uid, kTrajWithWaypointsFilename)
        poses_dict[bag_uid] = pd.read_csv(filepath)
    plot_topdown_trajectory(poses_dict, \
            xlim=kTrajWithWaypointsXlim, \
            ylim=kTrajWithWaypointsYlim, \
            figsize=kTrajWithWaypointsFigsize, \
            savepath=cmdLineArgs.savepath, \
            no_duplicate_waypoints=cmdLineArgs.no_duplicate_waypoints)

# python src/evaluation/trajectory_sequence_topdown_plotter.py --datadir /root/LTOV-SLAM-Evaluation/data/orb_slam_3_out/evaluation_2023_07_v1_orb_finally_probably/ --sequence_path sequences/evaluation_2023_07_v1.json --savepath orb