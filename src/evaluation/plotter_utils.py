import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from brokenaxes import brokenaxes
from scipy.spatial.transform import Rotation as R

from collections import defaultdict
import os
import warnings
import json

kTranslNameX = " transl_x"
kTranslNameY = " transl_y"
kTranslNameZ = " transl_z"
kOrientNameQx = " quat_x"
kOrientNameQy = " quat_y"
kOrientNameQz = " quat_z"
kOrientNameQw = " quat_w"
kWaypointName = " waypoint_id"

kATETranslErrorType = "transl_ate"
kATEOrientErrorType = "orient_ate"

kATEErrorTypes = set([
    kATETranslErrorType, \
    kATEOrientErrorType
])

kATETranslErrorYLabel = "RMSE (m)"
kATEOrientErrorYLabel = "RMSE (deg)"

kATEErrorYLabelDict = {
    kATETranslErrorType: kATETranslErrorYLabel, \
    kATEOrientErrorType: kATEOrientErrorYLabel
}

kObViSLAMApproachName = "ObVi-SLAM"
kORBSLAM3ApproachName = "ORB-SLAM3"
kOASLAMApproachName = "OA-SLAM3"
kGTApproachName = "Pseudo-Groundtruth"

kApproachNames = set([
    kObViSLAMApproachName, \
    kORBSLAM3ApproachName, \
    kOASLAMApproachName, \
    kGTApproachName
])

kObViSLAMColor = "tab:blue"
kORBSLAM3Color = "tab:orange"
kOASLAMColor = "tab:green"
kGTColor = "tab:red"

kApproachColorDict = {
    kObViSLAMApproachName: kObViSLAMColor, \
    kORBSLAM3ApproachName: kORBSLAM3Color, \
    kOASLAMApproachName: kOASLAMColor, \
    kGTApproachName: kGTColor
}

kObViSLAMLineStyle = "solid"
kORBSLAM3LineStyle = "dotted"
kOASLAMLineStyle = "dashdot"
kGTLineStyle = "dashed"

kApproachLineStyleDict = {
    kObViSLAMApproachName: kObViSLAMLineStyle, \
    kORBSLAM3ApproachName: kORBSLAM3LineStyle, \
    kOASLAMApproachName: kOASLAMLineStyle, \
    kGTApproachName: kGTLineStyle
}

kObViSLAMLinewidth = 3
kORBSLAM3Linewidth = 4
kOASLAMLinewidth = 2
kGTLinewidth = 3

kApproachLinewidthDict = {
    kObViSLAMApproachName: kObViSLAMLinewidth, \
    kORBSLAM3ApproachName: kORBSLAM3Linewidth, \
    kOASLAMApproachName: kOASLAMLinewidth, \
    kGTApproachName: kGTLinewidth
}

kObViSLAMMarker = "X"
kORBSLAM3Marker = "o"
kOASLAMMarker = "P"
kApproachMarkerDict = {
    kObViSLAMApproachName: kObViSLAMMarker, \
    kORBSLAM3ApproachName: kORBSLAM3Marker, \
    kOASLAMApproachName: kOASLAMMarker
}

kObViSLAMMakerSize = 100
kORBSLAM3MMakerSize = 100
kOASLAMMarkerSize = 100

kApproachMarkerSizeDict = {
    kObViSLAMApproachName: kObViSLAMMakerSize, \
    kORBSLAM3ApproachName: kORBSLAM3MMakerSize, \
    kOASLAMApproachName: kOASLAMMarkerSize
}

kTopdownTrajColor = "#969696"
kTopdownTrajWaypointColors = ["mediumorchid","maroon","mediumpurple","hotpink","indianred","magenta"]
kTopdownTrajWaypointHalfWidth = 1.2
kTopdownTrajWaypointHalfHeight = .6
kTopdownTrajWaypointCorner0 = [kTopdownTrajWaypointHalfWidth,  kTopdownTrajWaypointHalfHeight]
kTopdownTrajWaypointCorner1 = [-kTopdownTrajWaypointHalfWidth, kTopdownTrajWaypointHalfHeight]
kTopdownTrajWaypointCorner2 = [-kTopdownTrajWaypointHalfWidth, -kTopdownTrajWaypointHalfHeight]
kTopdownTrajWaypointCorner3 = [kTopdownTrajWaypointHalfWidth,  -kTopdownTrajWaypointHalfHeight]
kTopdownTrajWaypointCorneres = [kTopdownTrajWaypointCorner0, kTopdownTrajWaypointCorner1, kTopdownTrajWaypointCorner2, kTopdownTrajWaypointCorner3]

kAxisFontsize = 20
kGridAlpha = .4

def readCommonSeparateCSVWithoutIndices(path):
    if not os.path.exists(path):
        raise FileNotFoundError("Fail to open file " + path)
    return pd.read_csv(path, delimiter=",")
    
def parseCommonSeparatePlottingConfigCSV(path, checkPath=False):
    paths_dict = {}
    if checkPath and (not os.path.exists(path)):
        raise FileNotFoundError("Fail to open file " + path)
    df = pd.read_csv(path, delimiter=",", header = None)
    for row_idx, row in df.iterrows():
        paths_dict[row[0]] = row[1]
    return paths_dict

def parseSeqIdAndBagnamesFromSequenceFile(path):
    fp = open(path, "r")
    if fp.closed:
        raise FileNotFoundError("Fail to open file " + path)
    data = json.load(fp)["sequence_info"]
    seq_id = data["seq_id"]
    bagnames = []
    for baginfo in data["sequence"]:
        bagnames.append(baginfo["bag_base_name"])
    fp.close()
    return seq_id, bagnames

# TODO need to apply transformation across different trajectories
# need to add the transformation dict to handle this
def plot_single_session_trajectory_2d(paths_dict, legend_loc="lower left", xlim=[-65, 20], ylim=[-32, 32], savepath=None):
    plt.figure()
    # Assume all trajectories start from origin (0,0)
    plt.scatter(0, 0, marker="*", s=400, label="Start & Goal Loc", color="gold")
    for approach_name, path in paths_dict.items():
        if not os.path.exists(path):
            warnings.warn("Failed to find file " + path)
            continue
        if approach_name not in kApproachNames:
            warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
            continue
        if approach_name == kGTApproachName:
            # TODO double-check if the header is correctly formatted when adding transformation dict
            df = pd.read_csv(path, delimiter=" ", header=None)
            df = df.rename(columns={0:"timestamp", 1:kTranslNameX, 2:kTranslNameY, 3:kTranslNameZ, 4:kOrientNameQw, 5:kOrientNameQx, 6:kOrientNameQy, 7:kOrientNameQz})
        else:
            df = readCommonSeparateCSVWithoutIndices(path)
        # TODO fix hardcoding using transformation dict here
        if approach_name in set([kObViSLAMApproachName, kGTApproachName]):
            plt.plot(-df[kTranslNameY], df[kTranslNameX], \
                label=approach_name, \
                color=kApproachColorDict[approach_name], \
                linestyle=kApproachLineStyleDict[approach_name], \
                linewidth=kApproachLinewidthDict[approach_name])
        elif approach_name in set([kORBSLAM3ApproachName, kOASLAMApproachName]):
            plt.plot(df[kTranslNameX], df[kTranslNameZ], \
                label=approach_name, \
                color=kApproachColorDict[approach_name], \
                linestyle=kApproachLineStyleDict[approach_name], \
                linewidth=kApproachLinewidthDict[approach_name])
        else:
            warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
    plt.legend(loc=legend_loc)
    plt.xlabel("x (m)", fontsize=kAxisFontsize)
    plt.ylabel("y (m)", fontsize=kAxisFontsize)
    plt.grid(alpha=kGridAlpha)
    plt.axis("equal")
    plt.tight_layout()
    if savepath:
        print("Saving figure to " + savepath)
        plt.savefig(savepath)
    else:
        plt.show()

def load_approach_ATE_metrics(paths_dict):
    transl_errs_dict = defaultdict(list); orient_errs_dict = defaultdict(list)
    for approach_name, path in paths_dict.items():
        fp = open(path, "r")
        if fp.closed:
            warnings.warn("Failed to find file " + path)
            continue
        data = json.load(fp)
        for result in data["metrics"]["indiv_trajectory_metrics"]:
            transl_err = result["v"]["trajectory_sequence_ate_results"]["rmse_transl_err"]
            orient_err = result["v"]["trajectory_sequence_ate_results"]["rmse_rot_err"] / np.pi *180
            transl_errs_dict[approach_name].append(transl_err)
            orient_errs_dict[approach_name].append(orient_err)
        fp.close()
    return transl_errs_dict, orient_errs_dict

def plot_single_sequence_rmse(errs_dict, err_type, ylims=[], legend_loc="upper left", savepath=None):
    plt.figure()
    bax = brokenaxes(ylims=ylims)
    for approach_name, errs in errs_dict.items():
        if approach_name not in kApproachNames:
            warnings.warn("Undefined approach name " + approach_name + ". Skip plotting trajectory...")
            continue
        xx = np.arange(len(errs)) + 1
        bax.scatter(xx, errs, label=approach_name, \
            color=kApproachColorDict[approach_name], \
            marker=kApproachMarkerDict[approach_name], \
            s=kApproachMarkerSizeDict[approach_name])
    bax.set_xlabel("Bagfile Index", fontsize=kAxisFontsize)
    bax.set_ylabel(kATEErrorYLabelDict[err_type], fontsize=kAxisFontsize)
    bax.legend(loc=legend_loc)
    bax.grid(alpha=0.4)
    # Note: cannot use tight_layout. It'll break the brokenaxis
    if savepath:
        print("Saving figure to " + savepath)
        plt.savefig(savepath)
    else:
        plt.show()

def plot_topdown_trajectory(poses_dict, xlim=None, ylim=None, figsize=None, show_axis=True, savepath=None):
    if figsize is None:
        plt.figure(constrained_layout=True)
    else:
        plt.figure(constrained_layout=True, figsize=figsize)
    waypoints_df = None
    for bagid, pose_df in poses_dict.items():
        plt.plot(pose_df[kTranslNameX], pose_df[kTranslNameY], color=kTopdownTrajColor, zorder=-1)
        if waypoints_df is None:
            waypoints_df = pose_df[pose_df[kWaypointName] != -1]
        else:
            waypoints_df = pd.concat([waypoints_df, pose_df[pose_df[kWaypointName] != -1]])
    for name, group in waypoints_df.groupby(kWaypointName):
        for _, data in group.iterrows():
            points = []
            transl = np.array([data[kTranslNameX], data[kTranslNameY]])
            yaw = R.from_quat(data[[kOrientNameQx,kOrientNameQy,kOrientNameQz,kOrientNameQw]]).as_rotvec()[2]
            for corner in kTopdownTrajWaypointCorneres:
                rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
                point = transl + rot_mat @ corner
                points.append(point)
            points.append(points[0])
            points = np.array(points)
            plt.fill(points[:,0], points[:,1], color=kTopdownTrajWaypointColors[int(name) % len(kTopdownTrajWaypointColors)])
            # plt.plot(points[:,0], points[:,1], color="black", linewidth=0.5)
    plt.xlabel("x (m)", fontsize=kAxisFontsize)
    plt.ylabel("y (m)", fontsize=kAxisFontsize)
    if show_axis:
        plt.grid(alpha=0.4)
        plt.axis("equal")
    else:
        plt.grid(False)
        plt.axis('off')
    if xlim:
        plt.xlim(xlim)
        plt.xticks(np.arange(xlim[0], xlim[1]+1, 10))
    if ylim:
        plt.ylim(ylim)
        plt.yticks(np.arange(ylim[0], ylim[1]+1, 10))
    if savepath:
        print("Saving figure to " + savepath)
        plt.savefig(savepath)
    else:
        plt.show()