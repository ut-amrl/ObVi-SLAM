import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--root', default="/robodata/taijing/object-slam/vslam/debug", help="")
parser.add_argument('--bagname', default="1668019589")
parser.add_argument('--frame_id', default=1, type=int)

# assume more than 1 case
debug_labels = {"init": "initalization", "est": "after optimization"}

def plot_poses(args):
    pose_directories = {}
    for key, val in debug_labels.items():
        pose_directories[key] = os.path.join(args.pose_directory, key)

def plot_pointclouds_by_frameid(args, frame_id):
    pcl_directories = {}
    for key, val in debug_labels.items():
        pcl_directories[key] = os.path.join(args.pcl_directory, key)
    filename = str(frame_id) + ".csv"
    plot_df = pd.DataFrame(columns=["feature_id", "x", "y", "z", "label"])
    for case, pcl_directory in pcl_directories.items():
        filepath = os.path.join(pcl_directory, filename)
        points_df = pd.read_csv(filepath)
        points_df["label"] = debug_labels[case]
        plot_df = plot_df.append(points_df)
    kOutlierThresh = 100
    mask = (plot_df["x"] < kOutlierThresh) & (plot_df["x"] > -kOutlierThresh) \
        & (plot_df["y"] < kOutlierThresh) & (plot_df["y"] > -kOutlierThresh) \
        & (plot_df["z"] < kOutlierThresh) & (plot_df["z"] > -kOutlierThresh)
    plot_df = plot_df[mask]
    fig = px.scatter_3d(plot_df, x='x', y='y', z='z', color='label')
    fig.update_traces(marker_size = 2.5)
    savepath = os.path.join(args.pcl_directory, str(frame_id) + ".html")
    print("savepath: ", savepath)
    fig.write_html(savepath)

def plot_pointclouds(args):
    pcl_directories = {}
    for key, val in debug_labels.items():
        pcl_directories[key] = os.path.join(args.pcl_directory, key)
    
    cases = list(pcl_directories.keys())
    # determine if plotting differences between two cases by lines
    plot_diff = False
    if len(cases) == 2:
        plot_diff = True

    key_case = cases[0]; cases = set(cases)
    if len(cases) <= 1:
        print("Undefined control flow! Exiting...")
        exit(0)
    cases.remove(key_case); cases = list(cases)

    for filename in os.listdir(pcl_directories[key_case]):
        frame_id = filename.split(".")[0]
        plot_figs = []
        plot_df = pd.DataFrame(columns=["feature_id", "x", "y", "z", "label"])
        for case, pcl_directory in pcl_directories.items():
            filepath = os.path.join(pcl_directory, filename)
            points_df = pd.read_csv(filepath)
            points_df["label"] = case
            plot_df = plot_df.append(points_df)
        fig = px.scatter_3d(plot_df, x='x', y='y', z='z', color='label')
        plot_figs.append(fig)
        
        if plot_diff:
            case1, case2 = key_case, cases[0]
            filepath1, filepath2 = os.path.join(pcl_directories[case1], filename), os.path.join(pcl_directories[case2], filename)
            points1_df, points2_df = pd.read_csv(filepath1), pd.read_csv(filepath2)
            plot_df = pd.DataFrame(columns=["feature_id", "x", "y", "z", "label"])
            for idx, row in points1_df.iterrows():
                feature_id = row["feature_id"]
            point1_df, point2_df = points1_df[points1_df["feature_id"]==feature_id], points2_df[points2_df["feature_id"]==feature_id]
            point1_df["label"], point2_df["label"] = case1, case2
            print("point1_df: ", point1_df)
            print("point2_df: ", point2_df)
            plot_df.append(point1_df); plot_df.append(point2_df)
            exit(0)

if __name__ == '__main__':
    args = parser.parse_args()

    args.root_directory = os.path.join(args.root, args.bagname)
    args.pose_directory = os.path.join(args.root_directory, "poses")
    args.pcl_directory  = os.path.join(args.root_directory, "pointclouds")
    plot_pointclouds_by_frameid(args, args.frame_id)