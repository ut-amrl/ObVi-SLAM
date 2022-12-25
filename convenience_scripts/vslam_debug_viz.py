import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--root', default="/robodata/taijing/object-slam/vslam/debug", help="")
parser.add_argument('--bagname', default="1668019589")

def plot_poses(args):
    pose_directories = {}
    pose_directories["init"] = os.path.join(args.pose_directory, "init")
    pose_directories["est"]  = os.path.join(args.pose_directory, "est")


if __name__ == '__main__':
    args = parser.parse_args()

    args.pose_directory = os.path.join(args.root, args.bagname)