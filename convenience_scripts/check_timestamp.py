import os
import argparse
from collections import defaultdict

import rosbag
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument('--bagfile', 
                    default="/robodata/taijing/uncertainty-aware-perception/bags/uncertainty_husky_2022-11-07-12-02-34.bag",
                    help="")

def read_bag_timestamp(filepath, topics):
    timestamps_dict = defaultdict(list)
    for topic, msg, t in rosbag.Bag(filepath, "r").read_messages():
        if topic not in topics:
            continue
        timestamps_dict[topic].append(t)
    return timestamps_dict

def read_pose_timestamp(filepath, topics):
    timestamps_dict = defaultdict(list)
    timestamps_dict["pose"] = pd.read_csv(filepath, header=None).values[:,0]
    return timestamps_dict

def read_timestamp(topics, bagfile=None, filepath=None):
    pass

if __name__ == '__main__':
    args = parser.parse_args()
    timestamps_dict = read_bag_timestamp(args.bagfile, ["/stereo/left/image_raw/compressed", "/ouster/lidar_packets", "/ouster/points"])
