import argparse
import numpy as np
from collections import OrderedDict

def parse_args():
    parser = argparse.ArgumentParser(description="Convert object detections from OA-SLAM to ObVi-SLAM format") 
    parser.add_argument("--inpath", required=True, type=str, help="input detection file .json")
    parser.add_argument("--tpath", required=True, type=str, help="")
    parser.add_argument("--outpath", required=True, type=str, help="output detection file .csv")
    args = parser.parse_args()
    return args

def load_stamped_poses(filepath:str) -> OrderedDict:
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    stamped_poses = OrderedDict()
    for line in fp.readlines():
        tokens = [token.strip() for token in line.split()]
        tstamp = float(tokens[0])
        stamped_poses[tstamp] = tokens[1:]
    fp.close()
    return stamped_poses

def load_timestamps(filepath:str) -> list:
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    timestamps = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        # tstamp = float(tokens[0]) + float(tokens[1]) * 1e-9
        tstamp = (int(tokens[0]), int(tokens[1]))
        timestamps.append(tstamp)
    fp.close()
    return timestamps

def associate(stamped_poses:OrderedDict, timestamps:list) -> list:
    all_timestamps = np.array(list(stamped_poses.keys()), dtype=float)
    all_poses = np.array(list(stamped_poses.values()))
    associated_stamped_poses = []; tdiffs = []
    
    for timestamp in timestamps:
        timestamp_d = timestamp[0] + timestamp[1]*1e-9
        idx = np.argmin(np.abs(all_timestamps - timestamp_d))
        stamp = all_timestamps[idx]
        pose = all_poses[idx]
        tdiff = np.abs(stamp-timestamp_d); tdiffs.append(tdiff)
        associated_stamped_poses.append((timestamp, pose))
    return associated_stamped_poses, tdiffs

def save_stamped_poses(filepath:str, associated_stamped_poses:list): 
    kDelim = ", "
    
    fp = open(filepath, "w")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    fp.write("seconds, nanoseconds, transl_x, transl_y, transl_z, quat_x, quat_y, quat_z, quat_w\n")
    for stamped_pose in associated_stamped_poses:
        stamp = stamped_pose[0]
        pose = stamped_pose[1]
        fp.write(str(stamp[0]) + kDelim + 
                 str(stamp[1]) + kDelim + 
                 str(pose[0]) + kDelim + 
                 str(pose[1]) + kDelim +
                 str(pose[2]) + kDelim +
                 str(pose[4]) + kDelim +
                 str(pose[5]) + kDelim +
                 str(pose[6]) + kDelim +
                 str(pose[3]))
        fp.write("\n")
    fp.close()


if __name__ == "__main__":
    args = parse_args()
    stamped_poses = load_stamped_poses(args.inpath)
    timestamps = load_timestamps(args.tpath)
    associated_stamped_poses, tdiffs = associate(stamped_poses, timestamps)
    save_stamped_poses(args.outpath, associated_stamped_poses)