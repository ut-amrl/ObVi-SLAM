import matplotlib.pyplot as plt
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

kDatadir = "/home/tiejean/Documents/mnt/oslam/TUM_results/fr2_desk"
kObViSLAMFilepath = os.path.join(kDatadir, "obvislam.json")
kLeGOLOAMFilepath = os.path.join(kDatadir, "legoloam.csv")
kORBSLAM3Filepath = os.path.join(kDatadir, "orbslam3.csv")

def load_obvislam(filepath:str) -> np.array:
    framed_poses = []

    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    data = json.load(fp)
    for framed_pose in data["robot_poses"]["robot_pose_results_map"]:
        fid = int(framed_pose["frame_id"])
        transl = np.array(framed_pose["pose"]["transl"]["Data"])
        rotvec = framed_pose["pose"]["rot"]["angle"] * np.array(framed_pose["pose"]["rot"]["axis"]["Data"])
        quat = R.from_rotvec(rotvec).as_quat()
        framed_poses.append((fid, transl.tolist() + quat.tolist()))
    fp.close()
    
    framed_poses = sorted(framed_poses, key=lambda x: x[0])
    poses = [framed_pose[1] for framed_pose in framed_poses]
    poses = np.array(poses)
    return poses

def load_legoloam(filepath:str) -> np.array:
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        pose = [float(token) for token in tokens[2:]]
        poses.append(pose)
    fp.close()
    poses = np.array(poses)
    return poses

def load_orbslam3(filepath:str) -> np.array:
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        pose = [float(token) for token in tokens[3:]]
        poses.append(pose)
    fp.close()
    poses = np.array(poses)
    fp.close()

def plot(results:dict):
    plt.figure()
    for approach, poses in results.items():
        if approach == "ObVi-SLAM" or approach == "Groundtruth":
            plt.plot(poses[:,0], poses[:,1], label=approach)
        elif approach == "ORBSLAM3":
            plt.plot(-poses[:,2], -poses[:,1], label=approach)
    plt.legend()
    plt.axis("equal")
    plt.show()

if __name__ == "__main__": 
    results = {}
    results["ObVi-SLAM"] = load_obvislam(kObViSLAMFilepath)
    results["Groundtruth"] = load_legoloam(kLeGOLOAMFilepath)
    results["ORBSLAM3"] = load_legoloam(kORBSLAM3Filepath)
    plot(results=results)
    # import pdb; pdb.set_trace()