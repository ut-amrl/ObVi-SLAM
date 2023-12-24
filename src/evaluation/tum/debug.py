import json
from scipy.spatial.transform import Rotation as R
import numpy as np
from collections import OrderedDict

kTimeFilepath = "/home/tiejean/Documents/mnt/oslam/orb_post_process/sparsified_ut_vslam_in/tum_fr2_desk_obj_fast/freiburg2_desk/timestamps/timestamps_only.txt"

kObViSLAMJsonFilepath = "/home/tiejean/Documents/mnt/oslam/ut_vslam_results/tum_fr2_desk/tum_fr2_desk_obj_fast/0_freiburg2_desk/ut_vslam_out/robot_pose_results.json"
kObViSLAMCsvFilepath = "/home/tiejean/Documents/mnt/oslam/ut_vslam_results/tum_fr2_desk/tum_fr2_desk_obj_fast/0_freiburg2_desk/postprocessing/trajectory.csv"

kGTCsvFilepath = "/home/tiejean/Documents/mnt/oslam/ut_vslam_results/tum_fr2_desk/tum_fr2_desk_obj_fast/0_freiburg2_desk/postprocessing/interpolated_lego_loam_poses.csv"

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

def json2cvs(filepath_time:str, filepath_json:str, filepath_csv: str):
    timestamps = load_timestamps(filepath_time)

    framed_poses = []
    fp_json = open(filepath_json, "r")
    if fp_json.closed:
        raise FileNotFoundError("Failed to open file " + filepath_json)
    data = json.load(fp_json)
    for framed_pose in data["robot_poses"]["robot_pose_results_map"]:
        fid = int(framed_pose["frame_id"])
        transl = np.array(framed_pose["pose"]["transl"]["Data"])
        rotvec = framed_pose["pose"]["rot"]["angle"] * np.array(framed_pose["pose"]["rot"]["axis"]["Data"])
        quat = R.from_rotvec(rotvec).as_quat()
        framed_poses.append((fid, transl, quat))
    framed_poses = sorted(framed_poses, key=lambda x: x[0])
    
    fp_csv = open(filepath_csv, "w")
    if fp_csv.closed:
        raise FileNotFoundError("Failed to open file " + filepath_csv)
    fp_csv.write("seconds, nanoseconds, lost, transl_x, transl_y, transl_z, quat_x, quat_y, quat_z, quat_w\n")
    for fid, transl, quat in framed_poses:
        fp_csv.write(str(timestamps[fid][0]) + ", " + str(timestamps[fid][1]))
        fp_csv.write(", 0") # hardcoding lost to be 0
        fp_csv.write(", " + str(transl[0]) + ", " + str(transl[1]) + ", " + str(transl[2]) \
                    + ", " + str(quat[0]) + ", " + str(quat[1]) + ", " + str(quat[2]) + ", " + str(quat[3]) )
        fp_csv.write("\n")
    fp_json.close(); fp_csv.close() 

def load_obvi_slam(filepath:str):
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    timestamps = []
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        sec  = int(tokens[0])
        nsec = int(tokens[1])
        timestamps.append((sec, nsec))
        pose = [float(token) for token in tokens[3:]]
        poses.append(pose)
    fp.close()
    return timestamps, poses

def load_lego_loam(filepath:str):
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    timestamps = []
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        sec  = int(tokens[0])
        nsec = int(tokens[1])
        timestamps.append((sec, nsec))
        pose = [float(token) for token in tokens[2:]]
        poses.append(pose)
    fp.close()
    return timestamps, poses

def pose2transform(pose):
    transform = np.eye(4)
    transform[:3,:3] = R.from_quat(pose[3:]).as_matrix()
    transform[:3,3] = pose[:3]
    return transform

def transform2pose(transform):
    transl = transform[:3,3]
    quat = R.from_matrix(transform[:3,:3]).as_quat()
    pose = []
    for x in transl:
        pose.append(x)
    for x in quat:
        pose.append(x)
    return pose

def applyRoffset(Rot, pose):
    # Rt = np.eye(4)
    # Rt[:3,:3] = Rot
    # transform = pose2transform(pose)
    # transform = transform @ Rt
    transl = np.array(pose[:3])
    rot = R.from_quat(np.array(pose[3:]))
    transl = Rot @ transl
    # quat = R.from_matrix(Rot @ quat.as_matrix()).as_quat()
    return np.concatenate([transl, rot.as_quat()]).tolist()


if __name__ == "__main__":
    Roffset = np.array([[ 0.,  0.,  1.],
                        [-1.,  0.,  0.],
                        [ 0., -1.,  0.]])
    # json2cvs(kTimeFilepath, kObViSLAMJsonFilepath, kObViSLAMCsvFilepath)
    timestamps_lego, poses_lego = load_lego_loam(kGTCsvFilepath)
    fp_lego = open(kGTCsvFilepath, "w")
    if fp_lego.closed:
        raise FileNotFoundError("Failed to open file " + kGTCsvFilepath)
    fp_lego.write("seconds, nanoseconds, transl_x, transl_y, transl_z, quat_x, quat_y, quat_z, quat_w\n")
    for timestamp, pose in zip(timestamps_lego, poses_lego):
        new_pose = applyRoffset(Roffset, pose)
        transl, quat = new_pose[:3], new_pose[3:]
        fp_lego.write(str(timestamp[0]) + ", " + str(timestamp[1]))
        fp_lego.write(", " + str(transl[0]) + ", " + str(transl[1]) + ", " + str(transl[2]) \
                    + ", " + str(quat[0]) + ", " + str(quat[1]) + ", " + str(quat[2]) + ", " + str(quat[3]) )
        fp_lego.write("\n")
    fp_lego.close()
    # poses_lego = load_lego_loam(kGTCsvFilepath)
    # for pose_obvi, pose_lego in zip(poses_obvi, poses_lego):
    #     pose_obvi = poses_obvi[50]
    #     pose_lego = poses_lego[50]
    #     print("pose_lego: ", pose_lego)
    #     pose_lego = applyRoffset(Roffset, pose_lego)
    #     print("pose_lego: ", pose_lego)
    #     print("pose_obvi: ", pose_obvi)
    #     exit()