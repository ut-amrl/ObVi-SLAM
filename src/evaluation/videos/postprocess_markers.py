import rosbag
import numpy as np
import copy
from geometry_msgs.msg import Point

kBagname = "0__2023_05_11_18_35_54"
kPrefix = "/evaluation_2023_07_v1_update_rev_lowerererish_conv_thresholds_manual_feat_adj_tighterer_vo_v4_" + kBagname + "/"

kBagpathIn  = "/home/tiejean/Documents/mnt/oslam/visualization/0__2023_05_11_18_35_54.bag"
kBagpathOut = "/home/tiejean/Documents/mnt/oslam/visualization/0__2023_05_11_18_35_54_postprocessed2.bag"

kLeGOLOAMFilepath = "/home/tiejean/Documents/mnt/oslam/ut_vslam_results/postprocessing/interpolated_lego_loam_poses_bag_0_viz.cvs"

def load_lego_loam(filepath: str):
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    # stamped_poses = {}
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        sec, nsec = int(tokens[0]), int(tokens[1])
        pose = [float(token) for token in tokens[2:]]
        poses.append(pose)
        # stamped_poses[(sec, nsec)] = pose
    fp.close()
    # return stamped_poses
    return poses


if __name__ == "__main__":
    interesting_topics = ["/est_pose", "/est_ellipsoids", "/est_/latest/cam_1_bb/image_raw", "/tf"]
    interesting_topics = [kPrefix + topic for topic in interesting_topics]

    # stamped_ref_poses = load_lego_loam(kLeGOLOAMFilepath)
    ref_poses = load_lego_loam(kLeGOLOAMFilepath)
    bagIn = rosbag.Bag(kBagpathIn, 'r')
    bagOut = rosbag.Bag(kBagpathOut, 'w')

    all_gt_dtimes = []
    all_gt_msgs = []
    for topic, msg, t in bagIn.read_messages(topics=(kPrefix + "/gt_pose")):
        if msg.type != 4:
            continue
        sec, nsec = msg.header.stamp.secs, msg.header.stamp.nsecs
        all_gt_dtimes.append(sec + nsec * 1e-9)
        all_gt_msgs.append(msg)

    for topic, msg, t in bagIn.read_messages(topics=[kPrefix + "/gt_pose", kPrefix + "/est_pose"]):
        if topic == (kPrefix + "/est_pose"):
            if msg.type != 4:
                continue
            sec, nsec = msg.header.stamp.secs, msg.header.stamp.nsecs
            dtime = sec + nsec * 1e-9

            msgEst = copy.deepcopy(msg)
            tdiffs_gt  = np.array(all_gt_dtimes)-dtime;  idx_gt  = np.argmin(np.abs(tdiffs_gt))
            msgGt = copy.deepcopy(all_gt_msgs[idx_gt])

            nEstPoses, nGtPoses = len(msgEst.points), len(msgGt.points)
            if (nGtPoses >= nEstPoses):
                # cap the number of poses to msgGt to be nEstPoses
                msgGt.points = msgGt.points[:nEstPoses] 
            else:
                for pose in ref_poses[nGtPoses:nEstPoses]:
                    # msgGt.points.append(Point(pose[0]+0.46618, pose[1], pose[2]))
                    msgGt.points.append(Point(pose[0], pose[1], pose[2]))

            bagOut.write(kPrefix + "/gt_pose", msgGt, t)
            bagOut.write(kPrefix + "/est_pose", msgEst, t)


    # for topic, msg, t in bagIn.read_messages():
    #     if topic == (kPrefix + "/gt_pose"):
    #         continue
    #     if topic == (kPrefix + "/est_pose"):
    #         import pdb; pdb.set_trace()
    #         msgEst = msg
    #         msgEst.color.r = 0
    #         msgEst.color.g = 1
    #         msgEst.color.b = 1
    #         msgEst.type = 4
    #         msgEst.scale.x *= 4
    #         msgEst.scale.y *= 8
    #         msgEst.scale.z *= 4
    #         bagOut.write(topic, msgEst, t)

    #         sec, nsec = msgEst.header.stamp.secs, msgEst.header.stamp.nsecs
    #         dtime = sec + nsec * 1e-9
    #         tdiffs = np.array(all_gt_dtimes)-dtime
    #         idx = np.argmin(np.abs(tdiffs)) # Reference GT message: all_gt_msgs[idx]

    #         msgGt = copy.deepcopy(all_gt_msgs[idx])
    #         msgGt.color.r = 0
    #         msgGt.color.g = 1
    #         msgGt.color.b = 1
    #         msgGt.type = 4
    #         msgGt.scale.x *= 8
    #         msgGt.scale.y *= 16
    #         msgGt.scale.z *= 8
    #         msgGt.color.r = 0
    #         msgGt.color.g = .5
    #         msgGt.color.b = 1
    #         bagOut.write(kPrefix + "/gt_pose", msgGt, t)

    #     else:
    #         bagOut.write(topic, msg, t)
    bagIn.close(); bagOut.close()
