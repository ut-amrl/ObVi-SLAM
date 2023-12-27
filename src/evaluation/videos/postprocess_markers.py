import rosbag
import numpy as np
import copy
from geometry_msgs.msg import Point

kBagname = "0__2023_05_11_18_35_54"
kPrefix = "/evaluation_2023_07_v1_update_rev_lowerererish_conv_thresholds_manual_feat_adj_tighterer_vo_v4_" + kBagname + "/"
kBagpathIn  = "/home/tiejean/Documents/mnt/oslam/visualization/" + kBagname + ".bag"
kBagpathOut = "/home/tiejean/Documents/mnt/oslam/visualization/" + kBagname + "_postprocessed2.bag"
kLeGOLOAMFilepath = "/home/tiejean/Documents/mnt/oslam/ut_vslam_results/postprocessing/interpolated_lego_loam_poses_bag_0_viz.cvs"

def load_lego_loam(filepath: str):
    fp = open(filepath, "r")
    if fp.closed:
        raise FileNotFoundError("Failed to open file " + filepath)
    poses = []
    for line in fp.readlines()[1:]:
        tokens = [token.strip() for token in line.split(",")]
        sec, nsec = int(tokens[0]), int(tokens[1])
        pose = [float(token) for token in tokens[2:]]
        poses.append(pose)
    fp.close()
    return poses


if __name__ == "__main__":
    ref_poses = load_lego_loam(kLeGOLOAMFilepath)
    bagIn = rosbag.Bag(kBagpathIn, 'r')
    bagOut = rosbag.Bag(kBagpathOut, 'w')

    for topic, msg, t in bagIn.read_messages():
        if topic == (kPrefix + "/gt_pose"):
            continue
        elif topic == (kPrefix + "/est_pose"):
            if msg.type != 4:
                continue
            sec, nsec = msg.header.stamp.secs, msg.header.stamp.nsecs
            dtime = sec + nsec * 1e-9

            msgEst, msgGt = copy.deepcopy(msg), copy.deepcopy(msg)

            nEstPoses, nGtPoses = len(msgEst.points), len(msgGt.points)
            msgGt.points = []
            for pose in ref_poses[0:nEstPoses]:
                msgGt.points.append(Point(pose[0], pose[1], pose[2]))

            msgEst.color.r = 0
            msgEst.color.g = 1
            msgEst.color.b = 1
            msgEst.scale.x *= 8
            msgEst.scale.y *= 1
            msgEst.scale.z *= 8

            msgGt.color.r = 0
            msgGt.color.g = .5
            msgGt.color.b = 1
            msgGt.scale.x *= 8
            msgGt.scale.y *= 1
            msgGt.scale.z *= 8

            bagOut.write(kPrefix + "/gt_pose", msgGt, t)
            bagOut.write(kPrefix + "/est_pose", msgEst, t)
        else:
            bagOut.write(topic, msg, t)


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
