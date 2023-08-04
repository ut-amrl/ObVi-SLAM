from cmd_line_arg_utils import *
from file_structure_utils import *
import os


class InterpolatorParamConstants:
    required_timestamps_file = "required_timestamps_file"
    coarse_trajectory_file = "coarse_trajectory_file"
    rosbag_file = "rosbag_file"
    poses_for_required_timestamps_file = "poses_for_required_timestamps_file"
    coarse_trajectory_frame_rel_bl_file = "coarse_trajectory_frame_rel_bl_file"
    odom_frame_rel_bl_file = "odom_frame_rel_bl_file"
    odometry_topic = "odometry_topic"
    param_prefix = "param_prefix"


class CalibrationFileConstants:
    legoLoamCalibFile = "lego_loam_bl.txt"
    orbslam3CalibFile = "orb_slam3_bl.txt"
    ovslamCalibFile = "ov_slam_bl.txt"
    odomCalibFile = "odom_bl.txt"
    identityTransformFile = "identity_transform.txt"


class InterpolatorConfig:
    def __init__(self, required_timestamps_file, coarse_trajectory_file, rosbag_file,
                 poses_for_required_timestamps_file, coarse_trajectory_frame_rel_bl_file, odom_frame_rel_bl_file,
                 odometry_topic, param_prefix):
        self.required_timestamps_file = required_timestamps_file
        self.coarse_trajectory_file = coarse_trajectory_file
        self.rosbag_file = rosbag_file
        self.poses_for_required_timestamps_file = poses_for_required_timestamps_file
        self.coarse_trajectory_frame_rel_bl_file = coarse_trajectory_frame_rel_bl_file
        self.odom_frame_rel_bl_file = odom_frame_rel_bl_file
        self.odometry_topic = odometry_topic
        self.param_prefix = param_prefix


def runInterpolatorCmd(interpolatorConfig):
    argsString = ""
    argsString += createCommandStrAddition(InterpolatorParamConstants.required_timestamps_file,
                                           interpolatorConfig.required_timestamps_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.coarse_trajectory_file,
                                           interpolatorConfig.coarse_trajectory_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.rosbag_file, interpolatorConfig.rosbag_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.poses_for_required_timestamps_file,
                                           interpolatorConfig.poses_for_required_timestamps_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.coarse_trajectory_frame_rel_bl_file,
                                           interpolatorConfig.coarse_trajectory_frame_rel_bl_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.odom_frame_rel_bl_file,
                                           interpolatorConfig.odom_frame_rel_bl_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.odometry_topic,
                                           interpolatorConfig.odometry_topic)
    argsString += createCommandStrAddition(InterpolatorParamConstants.param_prefix,
                                           interpolatorConfig.param_prefix)

    cmdToRun = "./bin/interpolate_poses_with_required_nodes " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def runInterpolator(rosbag_dir, rosbag_name, interpolation_traj_dir, lego_loam_root_dir, required_timestamps_file_dir,
                    coarse_trajectory_frame_rel_bl_file, odom_frame_rel_bl_file, odometry_topic, forceRerunInterpolator,
                    overrideRequiredStampsFile=None, paramPrefix=None):
    poses_for_required_timestamps_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        interpolation_traj_dir) + "interpolated_lego_loam_poses.csv"
    needToRerunInterpolator = False
    if (forceRerunInterpolator):
        needToRerunInterpolator = True
    elif (not os.path.exists(poses_for_required_timestamps_file)):
        needToRerunInterpolator = True
    if (not needToRerunInterpolator):
        print("Interpolated trajectory already exists so skipping generation")
        return poses_for_required_timestamps_file
    if (overrideRequiredStampsFile is not None):
        required_timestamps_file = overrideRequiredStampsFile
    else:
        required_timestamps_file = (FileStructureUtils.ensureDirectoryEndsWithSlash(
            required_timestamps_file_dir) + FileStructureConstants.finalTrajectoryFileBaseName)

    coarse_trajectory_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        lego_loam_root_dir) + rosbag_name + "/poses/lego_loam_poses.csv"
    if (not os.path.exists(coarse_trajectory_file)):
        print("No ground truth trajectory generated for rosbag " + rosbag_name + "; skipping interpolation")
        return None

    interpolatorConfig = InterpolatorConfig(
        required_timestamps_file=required_timestamps_file,
        coarse_trajectory_file=coarse_trajectory_file,
        rosbag_file=(FileStructureUtils.ensureDirectoryEndsWithSlash(rosbag_dir) + \
                     rosbag_name + FileStructureConstants.bagSuffix),
        poses_for_required_timestamps_file=poses_for_required_timestamps_file,
        coarse_trajectory_frame_rel_bl_file=coarse_trajectory_frame_rel_bl_file,
        odom_frame_rel_bl_file=odom_frame_rel_bl_file,
        odometry_topic=odometry_topic,
        param_prefix=paramPrefix)
    runInterpolatorCmd(interpolatorConfig)
    return poses_for_required_timestamps_file
