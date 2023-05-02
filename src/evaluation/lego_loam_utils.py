import os
import subprocess
import signal
import time
from trajectory_sequence import *
from file_structure_utils import *
from cmd_line_arg_utils import *


class LegoLoamRelatedConstants:
    legoLoamPosesDirName = "poses/"
    legoLoamPointCloudsDirName = "point_clouds/"
    legoLoamPosesFileName = "lego_loam_poses.csv"


class LeGOLOAMExecutionInfo:
    def __init__(self, lego_loam_out_root_dir, rosbag_file_directory, rosbag_base_name,
                 force_run_lego_loam):
        self.lego_loam_out_root_dir = lego_loam_out_root_dir
        self.rosbag_file_directory = rosbag_file_directory
        self.rosbag_base_name = rosbag_base_name
        self.force_run_lego_loam = force_run_lego_loam


class LeGOLOAMExecutionInfoForSequence:
    def __init__(self, lego_loam_out_root_dir, trajectory_sequence_file_directory,
                 sequence_file_base_name, rosbag_file_directory, force_run_lego_loam):
        self.lego_loam_out_root_dir = lego_loam_out_root_dir
        self.trajectory_sequence_file_directory = trajectory_sequence_file_directory
        self.sequence_file_base_name = sequence_file_base_name
        self.rosbag_file_directory = rosbag_file_directory
        self.force_run_lego_loam = force_run_lego_loam


def generateLegoLoamPosesFile(lego_loam_out_root_dir, rosbag_base_name):
    outputDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(lego_loam_out_root_dir) + \
                      rosbag_base_name + "/"
    outputFileForPoses = outputDirForBag + LegoLoamRelatedConstants.legoLoamPosesDirName + \
                         LegoLoamRelatedConstants.legoLoamPosesFileName
    return outputFileForPoses


def runLegoLoamSingleTrajectory(legoLoamExecutionInfo, bagPlayer):
    outputDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(
        legoLoamExecutionInfo.lego_loam_out_root_dir) + legoLoamExecutionInfo.rosbag_base_name + "/"

    needToRerun = False
    if (legoLoamExecutionInfo.force_run_lego_loam):
        needToRerun = True
    else:
        outputFileForPoses = generateLegoLoamPosesFile(
            legoLoamExecutionInfo.lego_loam_out_root_dir, legoLoamExecutionInfo.rosbag_base_name)
        if (not os.path.exists(outputFileForPoses)):
            needToRerun = True

    if (needToRerun):
        FileStructureUtils.makeDirectory(outputDirForBag + LegoLoamRelatedConstants.legoLoamPosesDirName)
        FileStructureUtils.makeDirectory(outputDirForBag + LegoLoamRelatedConstants.legoLoamPointCloudsDirName)

        # Start lego-loam -- need to capture PID to kill
        legoLoamCmdArgs = []
        legoLoamCmdArgs.append("roslaunch")
        legoLoamCmdArgs.append("lego_loam")
        legoLoamCmdArgs.append("run.launch")
        legoLoamCmdArgs.append(("results_dir:=" + outputDirForBag))
        legoLoamProcessReturn = subprocess.Popen(legoLoamCmdArgs, preexec_fn=os.setsid)

        time.sleep(3)
        bagPlayer(legoLoamExecutionInfo)
        time.sleep(3)

        # Kill lego loam
        os.killpg(os.getpgid(legoLoamProcessReturn.pid), signal.SIGTERM)


def runLegoLoamSequence(legoLoamExecutionInfoForSequence, singleTrajectoryExecutionCreator, bagPlayer):
    rosbagsSequence = readTrajectorySequence(legoLoamExecutionInfoForSequence.trajectory_sequence_file_directory,
                                             legoLoamExecutionInfoForSequence.sequence_file_base_name)

    for bagName in rosbagsSequence:
        singleTrajExecutionInfo = singleTrajectoryExecutionCreator(bagName, legoLoamExecutionInfoForSequence)
        runLegoLoamSingleTrajectory(singleTrajExecutionInfo, bagPlayer)
