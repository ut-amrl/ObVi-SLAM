import os
import subprocess
import signal
import argparse
import time
import sys
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


def runLegoLoamSingleTrajectory(legoLoamExecutionInfo):
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

        bagNameWithExt = legoLoamExecutionInfo.rosbag_base_name + FileStructureConstants.bagSuffix

        bagFileLocation = FileStructureUtils.ensureDirectoryEndsWithSlash(
            legoLoamExecutionInfo.rosbag_file_directory) + bagNameWithExt
        playbackCmd = "rosbag play --clock -r 0.2 " + bagFileLocation
        print("Running command to play rosbag")
        print(playbackCmd)
        os.system(playbackCmd)
        time.sleep(3)

        # Kill lego loam
        os.killpg(os.getpgid(legoLoamProcessReturn.pid), signal.SIGTERM)


def runLegoLoamSequence(legoLoamExecutionInfoForSequence):
    rosbagsSequence = readTrajectorySequence(legoLoamExecutionInfoForSequence.trajectory_sequence_file_directory,
                                             legoLoamExecutionInfoForSequence.sequence_file_base_name)

    for bagName in rosbagsSequence:
        singleTrajeExecutionInfo = LeGOLOAMExecutionInfo(
            lego_loam_out_root_dir=legoLoamExecutionInfoForSequence.lego_loam_out_root_dir,
            rosbag_base_name=bagName,
            rosbag_file_directory=legoLoamSequenceConfig.rosbag_file_directory,
            force_run_lego_loam=legoLoamExecutionInfoForSequence.force_run_lego_loam)
        runLegoLoamSingleTrajectory(singleTrajeExecutionInfo)


def legoLoamTrajectoryArgParse():
    parser = argparse.ArgumentParser(description="Run lego loam for full trajectory sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagBaseNameBaseArgName),
        required=True,
        help=CmdLineArgConstants.rosbagBaseNameHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRunLegoLoamBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRunLegoLoamHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRunLegoLoamBaseArgName,
                        dest=CmdLineArgConstants.forceRunLegoLoamBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRunLegoLoamBaseArgName)
    args_dict = vars(parser.parse_args())
    return LeGOLOAMExecutionInfo(
        lego_loam_out_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
        rosbag_base_name=args_dict[CmdLineArgConstants.rosbagBaseNameBaseArgName],
        rosbag_file_directory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        force_run_lego_loam=args_dict[CmdLineArgConstants.forceRunLegoLoamBaseArgName])


def legoLoamSequenceArgParse():
    parser = argparse.ArgumentParser(description="Run lego loam for full trajectory sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRunLegoLoamBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRunLegoLoamHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRunLegoLoamBaseArgName,
                        dest=CmdLineArgConstants.forceRunLegoLoamBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRunLegoLoamBaseArgName)
    args_dict = vars(parser.parse_args())
    return LeGOLOAMExecutionInfoForSequence(
        lego_loam_out_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
        trajectory_sequence_file_directory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        sequence_file_base_name=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        rosbag_file_directory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        force_run_lego_loam=args_dict[CmdLineArgConstants.forceRunLegoLoamBaseArgName])


if __name__ == "__main__":

    if (("--" + CmdLineArgConstants.rosbagBaseNameBaseArgName) in sys.argv):
        legoLoamSingleBagConfig = legoLoamTrajectoryArgParse()
        runLegoLoamSingleTrajectory(legoLoamSingleBagConfig)
    else:
        legoLoamSequenceConfig = legoLoamSequenceArgParse()
        runLegoLoamSequence(legoLoamSequenceConfig)
