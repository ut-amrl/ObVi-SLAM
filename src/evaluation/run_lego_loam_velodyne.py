import os
import argparse
import sys
from trajectory_sequence import *
from file_structure_utils import *
from cmd_line_arg_utils import *
from lego_loam_utils import *


def rosbagPlay(legoLoamExecutionInfo):
    bagNameWithExt = legoLoamExecutionInfo.rosbag_base_name + FileStructureConstants.bagSuffix

    bagFileLocation = FileStructureUtils.ensureDirectoryEndsWithSlash(
        legoLoamExecutionInfo.rosbag_file_directory) + bagNameWithExt
    playbackCmd = "rosbag play --clock -r 0.2 " + bagFileLocation
    print("Running command to play rosbag")
    print(playbackCmd)
    os.system(playbackCmd)


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


def createSingleTrajectoryInfoFromSeq(legoLoamExecutionInfoForSequence, bagName):
    return LeGOLOAMExecutionInfo(
        lego_loam_out_root_dir=legoLoamExecutionInfoForSequence.lego_loam_out_root_dir,
        rosbag_base_name=bagName,
        rosbag_file_directory=legoLoamExecutionInfoForSequence.rosbag_file_directory,
        force_run_lego_loam=legoLoamExecutionInfoForSequence.force_run_lego_loam)


if __name__ == "__main__":

    if (("--" + CmdLineArgConstants.rosbagBaseNameBaseArgName) in sys.argv):
        legoLoamSingleBagConfig = legoLoamTrajectoryArgParse()
        runLegoLoamSingleTrajectory(legoLoamSingleBagConfig, rosbagPlay)
    else:
        legoLoamSequenceConfig = legoLoamSequenceArgParse()
        runLegoLoamSequence(legoLoamSequenceConfig, createSingleTrajectoryInfoFromSeq, rosbagPlay)
