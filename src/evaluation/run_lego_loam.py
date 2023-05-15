import os
import argparse
import sys
from trajectory_sequence import *
from file_structure_utils import *
from cmd_line_arg_utils import *
from lego_loam_utils import *


class OusterLeGOLOAMExecutionInfo(LeGOLOAMExecutionInfo):
    def __init__(self, lego_loam_out_root_dir, coda_parser_repo_root_dir, rosbag_file_directory, rosbag_base_name,
                 force_run_lego_loam):
        LeGOLOAMExecutionInfo.__init__(self,
                                       lego_loam_out_root_dir=lego_loam_out_root_dir,
                                       rosbag_file_directory=rosbag_file_directory,
                                       rosbag_base_name=rosbag_base_name,
                                       force_run_lego_loam=force_run_lego_loam)
        self.coda_parser_repo_root_dir = coda_parser_repo_root_dir


class OusterLeGOLOAMExecutionInfoForSequence(LeGOLOAMExecutionInfoForSequence):
    def __init__(self, lego_loam_out_root_dir, coda_parser_repo_root_dir, trajectory_sequence_file_directory,
                 sequence_file_base_name, rosbag_file_directory, force_run_lego_loam):
        LeGOLOAMExecutionInfoForSequence.__init__(self, lego_loam_out_root_dir=lego_loam_out_root_dir,
                                                  trajectory_sequence_file_directory=trajectory_sequence_file_directory,
                                                  sequence_file_base_name=sequence_file_base_name,
                                                  rosbag_file_directory=rosbag_file_directory,
                                                  force_run_lego_loam=force_run_lego_loam)
        self.coda_parser_repo_root_dir = coda_parser_repo_root_dir


class CodaOusterParserScriptConstants:
    codaScriptRelativePath = "scripts/decode_multiday.py "
    codaConfigRelativePath = "config/bagdecoder_lidarimuonly.yaml "
    codaConfigArgName = "--config "
    codaBagNameArgName = "--bag_name "
    codaBagDirArgName = "--bag_dir "


def runBagWithCoda(legoLoamExecutionInfo):
    codaRepoDir = FileStructureUtils.ensureDirectoryEndsWithSlash(legoLoamExecutionInfo.coda_parser_repo_root_dir)
    os.chdir(codaRepoDir)
    codaScriptLocation = codaRepoDir + CodaOusterParserScriptConstants.codaScriptRelativePath
    codaConfigLocation = codaRepoDir + CodaOusterParserScriptConstants.codaConfigRelativePath
    bagNameWithExt = legoLoamExecutionInfo.rosbag_base_name + FileStructureConstants.bagSuffix

    codaCmd = "python3 " + codaScriptLocation + CodaOusterParserScriptConstants.codaConfigArgName + \
              codaConfigLocation + CodaOusterParserScriptConstants.codaBagNameArgName + bagNameWithExt + " " + \
              CodaOusterParserScriptConstants.codaBagDirArgName + legoLoamExecutionInfo.rosbag_file_directory
    print("Running command to parse ouster packets")
    print(codaCmd)
    os.system(codaCmd)


def createOusterSingleConfigFromSequence(bagName, legoLoamExecutionInfoForSequence):
    return OusterLeGOLOAMExecutionInfo(
        lego_loam_out_root_dir=legoLoamExecutionInfoForSequence.lego_loam_out_root_dir,
        coda_parser_repo_root_dir=legoLoamExecutionInfoForSequence.coda_parser_repo_root_dir,
        rosbag_base_name=bagName,
        rosbag_file_directory=legoLoamExecutionInfoForSequence.rosbag_file_directory,
        force_run_lego_loam=legoLoamExecutionInfoForSequence.force_run_lego_loam)


def legoLoamTrajectoryArgParse():
    parser = argparse.ArgumentParser(description="Run lego loam for full trajectory sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.codaParserRepoRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.codaParserRepoRootDirHelp)
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
    return OusterLeGOLOAMExecutionInfo(
        lego_loam_out_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
        coda_parser_repo_root_dir=args_dict[CmdLineArgConstants.codaParserRepoRootDirBaseArgName],
        rosbag_base_name=args_dict[CmdLineArgConstants.rosbagBaseNameBaseArgName],
        rosbag_file_directory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        force_run_lego_loam=args_dict[CmdLineArgConstants.forceRunLegoLoamBaseArgName])


def legoLoamSequenceArgParse():
    parser = argparse.ArgumentParser(description="Run lego loam for full trajectory sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.codaParserRepoRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.codaParserRepoRootDirHelp)
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
    return OusterLeGOLOAMExecutionInfoForSequence(
        lego_loam_out_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
        coda_parser_repo_root_dir=args_dict[CmdLineArgConstants.codaParserRepoRootDirBaseArgName],
        trajectory_sequence_file_directory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        sequence_file_base_name=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        rosbag_file_directory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        force_run_lego_loam=args_dict[CmdLineArgConstants.forceRunLegoLoamBaseArgName])


if __name__ == "__main__":

    if (("--" + CmdLineArgConstants.rosbagBaseNameBaseArgName) in sys.argv):
        legoLoamSingleBagConfig = legoLoamTrajectoryArgParse()
        runLegoLoamSingleTrajectory(legoLoamSingleBagConfig, runBagWithCoda)
    else:
        legoLoamSequenceConfig = legoLoamSequenceArgParse()
        runLegoLoamSequence(legoLoamSequenceConfig, createOusterSingleConfigFromSequence, runBagWithCoda)
