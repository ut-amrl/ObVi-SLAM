import os
import subprocess
import signal
import argparse
import time
import rospy
import sys
import roslib
from trajectory_sequence import *
from file_structure_utils import *
from cmd_line_arg_utils import *

roslib.load_manifest('amrl_msgs')
from amrl_msgs.srv import *


class ORBSLAM3RelatedConstants:
    posesFileBaseName = "trajectory.csv"
    mapFileBaseName = "orbslam_map.osa"
    orbSlamSaveTrajServiceName = "/orb_slam/save_and_start_new_trajectory"


class ORBSLAM3LaunchStartInfo:
    def __init__(self, orb_slam_configuration_file, orb_slam_vocabulary_file):
        self.orb_slam_configuration_file = orb_slam_configuration_file
        self.orb_slam_vocabulary_file = orb_slam_vocabulary_file


class ORBSLAM3SessionConfig:
    def __init__(self, top_level_orb_slam_results_dir, rosbagDirectory, bagIdxInSequence, rosbagBaseName,
                 generate_map_file=False):
        self.top_level_orb_slam_results_dir = top_level_orb_slam_results_dir
        self.rosbagDirectory = rosbagDirectory
        self.bagIdxInSequence = bagIdxInSequence
        self.rosbagBaseName = rosbagBaseName
        self.generate_map_file = generate_map_file


class ORBSLAM3SequenceConfig:
    def __init__(self, sequence_dir, sequence_file_base_name, top_level_orb_slam_results_dir, rosbagDirectory,
                 generate_map_file=False, force_regenerate_results=False):
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.top_level_orb_slam_results_dir = top_level_orb_slam_results_dir
        self.rosbagDirectory = rosbagDirectory
        self.generate_map_file = generate_map_file
        self.force_regenerate_results = force_regenerate_results


def generateResultsFileNamesForBagInSeq(bagNumInSeq, bagName, top_level_orb_slam_results_dir):
    bagSpecificDir = str(bagNumInSeq) + "_" + bagName + "/"
    sessionRootDir = FileStructureUtils.ensureDirectoryEndsWithSlash(
        top_level_orb_slam_results_dir) + "/" + bagSpecificDir
    trajectoryFileName = sessionRootDir + ORBSLAM3RelatedConstants.posesFileBaseName
    mapFileName = sessionRootDir + ORBSLAM3RelatedConstants.mapFileBaseName

    return sessionRootDir, trajectoryFileName, mapFileName


def checkForExistingResults(top_level_orb_slam_results_dir, generate_map_file, rosbagsSequence):
    expectedFiles = []
    for idx, bagName in enumerate(rosbagsSequence):
        sessionRootDir, trajectoryFileName, mapFileName = generateResultsFileNamesForBagInSeq(
            idx, bagName, top_level_orb_slam_results_dir)
        expectedFiles.append(trajectoryFileName)
        if (generate_map_file):
            expectedFiles.append(mapFileName)

    for expectedFile in expectedFiles:
        if (not os.path.exists(expectedFile)):
            return False
    return True


def startORBSLAM3(launchInfo):
    useSimTimeBefore = rospy.get_param('/use_sim_time', False)
    if (not useSimTimeBefore):
        rospy.set_param('/use_sim_time', True)

    orbSLAMRunCmdArgs = []
    orbSLAMRunCmdArgs.append('rosrun')
    orbSLAMRunCmdArgs.append('ORB_SLAM3')
    orbSLAMRunCmdArgs.append('Stereo')
    vocFile = launchInfo.orb_slam_vocabulary_file
    orbSLAMRunCmdArgs.append(vocFile)
    orbSLAMRunCmdArgs.append(launchInfo.orb_slam_configuration_file)
    orbSLAMRunCmdArgs.append('false')

    processReturn = subprocess.Popen(orbSLAMRunCmdArgs, preexec_fn=os.setsid)

    # Sleep while orbslam starts up
    # time.sleep(15) # don't think we need this since we have a service wait

    return (processReturn, useSimTimeBefore)


def cleanupORBSLAM3(processReturn, useSimTimeBefore):
    # Sleep to make sure the files were saved
    if (not useSimTimeBefore):
        rospy.set_param('/use_sim_time', useSimTimeBefore)

    os.killpg(os.getpgid(processReturn.pid), signal.SIGTERM)


def runOrbSLAMSingleSeqAfterSetup(orbSlamSessionConfig, saveTrajService):
    fullBagName = FileStructureUtils.ensureDirectoryEndsWithSlash(orbSlamSessionConfig.rosbagDirectory) + \
                  orbSlamSessionConfig.rosbagBaseName + FileStructureConstants.bagSuffix

    bagPlayCmd = "rosbag play --clock -r 0.5 " + fullBagName
    os.system(bagPlayCmd)
    time.sleep(2)

    # Send save trajectory call
    sessionRootDir, trajectoryFileName, mapFileName = generateResultsFileNamesForBagInSeq(
        orbSlamSessionConfig.bagIdxInSequence, orbSlamSessionConfig.rosbagBaseName,
        orbSlamSessionConfig.top_level_orb_slam_results_dir)
    FileStructureUtils.makeDirectory(sessionRootDir)

    if (not orbSlamSessionConfig.generate_map_file):
        mapFileName = ""

    resp = saveTrajService(trajectory_file_name=trajectoryFileName, map_file_name=mapFileName)


def runOrbSLAM3Sequence(orbSlamLaunchConfig, orbSlamSequenceConfig):
    rosbagsSequence = readTrajectorySequence(orbSlamSequenceConfig.sequence_dir,
                                             orbSlamSequenceConfig.sequence_file_base_name)

    topLevelResultsDirForSeq = FileStructureUtils.ensureDirectoryEndsWithSlash(
        orbSlamSequenceConfig.top_level_orb_slam_results_dir) + orbSlamSequenceConfig.sequence_file_base_name + "/"

    if (not orbSlamSequenceConfig.force_regenerate_results):
        if (checkForExistingResults(topLevelResultsDirForSeq,
                                    orbSlamSequenceConfig.generate_map_file, rosbagsSequence)):
            print("Results already exist for the full sequence. Not regenerating")
            return

    processReturn, beforeStartUseSimTime = startORBSLAM3(orbSlamLaunchConfig)

    rospy.init_node('orb_slam_3_session_executor')
    rospy.wait_for_service(ORBSLAM3RelatedConstants.orbSlamSaveTrajServiceName, 60)
    orbSaveService = rospy.ServiceProxy(ORBSLAM3RelatedConstants.orbSlamSaveTrajServiceName,
                                        OrbSLAMSaveFilesAndStartNewTrajectorySrv)

    for idx, bagName in enumerate(rosbagsSequence):
        sessionConfig = ORBSLAM3SessionConfig(
            top_level_orb_slam_results_dir=topLevelResultsDirForSeq,
            rosbagDirectory=orbSlamSequenceConfig.rosbagDirectory,
            bagIdxInSequence=idx,
            rosbagBaseName=bagName,
            generate_map_file=orbSlamSequenceConfig.generate_map_file)
        runOrbSLAMSingleSeqAfterSetup(sessionConfig, orbSaveService)

    cleanupORBSLAM3(processReturn, beforeStartUseSimTime)


def orbSlam3SequenceArgParse():
    parser = argparse.ArgumentParser(description="Run ORB-SLAM3 for full trajectory sequence")
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlamConfigurationFileBaseArgName),
        required=True,
        help=CmdLineArgConstants.orbSlamConfigurationFileHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlamVocabularyFileBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.orbSlamVocabularyFileHelp)

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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlam3OutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.orbSlam3OutRootDirHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRunORBSLAMBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRunORBSLAMHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRunORBSLAMBaseArgName,
                        dest=CmdLineArgConstants.forceRunORBSLAM3BaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRunORBSLAMBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.generateMapFileBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.generateMapFileHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.generateMapFileBaseArgName,
                        dest=CmdLineArgConstants.generateMapFileBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.generateMapFileBaseArgName)
    args_dict = vars(parser.parse_args())

    launchConfig = ORBSLAM3LaunchStartInfo(
        orb_slam_configuration_file=args_dict[CmdLineArgConstants.orbSlam3ConfigurationFileBaseArgName],
        orb_slam_vocabulary_file=args_dict[CmdLineArgConstants.orbSlam3VocabularyFileBaseArgName])

    sequenceConfig = ORBSLAM3SequenceConfig(
        sequence_dir=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        sequence_file_base_name=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        top_level_orb_slam_results_dir=args_dict[CmdLineArgConstants.orbSlam3OutRootDirBaseArgName],
        rosbagDirectory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        generate_map_file=args_dict[CmdLineArgConstants.generateMapFileBaseArgName],
        force_regenerate_results=args_dict[CmdLineArgConstants.forceRunORBSLAM3BaseArgName])

    return launchConfig, sequenceConfig


if __name__ == "__main__":
    launchConfig, sequenceConfig = orbSlam3SequenceArgParse()
    runOrbSLAM3Sequence(orbSlamLaunchConfig=launchConfig, orbSlamSequenceConfig=sequenceConfig)
