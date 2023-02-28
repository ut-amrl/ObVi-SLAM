import os
import subprocess
import signal
import argparse
import time
import rospy

from trajectory_sequence import *
from file_structure_utils import *
from cmd_line_arg_utils import *


class OrbSlam2RelatedConstants:
    timestampsFileName = "timestamps/node_ids_and_timestamps.txt"
    velocitiesDir = "velocities"
    minOrbFilesForLegitimateResults = 20


class ORBSLAM2SingleRunConfig:
    def __init__(self, orb_out_root_dir, rosbag_base_name, orb_slam_vocabulary_file, orb_slam_configuration_file,
                 rosbagDirectory, force_run_orb_slam_2=False):
        self.orb_out_root_dir = orb_out_root_dir
        self.rosbag_base_name = rosbag_base_name
        self.orb_slam_vocabulary_file = orb_slam_vocabulary_file
        self.orb_slam_configuration_file = orb_slam_configuration_file
        self.rosbagDirectory = rosbagDirectory
        self.force_run_orb_slam_2 = force_run_orb_slam_2


class ORBSLAM2SequenceConfig:
    def __init__(self, orb_out_root_dir, sequence_dir, sequence_file_base_name, orb_slam_vocabulary_file,
                 orb_slam_configuration_file, rosbagDirectory, force_run_orb_slam_2=False):
        self.orb_out_root_dir = orb_out_root_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.orb_slam_vocabulary_file = orb_slam_vocabulary_file
        self.orb_slam_configuration_file = orb_slam_configuration_file
        self.rosbagDirectory = rosbagDirectory
        self.force_run_orb_slam_2 = force_run_orb_slam_2


def runOrbSlam2SingleTrajectory(orbSlam2ExecutionInfo):
    outputDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(
        orbSlam2ExecutionInfo.orb_out_root_dir) + orbSlam2ExecutionInfo.rosbag_base_name + "/"

    needToRerun = False
    if (orbSlam2ExecutionInfo.force_run_orb_slam_2):
        needToRerun = True
    else:
        if (not os.path.exists(outputDirForBag)):
            needToRerun = True
        elif (not os.path.exists(outputDirForBag + OrbSlam2RelatedConstants.timestampsFileName)):
            needToRerun = True
        elif (len(os.listdir(outputDirForBag)) < OrbSlam2RelatedConstants.minOrbFilesForLegitimateResults):
            needToRerun = True

    if (needToRerun):
        os.system("rm -r " + outputDirForBag)
        FileStructureUtils.makeDirectory(outputDirForBag)

        # Start orbslam 2 -- need to capture PID to kill
        orbSLAMRunCmdArgs = []
        orbSLAMRunCmdArgs.append('rosrun')
        orbSLAMRunCmdArgs.append('ORB_SLAM2')
        orbSLAMRunCmdArgs.append('Stereo')
        orbSLAMRunCmdArgs.append(orbSlam2ExecutionInfo.orb_slam_vocabulary_file)
        orbSLAMRunCmdArgs.append(orbSlam2ExecutionInfo.orb_slam_configuration_file)
        orbSLAMRunCmdArgs.append('false')
        orbSLAMRunCmdArgs.append(outputDirForBag)
        orbSlamProcessReturn = subprocess.Popen(orbSLAMRunCmdArgs, preexec_fn=os.setsid)

        time.sleep(10)

        # Start parser
        fullBagName = FileStructureUtils.ensureDirectoryEndsWithSlash(orbSlam2ExecutionInfo.rosbagDirectory) + \
                      orbSlam2ExecutionInfo.rosbag_base_name + FileStructureConstants.bagSuffix

        bagPlayCmd = "rosbag play --clock -r 0.3 " + fullBagName
        os.system(bagPlayCmd)
        time.sleep(3)

        # Kill lego loam
        os.killpg(os.getpgid(orbSlamProcessReturn.pid), signal.SIGTERM)
    else:
        print("Skipping orb-slam for bag because results already exist and force rerun was off")


def runOrbSlam2Sequence(orbSlamExecutionInfoForSequence):
    rospy.init_node('orb_slam_2_runner')
    useSimTimeBefore = rospy.get_param('/use_sim_time', False)
    if (not useSimTimeBefore):
        rospy.set_param('/use_sim_time', True)
    rosbagsSequence = readTrajectorySequence(orbSlamExecutionInfoForSequence.sequence_dir,
                                             orbSlamExecutionInfoForSequence.sequence_file_base_name)

    for bagName in rosbagsSequence:
        singleTrajExecutionInfo = ORBSLAM2SingleRunConfig(
            orb_out_root_dir=orbSlamExecutionInfoForSequence.orb_out_root_dir,
            rosbag_base_name=bagName, orb_slam_vocabulary_file=orbSlamExecutionInfoForSequence.orb_slam_vocabulary_file,
            orb_slam_configuration_file=orbSlamExecutionInfoForSequence.orb_slam_configuration_file,
            rosbagDirectory=orbSlamExecutionInfoForSequence.rosbagDirectory,
            force_run_orb_slam_2=orbSlamExecutionInfoForSequence.force_run_orb_slam_2)

        runOrbSlam2SingleTrajectory(singleTrajExecutionInfo)

    if (not useSimTimeBefore):
        rospy.set_param('/use_sim_time', useSimTimeBefore)


def orbSlam2SequenceArgParse():
    parser = argparse.ArgumentParser(description="Run orb-slam2 for full trajectory sequence")

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

    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlam2OutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.orbSlam2OutRootDirHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRunORBSLAMBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRunORBSLAMHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRunORBSLAMBaseArgName,
                        dest=CmdLineArgConstants.forceRunORBSLAMBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRunORBSLAMBaseArgName)

    args_dict = vars(parser.parse_args())
    return ORBSLAM2SequenceConfig(
        orb_out_root_dir=args_dict[CmdLineArgConstants.orbSlam2OutRootDirBaseArgName],
        sequence_dir=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        sequence_file_base_name=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        orb_slam_vocabulary_file=args_dict[CmdLineArgConstants.orbSlamVocabularyFileBaseArgName],
        orb_slam_configuration_file=args_dict[CmdLineArgConstants.orbSlamConfigurationFileBaseArgName],
        rosbagDirectory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        force_run_orb_slam_2=args_dict[CmdLineArgConstants.forceRunORBSLAMBaseArgName])


if __name__ == "__main__":
    orbSlam2SeqConfig = orbSlam2SequenceArgParse()
    runOrbSlam2Sequence(orbSlam2SeqConfig)
