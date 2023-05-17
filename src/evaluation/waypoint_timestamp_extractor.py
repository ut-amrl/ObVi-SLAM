import argparse
import os
from file_structure_utils import *
from cmd_line_arg_utils import *

class WaypointExtractorExecutionConfig:
    def __init__(self, rosbag_base_name, rosbag_file_directory, waypoint_topic_trigger, camera_topics_file,
                 waypoints_file_version, force_waypoint_file_regeneration=False):
        self.rosbag_base_name = rosbag_base_name
        self.rosbag_file_directory = rosbag_file_directory
        self.waypoint_topic_trigger = waypoint_topic_trigger
        self.camera_topics_file = camera_topics_file
        self.waypoints_file_version = waypoints_file_version
        self.force_waypoint_file_regeneration = force_waypoint_file_regeneration


class WaypointExtractorParamConstants:
    rosbagFileArgName = "rosbag_file"
    timestampsForWaypointsOutFileArgName = "timestamps_for_waypoints_out_file"
    cameraTopicsListFileArgName = "camera_topics_list_file"
    waypointTriggerTopicArgName = "waypoint_trigger_topic"


def runWaypointExtraction(waypointExtractorExecutionConfig):
    fullBagName = FileStructureUtils.createRosbagFileName(waypointExtractorExecutionConfig.rosbag_file_directory,
                                                          waypointExtractorExecutionConfig.rosbag_base_name)

    waypointOutputFileName = FileStructureUtils.createWaypointTimestampsFileNameFromComponents(
        waypointExtractorExecutionConfig.rosbag_file_directory,
        waypointExtractorExecutionConfig.rosbag_base_name,
        waypointExtractorExecutionConfig.waypoints_file_version)

    needToRun = False
    if (waypointExtractorExecutionConfig.force_waypoint_file_regeneration):
        needToRun = True
    elif (not os.path.exists(waypointOutputFileName)):
        needToRun = True

    if (not needToRun):
        print("Waypoints file already exists, so skipping")

    argsString = ""
    argsString += createCommandStrAddition(WaypointExtractorParamConstants.waypointTriggerTopicArgName,
                                           waypointExtractorExecutionConfig.waypoint_topic_trigger)
    argsString += createCommandStrAddition(WaypointExtractorParamConstants.rosbagFileArgName,
                                           fullBagName)
    argsString += createCommandStrAddition(WaypointExtractorParamConstants.cameraTopicsListFileArgName,
                                           waypointExtractorExecutionConfig.camera_topics_file)
    argsString += createCommandStrAddition(WaypointExtractorParamConstants.timestampsForWaypointsOutFileArgName,
                                           waypointOutputFileName)

    cmdToRun = "./bin/waypoint_timestamp_extractor " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def waypointExtractorArgParse():
    parser = argparse.ArgumentParser(description="Run waypoint extractor for the given bag")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagBaseNameBaseArgName),
        required=True,
        help=CmdLineArgConstants.rosbagBaseNameHelp)

    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.waypointTopicTriggerBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.waypointTopicTriggerHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.cameraTopicsFileBaseArgName),
        required=True,
        help=CmdLineArgConstants.cameraTopicsFileHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.waypointFileVersionBaseArgName),
        required=True,
        help=CmdLineArgConstants.waypointFileVersionHelp)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceWaypointFileRegenerationBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceWaypointFileRegenerationHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceWaypointFileRegenerationBaseArgName,
                        dest=CmdLineArgConstants.forceWaypointFileRegenerationBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceWaypointFileRegenerationBaseArgName)


    args_dict = vars(parser.parse_args())
    return WaypointExtractorExecutionConfig(
        rosbag_base_name=args_dict[CmdLineArgConstants.rosbagBaseNameBaseArgName],
        rosbag_file_directory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        waypoint_topic_trigger=args_dict[CmdLineArgConstants.waypointTopicTriggerBaseArgName],
        camera_topics_file=args_dict[CmdLineArgConstants.cameraTopicsFileBaseArgName],
        waypoints_file_version=args_dict[CmdLineArgConstants.waypointFileVersionBaseArgName],
        force_waypoint_file_regeneration=args_dict[CmdLineArgConstants.waypointTopicTriggerBaseArgName])


if __name__ == "__main__":
    waypointExtractorExecutionConfig = waypointExtractorArgParse()
    runWaypointExtraction(waypointExtractorExecutionConfig)
