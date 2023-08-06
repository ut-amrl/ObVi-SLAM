from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *
import argparse
import os


class BoundingBoxWriterRosbagExecutionConfig:

    def __init__(self,
                 configFileDirectory, rosbagDirectory,
                 boundingBoxesPostProcessBaseDirectory, configFileBaseName,
                 rosbagBaseName,
                 forceRerunBoundingBoxGeneration=False):
        self.configFileDirectory = configFileDirectory
        self.rosbagDirectory = rosbagDirectory
        self.boundingBoxesPostProcessBaseDirectory = boundingBoxesPostProcessBaseDirectory
        self.configFileBaseName = configFileBaseName
        self.rosbagBaseName = rosbagBaseName
        self.forceRerunBoundingBoxGeneration = forceRerunBoundingBoxGeneration


class BoundingBoxWriterSequenceExecutionConfig:

    def __init__(self,
                 configFileDirectory, rosbagDirectory,
                 boundingBoxesPostProcessBaseDirectory,
                 sequenceFilesDirectory, configFileBaseName,
                 sequenceFileBaseName,
                 forceRerunBoundingBoxGeneration=False):
        self.configFileDirectory = configFileDirectory
        self.rosbagDirectory = rosbagDirectory
        self.boundingBoxesPostProcessBaseDirectory = boundingBoxesPostProcessBaseDirectory
        self.sequenceFilesDirectory = sequenceFilesDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.forceRerunBoundingBoxGeneration = forceRerunBoundingBoxGeneration


class BoundingBoxWriterParamConstants:
    bounding_boxes_by_timestamp_out_file = "bounding_boxes_by_timestamp_out_file"
    rosbag_file = "rosbag_file"
    params_config_file = "params_config_file"


def extractBoundingBoxesForSingleBag(singleBagExecutionConfig):
    bounding_box_out_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        singleBagExecutionConfig.boundingBoxesPostProcessBaseDirectory) + \
                            FileStructureConstants.boundingBoxFilePrefix + singleBagExecutionConfig.rosbagBaseName + FileStructureConstants.csvExtension

    if (singleBagExecutionConfig.forceRerunBoundingBoxGeneration or (not os.path.exists(bounding_box_out_file))):
        fullConfigFileName = os.path.join(singleBagExecutionConfig.configFileDirectory,
                                          (singleBagExecutionConfig.configFileBaseName +
                                           FileStructureConstants.jsonExtension))
        rosbag_file_name = FileStructureUtils.ensureDirectoryEndsWithSlash(
            singleBagExecutionConfig.rosbagDirectory) + singleBagExecutionConfig.rosbagBaseName + FileStructureConstants.bagSuffix

        argsString = ""
        argsString += createCommandStrAddition(BoundingBoxWriterParamConstants.bounding_boxes_by_timestamp_out_file,
                                               bounding_box_out_file)
        argsString += createCommandStrAddition(BoundingBoxWriterParamConstants.rosbag_file, rosbag_file_name)
        argsString += createCommandStrAddition(BoundingBoxWriterParamConstants.params_config_file, fullConfigFileName)

        cmdToRun = "./bin/write_bounding_boxes_for_rosbag_to_file " + argsString
        print("Running command: ")
        print(cmdToRun)
        os.system(cmdToRun)


def writeBbsForSequence(bbWriterSequenceExecutionConfig):
    rosbagsSequence = readTrajectorySequence(bbWriterSequenceExecutionConfig.sequenceFilesDirectory,
                                             bbWriterSequenceExecutionConfig.sequenceFileBaseName)

    for idx, sequenceEntry in enumerate(rosbagsSequence):
        bagName = sequenceEntry
        # if (idx != 0):
        singleBagExecutionConfig = BoundingBoxWriterRosbagExecutionConfig(
            configFileDirectory=bbWriterSequenceExecutionConfig.configFileDirectory,
            boundingBoxesPostProcessBaseDirectory=bbWriterSequenceExecutionConfig.boundingBoxesPostProcessBaseDirectory,
            rosbagDirectory=bbWriterSequenceExecutionConfig.rosbagDirectory,
            configFileBaseName=bbWriterSequenceExecutionConfig.configFileBaseName,
            rosbagBaseName=bagName,
            forceRerunBoundingBoxGeneration=bbWriterSequenceExecutionConfig.forceRerunBoundingBoxGeneration)
        extractBoundingBoxesForSingleBag(singleBagExecutionConfig)


def bbSequenceGenerationArgParse():
    parser = argparse.ArgumentParser(description="Write bounding boxes for all trajectories in sequence to file")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRerunBoundingBoxGenerationBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRerunBoundingBoxGenerationHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRerunBoundingBoxGenerationBaseArgName,
                        dest=CmdLineArgConstants.forceRerunBoundingBoxGenerationBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRerunBoundingBoxGenerationBaseArgName)

    args_dict = vars(parser.parse_args())

    return BoundingBoxWriterSequenceExecutionConfig(
        configFileDirectory=args_dict[CmdLineArgConstants.configFileDirectoryBaseArgName],
        rosbagDirectory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        boundingBoxesPostProcessBaseDirectory=args_dict[
            CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryBaseArgName],
        sequenceFilesDirectory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        forceRerunBoundingBoxGeneration=args_dict[
            CmdLineArgConstants.forceRerunBoundingBoxGenerationBaseArgName])


if __name__ == "__main__":
    executionConfig = bbSequenceGenerationArgParse()
    if (os.system("make") != 0):
        print("Make failed")
    else:
        writeBbsForSequence(executionConfig)
