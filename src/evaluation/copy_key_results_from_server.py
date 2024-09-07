from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *
import argparse
import os


class CopyFromServerConstants:
    serverName = "robodata"


class CopyFromServerExecutionConfig:

    def __init__(self,
                 sequenceFilesDirectory, resultsRootDirectory, serverResultsRootDir, copyDestDir,
                 configFileBaseName, sequenceFileBaseName):
        self.sequenceFilesDirectory = sequenceFilesDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.serverResultsRootDir = serverResultsRootDir
        self.copyDestDir = copyDestDir
        self.sequenceFileBaseName = sequenceFileBaseName
        self.configFileBaseName = configFileBaseName


def copyResultsFromServer(executionConfig):
    rosbagsSequence = readTrajectorySequence(executionConfig.sequenceFilesDirectory,
                                             executionConfig.sequenceFileBaseName)

    dirForSequence = FileStructureUtils.ensureDirectoryEndsWithSlash(
        executionConfig.resultsRootDirectory) + executionConfig.sequenceFileBaseName
    FileStructureUtils.makeDirectoryIfNotExists(dirForSequence)
    copyDirForSequence = FileStructureUtils.ensureDirectoryEndsWithSlash(
        executionConfig.copyDestDir) + executionConfig.sequenceFileBaseName
    FileStructureUtils.makeDirectoryIfNotExists(copyDirForSequence)
    copyDirForSequenceAndConfig = FileStructureUtils.ensureDirectoryEndsWithSlash(
        copyDirForSequence) + executionConfig.configFileBaseName
    FileStructureUtils.makeDirectoryIfNotExists(copyDirForSequenceAndConfig)
    copyDirForSequenceAndConfig = FileStructureUtils.ensureDirectoryEndsWithSlash(copyDirForSequenceAndConfig)

    serverResultsForConfigName = FileStructureUtils.ensureDirectoryEndsWithSlash(
        FileStructureUtils.ensureDirectoryEndsWithSlash(
            FileStructureUtils.ensureDirectoryEndsWithSlash(
                executionConfig.serverResultsRootDir) + executionConfig.sequenceFileBaseName) + executionConfig.configFileBaseName)

    print("Copying metrics file from server")
    metricsCopyCmd = "scp " + CopyFromServerConstants.serverName + ":" + (
            serverResultsForConfigName + "metrics.json") + " " + copyDirForSequenceAndConfig
    os.system(metricsCopyCmd)

    objMetricsCopyCmd = "scp " + CopyFromServerConstants.serverName + ":" + (
            serverResultsForConfigName + "object_metrics.json") + " " + copyDirForSequenceAndConfig
    os.system(objMetricsCopyCmd)

    copySubdirs = ["ut_vslam_out", "postprocessing", "logs"]

    for idx, bagName in enumerate(rosbagsSequence):
        bagIdentifier = str(idx) + "_" + bagName
        serverDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(serverResultsForConfigName + bagIdentifier)
        copyDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(copyDirForSequenceAndConfig + bagIdentifier)
        FileStructureUtils.makeDirectoryIfNotExists(copyDirForBag)

        for copySubdir in copySubdirs:
            serverSubdirToCopy = serverDirForBag + copySubdir
            destDirForSubdir = copyDirForBag + copySubdir

            dirCopyCmd = "scp -r " + (
                    CopyFromServerConstants.serverName + ":" + serverSubdirToCopy) + " " + destDirForSubdir
            os.system(dirCopyCmd)

    dirForSequenceAndConfig = FileStructureUtils.ensureDirectoryEndsWithSlash(
        dirForSequence) + executionConfig.configFileBaseName

    os.system("ln -s " + copyDirForSequenceAndConfig + " " + dirForSequenceAndConfig)


def resultsCopyArgParse():
    parser = argparse.ArgumentParser(description="Write bounding boxes for all trajectories in sequence to file")
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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.serverResultsDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.serverResultsDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.copiedFromServerDestDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.copiedFromServerDestDirHelp)
    args_dict = vars(parser.parse_args())
    return CopyFromServerExecutionConfig(
        sequenceFilesDirectory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        resultsRootDirectory=args_dict[CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        copyDestDir=args_dict[CmdLineArgConstants.copiedFromServerDestDirBaseArgName],
        serverResultsRootDir=args_dict[CmdLineArgConstants.serverResultsDirBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName])


if __name__ == "__main__":
    executionConfig = resultsCopyArgParse()
    copyResultsFromServer(executionConfig)
