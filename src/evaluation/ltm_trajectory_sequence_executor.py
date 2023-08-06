from single_trajectory_estimator import *
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *


class TrajectorySequenceExecutionConfig:

    def __init__(self,
                 configFileDirectory, orbSlamOutDirectory, rosbagDirectory,
                 orbPostProcessBaseDirectory, calibrationFileDirectory,
                 sequenceFilesDirectory, resultsRootDirectory, configFileBaseName,
                 sequenceFileBaseName, lego_loam_root_dir, odometryTopic,
                 forceRunOrbSlamPostProcess=False, outputEllipsoidDebugInfo=True, outputJacobianDebugInfo=True,
                 outputBbAssocInfo=True, runRviz=False, recordVisualizationRosbag=False, logToFile=False,
                 forceRerunInterpolator=False, outputCheckpoints=False, readCheckpoints=False,
                 disableLogToStdErr=False,
                 boundingBoxesPostProcessBaseDirectory=None):
        self.configFileDirectory = configFileDirectory
        self.orbSlamOutDirectory = orbSlamOutDirectory
        self.rosbagDirectory = rosbagDirectory
        self.orbPostProcessBaseDirectory = orbPostProcessBaseDirectory
        self.calibrationFileDirectory = calibrationFileDirectory
        self.sequenceFilesDirectory = sequenceFilesDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.lego_loam_root_dir = lego_loam_root_dir
        self.odometryTopic = odometryTopic
        self.forceRunOrbSlamPostProcess = forceRunOrbSlamPostProcess
        self.outputEllipsoidDebugInfo = outputEllipsoidDebugInfo
        self.outputJacobianDebugInfo = outputJacobianDebugInfo
        self.outputBbAssocInfo = outputBbAssocInfo
        self.runRviz = runRviz
        self.recordVisualizationRosbag = recordVisualizationRosbag
        self.logToFile = logToFile
        self.forceRerunInterpolator = forceRerunInterpolator
        self.outputCheckpoints = outputCheckpoints
        self.readCheckpoints = readCheckpoints
        self.disableLogToStdErr = disableLogToStdErr
        self.boundingBoxesPostProcessBaseDirectory = boundingBoxesPostProcessBaseDirectory


def runTrajectorySequence(sequenceExecutionConfig):
    rosbagsSequence = readTrajectorySequenceAndWaypoints(sequenceExecutionConfig.sequenceFilesDirectory,
                                             sequenceExecutionConfig.sequenceFileBaseName)

    prevTrajectoryIdentifier = None

    for idx, sequenceEntry in enumerate(rosbagsSequence):
        bagName = sequenceEntry[0]
        waypointInfo = sequenceEntry[1]
        bagPrefix = str(idx) + "_"
        # if (idx != 0):
        trajectoryExecutionConfig = SingleTrajectoryExecutionConfig(
            configFileDirectory=sequenceExecutionConfig.configFileDirectory,
            orbSlamOutDirectory=sequenceExecutionConfig.orbSlamOutDirectory,
            rosbagDirectory=sequenceExecutionConfig.rosbagDirectory,
            orbPostProcessBaseDirectory=sequenceExecutionConfig.orbPostProcessBaseDirectory,
            calibrationFileDirectory=sequenceExecutionConfig.calibrationFileDirectory,
            resultsRootDirectory=sequenceExecutionConfig.resultsRootDirectory,
            configFileBaseName=sequenceExecutionConfig.configFileBaseName,
            sequenceFileBaseName=sequenceExecutionConfig.sequenceFileBaseName,
            rosbagBaseName=bagName,
            waypointFileBaseName=waypointInfo,
            resultsForBagDirPrefix=bagPrefix,
            longTermMapBagDir=prevTrajectoryIdentifier,
            lego_loam_root_dir=sequenceExecutionConfig.lego_loam_root_dir,
            odometryTopic=sequenceExecutionConfig.odometryTopic,
            forceRunOrbSlamPostProcess=sequenceExecutionConfig.forceRunOrbSlamPostProcess,
            outputEllipsoidDebugInfo=sequenceExecutionConfig.outputEllipsoidDebugInfo,
            outputJacobianDebugInfo=sequenceExecutionConfig.outputJacobianDebugInfo,
            outputBbAssocInfo=sequenceExecutionConfig.outputBbAssocInfo,
            runRviz=sequenceExecutionConfig.runRviz,
            recordVisualizationRosbag=sequenceExecutionConfig.recordVisualizationRosbag,
            logToFile=sequenceExecutionConfig.logToFile,
            forceRerunInterpolator=sequenceExecutionConfig.forceRerunInterpolator,
            outputCheckpoints=sequenceExecutionConfig.outputCheckpoints,
            readCheckpoints=sequenceExecutionConfig.readCheckpoints,
            disableLogToStdErr=sequenceExecutionConfig.disableLogToStdErr,
            boundingBoxesPostProcessBaseDirectory=sequenceExecutionConfig.boundingBoxesPostProcessBaseDirectory)
        runSingleTrajectory(trajectoryExecutionConfig)
        prevTrajectoryIdentifier = bagPrefix + bagName


def trajectorySequenceArgParse():
    parser = argparse.ArgumentParser(description="Run sequence of trajectory")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlamOutDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.orbSlamOutDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbPostProcessBaseDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.orbPostProcessBaseDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.calibrationFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.calibrationFileDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)

    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.odometryTopicBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.odometryTopicHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryBaseArgName),
        required=False,
        help=CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRunOrbSlamPostProcessHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName,
                        dest=CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.outputEllipsoidDebugInfoHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName,
                        dest=CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.outputJacobianDebugInfoBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.outputJacobianDebugInfoHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.outputJacobianDebugInfoBaseArgName,
                        dest=CmdLineArgConstants.outputJacobianDebugInfoBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.outputJacobianDebugInfoBaseArgName)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.outputBbAssocInfoBaseArgName),
                        default=False,
                        action='store_true',
                        help=CmdLineArgConstants.outputBbAssocInfoHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.outputBbAssocInfoBaseArgName,
                        dest=CmdLineArgConstants.outputBbAssocInfoBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.outputBbAssocInfoBaseArgName)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.runRvizBaseArgName),
                        default=False,
                        action='store_true',
                        help=CmdLineArgConstants.runRvizHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.runRvizBaseArgName,
                        dest=CmdLineArgConstants.runRvizBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.runRvizBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.recordVisualizationRosbagBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.recordVisualizationRosbagHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.recordVisualizationRosbagBaseArgName,
                        dest=CmdLineArgConstants.recordVisualizationRosbagBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.recordVisualizationRosbagBaseArgName)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.logToFileBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.logToFileHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.logToFileBaseArgName,
                        dest=CmdLineArgConstants.logToFileBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.logToFileBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRerunInterpolatorBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRerunInterpolatorHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRerunInterpolatorBaseArgName,
                        dest=CmdLineArgConstants.forceRerunInterpolatorBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRerunInterpolatorBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.outputCheckpointsBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.outputCheckpointsHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.outputCheckpointsBaseArgName,
                        dest=CmdLineArgConstants.outputCheckpointsBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.outputCheckpointsBaseArgName)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.readCheckpointsBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.readFromCheckpointsHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.readCheckpointsBaseArgName,
                        dest=CmdLineArgConstants.readCheckpointsBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.readCheckpointsBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.disableLogToStdErrBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.disableLogToStdErrHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.disableLogToStdErrBaseArgName,
                        dest=CmdLineArgConstants.disableLogToStdErrBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.disableLogToStdErrBaseArgName)

    args_dict = vars(parser.parse_args())

    return TrajectorySequenceExecutionConfig(
        configFileDirectory=args_dict[CmdLineArgConstants.configFileDirectoryBaseArgName],
        orbSlamOutDirectory=args_dict[CmdLineArgConstants.orbSlamOutDirectoryBaseArgName],
        rosbagDirectory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        orbPostProcessBaseDirectory=args_dict[
            CmdLineArgConstants.orbPostProcessBaseDirectoryBaseArgName],
        calibrationFileDirectory=args_dict[
            CmdLineArgConstants.calibrationFileDirectoryBaseArgName],
        sequenceFilesDirectory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        resultsRootDirectory=args_dict[CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        lego_loam_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
        odometryTopic=args_dict[CmdLineArgConstants.odometryTopicBaseArgName],
        forceRunOrbSlamPostProcess=args_dict[
            CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName],
        outputEllipsoidDebugInfo=args_dict[
            CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName],
        outputJacobianDebugInfo=args_dict[
            CmdLineArgConstants.outputJacobianDebugInfoBaseArgName],
        outputBbAssocInfo=args_dict[CmdLineArgConstants.outputBbAssocInfoBaseArgName],
        runRviz=args_dict[CmdLineArgConstants.runRvizBaseArgName],
        recordVisualizationRosbag=args_dict[CmdLineArgConstants.recordVisualizationRosbagBaseArgName],
        logToFile=args_dict[CmdLineArgConstants.logToFileBaseArgName],
        forceRerunInterpolator=args_dict[CmdLineArgConstants.forceRerunInterpolatorBaseArgName],
        outputCheckpoints=args_dict[CmdLineArgConstants.outputCheckpointsBaseArgName],
        readCheckpoints=args_dict[CmdLineArgConstants.readCheckpointsBaseArgName],
        disableLogToStdErr=args_dict[CmdLineArgConstants.disableLogToStdErrBaseArgName],
        boundingBoxesPostProcessBaseDirectory=args_dict[CmdLineArgConstants.boundingBoxesPostProcessBaseDirectoryBaseArgName])


if __name__ == "__main__":
    executionConfig = trajectorySequenceArgParse()
    if (os.system("make") != 0):
        print("Make failed")
    else:
        runTrajectorySequence(executionConfig)
