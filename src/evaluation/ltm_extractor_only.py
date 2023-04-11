from single_trajectory_estimator import *
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *


class LtmExtractorExecutionConfig:

    def __init__(self,
                 configFileDirectory, sequenceFilesDirectory, resultsRootDirectory, configFileBaseName,
                 sequenceFileBaseName, numberInSequence, outputJacobianDebugInfo=True, logToFile=False):
        self.configFileDirectory = configFileDirectory
        self.sequenceFilesDirectory = sequenceFilesDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.outputJacobianDebugInfo = outputJacobianDebugInfo
        self.logToFile = logToFile
        self.numberInSequence = numberInSequence


class LtmExtractorParamConstants:
    paramPrefix = "param_prefix"
    longTermMapInput = "long_term_map_input"
    longTermMapOutput = "long_term_map_output"
    ltmOptJacobianInfoDirectory = "ltm_opt_jacobian_info_directory"
    paramsConfigFile = "params_config_file"
    logsDirectory = "logs_directory"
    input_checkpoints_dir = "input_checkpoints_dir"


class LtmExtractorArgs:

    def __init__(self, param_prefix, long_term_map_input, long_term_map_output, ltm_opt_jacobian_info_directory,
                 params_config_file, logs_directory, input_checkpoints_dir):
        self.param_prefix = param_prefix
        self.long_term_map_input = long_term_map_input
        self.long_term_map_output = long_term_map_output
        self.ltm_opt_jacobian_info_directory = ltm_opt_jacobian_info_directory
        self.params_config_file = params_config_file
        self.logs_directory = logs_directory
        self.input_checkpoints_dir = input_checkpoints_dir


class LtmExtractorSpecificTrajectoryConfig:
    def __init__(self,
                 configFileDirectory, resultsRootDirectory, configFileBaseName,
                 sequenceFileBaseName, rosbagBaseName, resultsForBagDirPrefix, longTermMapBagDir,
                 outputJacobianDebugInfo=True, logToFile=False):
        self.configFileDirectory = configFileDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.rosbagBaseName = rosbagBaseName
        self.resultsForBagDirPrefix = resultsForBagDirPrefix
        self.longTermMapBagDir = longTermMapBagDir
        self.outputJacobianDebugInfo = outputJacobianDebugInfo
        self.logToFile = logToFile


def generateLtmExtractorArgsFromExecutionConfig(executionConfig):
    fullConfigFileName = os.path.join(executionConfig.configFileDirectory,
                                      (executionConfig.configFileBaseName + FileStructureConstants.jsonExtension))

    bag_results_dir_name = executionConfig.resultsForBagDirPrefix + executionConfig.rosbagBaseName

    # Param prefix
    param_prefix = executionConfig.sequenceFileBaseName + "_" + executionConfig.configFileBaseName + "_" + \
                   bag_results_dir_name

    utVslamOutRootDir = FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.resultsRootDirectory)
    utVslamResultsDir = FileStructureUtils.getAndOptionallyCreateUtVslamOutDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)

    jacobianDebuggingDir = None
    if (executionConfig.outputJacobianDebugInfo):
        jacobianDebuggingDir = FileStructureUtils.getAndOptionallyCreateJacobianDebuggingDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            bag_results_dir_name)

    checkpointsDir = FileStructureUtils.getAndOptionallyCreateCheckpointsDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)

    fileLogDir = None
    if (executionConfig.logToFile):
        fileLogDir = FileStructureUtils.getAndOptionallyCreateLogsDirectory(
            utVslamOutRootDir,
            executionConfig.sequenceFileBaseName,
            executionConfig.configFileBaseName,
            bag_results_dir_name)

    longTermMapOutputFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utVslamResultsDir) + FileStructureConstants.longTermMapFileBaseName

    longTermMapInputFile = None
    if (executionConfig.longTermMapBagDir is not None):
        longTermMapFileDir = FileStructureUtils.getAndOptionallyCreateUtVslamOutDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            executionConfig.longTermMapBagDir, False)
        longTermMapInputFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
            longTermMapFileDir) + FileStructureConstants.longTermMapFileBaseName

    ltmArgs = LtmExtractorArgs(param_prefix=param_prefix,
                               long_term_map_input=longTermMapInputFile,
                               long_term_map_output=longTermMapOutputFile,
                               ltm_opt_jacobian_info_directory=jacobianDebuggingDir,
                               params_config_file=fullConfigFileName,
                               logs_directory=fileLogDir,
                               input_checkpoints_dir=checkpointsDir)

    return ltmArgs

def runLtmExtractorFromArgs(ltmArgs):
    argsString = ""
    argsString += createCommandStrAddition(LtmExtractorParamConstants.paramPrefix,
                                           ltmArgs.param_prefix)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.longTermMapInput,
                                           ltmArgs.long_term_map_input)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.longTermMapOutput,
                                           ltmArgs.long_term_map_output)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.ltmOptJacobianInfoDirectory,
                                           ltmArgs.ltm_opt_jacobian_info_directory)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.paramsConfigFile,
                                           ltmArgs.params_config_file)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.logsDirectory,
                                           ltmArgs.logs_directory)
    argsString += createCommandStrAddition(LtmExtractorParamConstants.input_checkpoints_dir,
                                           ltmArgs.input_checkpoints_dir)

    cmdToRun = "./bin/ltm_extraction_only " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)

def runLtmExtractor(executionConfig):
    rosbagsSequence = readTrajectorySequence(executionConfig.sequenceFilesDirectory,
                                             executionConfig.sequenceFileBaseName)

    prevTrajectoryIdentifier = None

    for idx, bagName in enumerate(rosbagsSequence):
        bagPrefix = str(idx) + "_"
        print(type(executionConfig.numberInSequence))
        if (idx == executionConfig.numberInSequence):
            specificConfig = LtmExtractorSpecificTrajectoryConfig(
                configFileDirectory=executionConfig.configFileDirectory,
                resultsRootDirectory=executionConfig.resultsRootDirectory,
                configFileBaseName=executionConfig.configFileBaseName,
                sequenceFileBaseName=executionConfig.sequenceFileBaseName,
                rosbagBaseName=bagName,
                resultsForBagDirPrefix=bagPrefix,
                longTermMapBagDir=prevTrajectoryIdentifier,
                outputJacobianDebugInfo=executionConfig.outputJacobianDebugInfo,
                logToFile=executionConfig.logToFile)
            ltmArgs = generateLtmExtractorArgsFromExecutionConfig(specificConfig)
            runLtmExtractorFromArgs(ltmArgs)
            break
        prevTrajectoryIdentifier = bagPrefix + bagName


def ltmExtractorArgParse():
    parser = argparse.ArgumentParser(description="Run ltm extraction")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileDirectoryHelp)
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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.numberInSequenceBaseArgName),
                        required=True,
                        type=int,
                        help=CmdLineArgConstants.numberInSequenceHelp)

    # Boolean arguments
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.outputJacobianDebugInfoBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.outputJacobianDebugInfoHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.outputJacobianDebugInfoBaseArgName,
                        dest=CmdLineArgConstants.outputJacobianDebugInfoBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.outputJacobianDebugInfoBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.logToFileBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.logToFileHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.logToFileBaseArgName,
                        dest=CmdLineArgConstants.logToFileBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.logToFileBaseArgName)

    args_dict = vars(parser.parse_args())

    return LtmExtractorExecutionConfig(
        configFileDirectory=args_dict[CmdLineArgConstants.configFileDirectoryBaseArgName],
        sequenceFilesDirectory=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        resultsRootDirectory=args_dict[CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        numberInSequence=args_dict[CmdLineArgConstants.numberInSequenceBaseArgName],
        outputJacobianDebugInfo=args_dict[
            CmdLineArgConstants.outputJacobianDebugInfoBaseArgName],
        logToFile=args_dict[CmdLineArgConstants.logToFileBaseArgName])


if __name__ == "__main__":
    executionConfig = ltmExtractorArgParse()
    if (os.system("make") != 0):
        print("Make failed")
    else:
        runLtmExtractor(executionConfig)
