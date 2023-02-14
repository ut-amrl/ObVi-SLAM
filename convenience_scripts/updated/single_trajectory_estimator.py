import os
import subprocess
import signal
import argparse
import time


class CmdLineArgConstants:
    configFileDirectoryBaseArgName = 'config_file_directory'
    trajectorySequenceFileDirectoryBaseArgName = 'trajectory_sequence_file_directory'
    orbSlamOutDirectoryBaseArgName = 'orb_slam_out_directory'
    rosbagDirectoryBaseArgName = 'rosbag_file_directory'
    orbPostProcessBaseDirectoryBaseArgName = 'orb_post_process_base_directory'
    calibrationFileDirectoryBaseArgName = 'calibration_file_directory'
    resultsRootDirectoryBaseArgName = 'results_root_directory'
    configFileBaseNameBaseArgName = 'config_file_base_name'
    sequenceFileBaseNameBaseArgName = 'sequence_file_base_name'
    rosbagBaseNameBaseArgName = 'rosbag_base_name'
    resultsForBagDirPrefixBaseArgName = 'results_for_bag_dir_prefix'
    longTermMapBagDirBaseArgName = 'long_term_map_bag_dir_arg_name'

    # Boolean arg names
    forceRunOrbSlamPostProcessBaseArgName = 'force_run_orb_post_process'
    outputEllipsoidDebugInfoBaseArgName = 'output_ellipsoid_debug'
    outputJacobianDebugInfoBaseArgName = 'output_jacobian_debug'
    outputBbAssocInfoBaseArgName = 'output_bb_assoc'
    runRvizBaseArgName = 'run_rviz'
    recordVisualizationRosbagBaseArgName = 'record_viz_rosbag'
    logToFileBaseArgName = 'log_to_file'

    configFileDirectoryHelp = \
        "Directory where config files are stored"
    trajectorySequenceFileDirectoryHelp = \
        "Directory where sequences of trajectories to run together (i.e. build up a long term map) are stored"
    orbSlamOutDirectoryHelp = \
        "Root directory where orb slam output is stored. For each bag, there should be a directory with the base name " \
        "title of the bag under this directory and the orbslam files will be stored under that subdirectory."
    rosbagDirectoryHelp = "Root directory where rosbags are stored"
    orbPostProcessBaseDirectoryHelp = \
        "Base directory where orbslam post processing files will be stored. Under this directory, there should be a " \
        "directory for each configuration (directory will have config file name) and beneath that, there will be a " \
        "directory for each bag"
    calibrationFileDirectoryHelp = \
        "Directory containing the intrinsics and extrinsics files. In this directory, the intrinsics file is named " \
        "'camera_matrix.txt' and the extrinsics file is named 'extrinsics.txt'"
    resultsRootDirectoryHelp = \
        "Directory that contains the results. Within this directory, there will be a directory corresponding to the " \
        "sequence_base_name and within those directories will be a directory corresponding to the config base name. " \
        "Within the sequence-config directory, there will be a directory for each trajectory in the sequence. For " \
        "trajectories run in isolation, this will be named the same as the rosbag name. For trajectories from a" \
        " sequence (even if there is only one entry in the sequence), they will be prefixed with the number of the " \
        "trajectory in the sequence. Within the directory for the particular trajectory, there will be" \
        "(depending on the output configuration) directories named 'ut_vslam_out' (which contains the main " \
        "executable results), 'jacobian_debugging_out' (which will contain information for debugging jacobian rank " \
        "deficiencies experienced in covariance extraction), 'ellipsoid_debugging_out' (which contains debug" \
        " information/images for helping to tune/debug the ellipsoid front-end and estimation), 'logs' (which contains " \
        "the logs for the run, this is configured mainly by glog), and 'visualization_rosbags' (which contains " \
        "rosbags that have recorded the main visualization messages that are generated during trajectory optimization)"

    configFileBaseNameHelp = \
        "Base name of the configuration file. The config file used will be a file under the config file directory " \
        "with the suffix '.json' appended to it (this argument should not have the file extension)"
    sequenceFileBaseNameHelp = \
        "Base name of the file indicating the sequence of trajectories to run. This should be the base name of a " \
        "json file (argument specified here should not have json suffix). The sequence file that this points to will" \
        " contain a list of rosbag base names to execute in sequence as well as an identifier for the sequence"
    rosbagBaseNameHelp = "Base name of the rosbag that these results are for. "
    resultsForBagDirPrefixHelp = \
        "Prefix to add to the rosbag name that should be used for storing results. If part of a sequence, this should " \
        "be the number of the trajectory in the sequence (starting with 0) followed by an underscore"
    longTermMapBagDirHelp = \
        "If specified, this will be the directory (under the sequence then config directories) that will contain the" \
        " long term map that should be used as input for this trajectory"

    forceRunOrbSlamPostProcessHelp = "Force running the orb slam post processor even if there is data there"
    outputEllipsoidDebugInfoHelp = "Output the ellipsoid debug data to file while running"
    outputJacobianDebugInfoHelp = "Output the jacobian debug data to file while running"
    outputBbAssocInfoHelp = "Output the bounding box associations at the end of the trajectory"
    runRvizHelp = "Start the rviz visualization while optimizing a trajectory"
    recordVisualizationRosbagHelp = "Record a rosbag containing the debug visualization messages"
    logToFileHelp = "True if the process should log to file, false if only to standard error"

    @staticmethod
    def prefixWithDashDash(argName):
        return '--' + argName


class FileStructureConstants:
    intrinsicsBaseName = "camera_matrix.txt"
    extrinsicsBaseName = "extrinsics.txt"

    unsparsifiedDirectoryRootBaseName = "unsparsified_ut_vslam_in"
    sparsifiedDirectoryRootBaseName = "sparsified_ut_vslam_in"

    posesByNodeIdFileWithinSparsifiedDir = "poses/initial_robot_poses_by_node.txt"
    nodesByTimestampFileWithinSparsifiedDir = "timestamps/node_ids_and_timestamps.txt"

    jacobianDebuggingRootDirBaseName = "jacobian_debugging_out"
    ellipsoidDebuggingRootDirBaseName = "ellipsoid_debugging_out"
    utVslamOutRootDirBaseName = "ut_vslam_out"
    visualizationRosbagRootDirBaseName = "visualization_rosbags"
    logsRootDirBaseName = "logs"

    longTermMapFileBaseName = "long_term_map.json"
    visualFeatureResultsFileBaseName = "visual_feature_results.json"
    bbAssocResultsFileBaseName = "data_association_results.json"

    ellipsoidResultsFileBaseName = "ellipsoid_results.json"
    robotPoseResultsFileBaseName = "robot_pose_results.json"

    # File extensions
    jsonExtension = ".json"
    bagSuffix = ".bag"


class SingleTrajectoryExecutableParamConstants:
    paramPrefix = "param_prefix"
    intrinsicsFile = "intrinsics_file"
    extrinsicsFile = "extrinsics_file"
    posesByNodeIdFile = "poses_by_node_id_file"
    nodesByTimestampFile = "nodes_by_timestamp_file"
    rosbagFile = "rosbag_file"
    longTermMapInput = "long_term_map_input"
    longTermMapOutput = "long_term_map_output"
    lowLevelFeatsDir = "low_level_feats_dir"
    bbAssociationsOutFile = "bb_associations_out_file"
    ltmOptJacobianInfoDirectory = "ltm_opt_jacobian_info_directory"
    visualFeatureResultsFile = "visual_feature_results_file"
    debugImagesOutputDirectory = "debug_images_output_directory"
    paramsConfigFile = "params_config_file"
    boundingBoxesByNodeIdFile = "bounding_boxes_by_node_id_file"
    ellipsoidsResultsFile = "ellipsoids_results_file"
    robotPosesResultsFile = "robot_poses_results_file"
    logsDirectory = "logs_directory"


class FileStructureUtils:
    @staticmethod
    def makeDirectory(directoryName):
        os.system("mkdir -p " + directoryName)

    @staticmethod
    def ensureDirectoryEndsWithSlash(directoryName):
        if (len(directoryName) == 0):
            raise Exception("Directory name can't be empty")
        if (directoryName[-1] == "/"):
            return directoryName
        return directoryName + "/"

    @staticmethod
    def getAndOptionallyCreateConfigSpecificResultsDirectory(resultsBaseDir, resultsTypeVariantName, sequenceBaseName,
                                                             configBaseName, dirForBagResults, create):
        directoryName = FileStructureUtils.ensureDirectoryEndsWithSlash(
            resultsBaseDir) + sequenceBaseName + "/" + \
                        configBaseName + "/" + dirForBagResults + "/" + resultsTypeVariantName + "/"
        if (create):
            FileStructureUtils.makeDirectory(directoryName)
        # TODO Should we force remove old data
        return directoryName

    @staticmethod
    def getAndOptionallyCreateUtVslamOutDirectory(resultsBaseDir, sequenceBaseName, configBaseName, dirForBagResults,
                                                  create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.utVslamOutRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateJacobianDebuggingDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                         dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.jacobianDebuggingRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateEllipsoidDebuggingDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                          dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.ellipsoidDebuggingRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateVisualizationRosbagDirectory(resultsBaseDir, sequenceBaseName, configBaseName,
                                                           dirForBagResults, create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.visualizationRosbagRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)

    @staticmethod
    def getAndOptionallyCreateLogsDirectory(resultsBaseDir, sequenceBaseName, configBaseName, dirForBagResults,
                                            create=True):
        return FileStructureUtils.getAndOptionallyCreateConfigSpecificResultsDirectory(
            resultsBaseDir, FileStructureConstants.logsRootDirBaseName, sequenceBaseName, configBaseName,
            dirForBagResults, create)


class SingleTrajectoryExecutionConfig:

    def __init__(self,
                 configFileDirectory, orbSlamOutDirectory, rosbagDirectory,
                 orbPostProcessBaseDirectory, calibrationFileDirectory, resultsRootDirectory, configFileBaseName,
                 sequenceFileBaseName, rosbagBaseName, resultsForBagDirPrefix, longTermMapBagDir,
                 forceRunOrbSlamPostProcess=False, outputEllipsoidDebugInfo=True, outputJacobianDebugInfo=True,
                 outputBbAssocInfo=True, runRviz=False, recordVisualizationRosbag=False, logToFile=False):
        self.configFileDirectory = configFileDirectory
        self.orbSlamOutDirectory = orbSlamOutDirectory
        self.rosbagDirectory = rosbagDirectory
        self.orbPostProcessBaseDirectory = orbPostProcessBaseDirectory
        self.calibrationFileDirectory = calibrationFileDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.rosbagBaseName = rosbagBaseName
        self.resultsForBagDirPrefix = resultsForBagDirPrefix
        self.longTermMapBagDir = longTermMapBagDir
        self.forceRunOrbSlamPostProcess = forceRunOrbSlamPostProcess
        self.outputEllipsoidDebugInfo = outputEllipsoidDebugInfo
        self.outputJacobianDebugInfo = outputJacobianDebugInfo
        self.outputBbAssocInfo = outputBbAssocInfo
        self.runRviz = runRviz
        self.recordVisualizationRosbag = recordVisualizationRosbag
        self.logToFile = logToFile


class OfflineRunnerArgs:

    def __init__(self, param_prefix, intrinsics_file, extrinsics_file, poses_by_node_id_file, nodes_by_timestamp_file,
                 rosbag_file, long_term_map_input, long_term_map_output, low_level_feats_dir, bb_associations_out_file,
                 ltm_opt_jacobian_info_directory, visual_feature_results_file, debug_images_output_directory,
                 params_config_file, ellipsoids_results_file, robot_poses_results_file, logs_directory,
                 bounding_boxes_by_node_id_file=None):
        self.param_prefix = param_prefix
        self.intrinsics_file = intrinsics_file
        self.extrinsics_file = extrinsics_file
        self.poses_by_node_id_file = poses_by_node_id_file
        self.nodes_by_timestamp_file = nodes_by_timestamp_file
        self.rosbag_file = rosbag_file
        self.long_term_map_input = long_term_map_input
        self.long_term_map_output = long_term_map_output
        self.low_level_feats_dir = low_level_feats_dir
        self.bb_associations_out_file = bb_associations_out_file
        self.ltm_opt_jacobian_info_directory = ltm_opt_jacobian_info_directory
        self.visual_feature_results_file = visual_feature_results_file
        self.debug_images_output_directory = debug_images_output_directory
        self.params_config_file = params_config_file
        self.ellipsoids_results_file = ellipsoids_results_file
        self.robot_poses_results_file = robot_poses_results_file
        self.logs_directory = logs_directory
        self.bounding_boxes_by_node_id_file = bounding_boxes_by_node_id_file


def runOrbPostProcess(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                      configFileName, deletePrevData=True):
    print("Unsparsified ut vslam in dir: ")
    print(unsparsifiedUtVslamInDir)

    print("Sparsified ut vslam in dir: ")
    print(sparsifiedUtVslamInDir)

    print("Calibration dir ")
    print(calibrationDir)

    if (deletePrevData):
        os.system("rm -r " + unsparsifiedUtVslamInDir + " " + sparsifiedUtVslamInDir)

    FileStructureUtils.makeDirectory(unsparsifiedUtVslamInDir)
    FileStructureUtils.makeDirectory(sparsifiedUtVslamInDir)

    cmd1 = "python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i " + orbDataDirForBag + " -o " + unsparsifiedUtVslamInDir
    print("Running command: ")
    print(cmd1)
    os.system(cmd1)

    cmd2 = "./bin/initialize_traj_and_feats_from_orb_out --raw_data_path " + orbDataDirForBag + \
           " --calibration_path " + calibrationDir + " --processed_data_path " + unsparsifiedUtVslamInDir
    print("Running command: ")
    print(cmd2)
    os.system(cmd2)

    cmd3 = "./bin/orb_trajectory_sparsifier --input_processed_data_path " + unsparsifiedUtVslamInDir + \
           " --output_processed_data_path " + sparsifiedUtVslamInDir + " --params_config_file " + configFileName
    print("Running command: ")
    print(cmd3)
    os.system(cmd3)


def ensureOrbDataPostProcessed(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                               configFileName, forceRunOrbSlamPostProcess):
    needToRerun = False
    if (forceRunOrbSlamPostProcess):
        needToRerun = True
    else:
        if (os.path.exists(sparsifiedUtVslamInDir)):
            num_files = len(os.listdir(sparsifiedUtVslamInDir))
            if (num_files < 5):
                # If we have less than this many files there's probably something wrong
                needToRerun = True
        else:
            needToRerun = True

    if (needToRerun):
        runOrbPostProcess(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                          configFileName)


def generateOfflineRunnerArgsFromExecutionConfigAndPreprocessOrbDataIfNecessary(executionConfig):
    rosbagBaseName = executionConfig.rosbagBaseName
    calibrationFileDirectory = FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.calibrationFileDirectory)
    fullConfigFileName = os.path.join(executionConfig.configFileDirectory,
                                      (executionConfig.configFileBaseName + FileStructureConstants.jsonExtension))

    # Run any required preprocessing
    orbOutputDirForBag = FileStructureUtils.ensureDirectoryEndsWithSlash(
        FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.orbSlamOutDirectory) + rosbagBaseName)

    unsparsifiedUtVslamInDir = FileStructureUtils.ensureDirectoryEndsWithSlash(
        os.path.join(executionConfig.orbPostProcessBaseDirectory,
                     FileStructureConstants.unsparsifiedDirectoryRootBaseName, rosbagBaseName))

    sparsifiedUtVslamInDir = FileStructureUtils.ensureDirectoryEndsWithSlash(
        os.path.join(executionConfig.orbPostProcessBaseDirectory,
                     FileStructureConstants.sparsifiedDirectoryRootBaseName, executionConfig.configFileBaseName,
                     rosbagBaseName))

    poses_by_node_id_file = sparsifiedUtVslamInDir + FileStructureConstants.posesByNodeIdFileWithinSparsifiedDir
    nodes_by_timestamp_file = sparsifiedUtVslamInDir + FileStructureConstants.nodesByTimestampFileWithinSparsifiedDir
    rosbag_file_name = FileStructureUtils.ensureDirectoryEndsWithSlash(
        executionConfig.rosbagDirectory) + rosbagBaseName + FileStructureConstants.bagSuffix

    ensureOrbDataPostProcessed(orbOutputDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir,
                               calibrationFileDirectory, fullConfigFileName, executionConfig.forceRunOrbSlamPostProcess)

    bag_results_dir_name = executionConfig.resultsForBagDirPrefix + executionConfig.rosbagBaseName

    # Param prefix

    param_prefix = executionConfig.sequenceFileBaseName + "_" + executionConfig.configFileBaseName + "_" + \
                   bag_results_dir_name

    intrinsicsFile = calibrationFileDirectory + FileStructureConstants.intrinsicsBaseName
    extrinsicsFile = calibrationFileDirectory + FileStructureConstants.extrinsicsBaseName

    utVslamOutRootDir = FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.resultsRootDirectory)
    utVslamResultsDir = FileStructureUtils.getAndOptionallyCreateUtVslamOutDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)

    jacobianDebuggingDir = None
    if (executionConfig.outputJacobianDebugInfo):
        jacobianDebuggingDir = FileStructureUtils.getAndOptionallyCreateJacobianDebuggingDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            bag_results_dir_name)

    ellipsoidDebuggingDir = None
    if (executionConfig.outputEllipsoidDebugInfo):
        ellipsoidDebuggingDir = FileStructureUtils.getAndOptionallyCreateEllipsoidDebuggingDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            bag_results_dir_name)

    bbAssociationsFile = None
    if (executionConfig.outputBbAssocInfo):
        bbAssociationsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
            utVslamResultsDir) + FileStructureConstants.bbAssocResultsFileBaseName

    fileLogDir = None
    if (executionConfig.logToFile):
        fileLogDir = FileStructureUtils.getAndOptionallyCreateLogsDirectory(
            utVslamOutRootDir,
            executionConfig.sequenceFileBaseName,
            executionConfig.configFileBaseName,
            bag_results_dir_name)

    longTermMapOutputFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utVslamResultsDir) + FileStructureConstants.longTermMapFileBaseName
    visualFeatureResultsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utVslamResultsDir) + FileStructureConstants.visualFeatureResultsFileBaseName
    ellipsoidResultsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utVslamResultsDir) + FileStructureConstants.ellipsoidResultsFileBaseName
    robotPoseResultsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utVslamResultsDir) + FileStructureConstants.robotPoseResultsFileBaseName

    longTermMapInputFile = None
    if (executionConfig.longTermMapBagDir is not None):
        longTermMapFileDir = FileStructureUtils.getAndOptionallyCreateUtVslamOutDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            executionConfig.longTermMapBagDir, False)
        longTermMapInputFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
            longTermMapFileDir) + FileStructureConstants.longTermMapFileBaseName

    offlineArgs = OfflineRunnerArgs(param_prefix=param_prefix,
                                    intrinsics_file=intrinsicsFile,
                                    extrinsics_file=extrinsicsFile,
                                    poses_by_node_id_file=poses_by_node_id_file,
                                    nodes_by_timestamp_file=nodes_by_timestamp_file,
                                    rosbag_file=rosbag_file_name,
                                    long_term_map_input=longTermMapInputFile,
                                    long_term_map_output=longTermMapOutputFile,
                                    low_level_feats_dir=sparsifiedUtVslamInDir,
                                    bb_associations_out_file=bbAssociationsFile,
                                    ltm_opt_jacobian_info_directory=jacobianDebuggingDir,
                                    visual_feature_results_file=visualFeatureResultsFile,
                                    debug_images_output_directory=ellipsoidDebuggingDir,
                                    params_config_file=fullConfigFileName,
                                    ellipsoids_results_file=ellipsoidResultsFile,
                                    robot_poses_results_file=robotPoseResultsFile,
                                    logs_directory=fileLogDir)
    return (param_prefix, offlineArgs)


def createCommandStrAddition(argumentName, argumentValue):
    if (argumentValue is None):
        return ""
    return CmdLineArgConstants.prefixWithDashDash(argumentName) + " " + argumentValue + " "


def runTrajectoryFromOfflineArgs(offlineArgs):
    argsString = ""
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.paramPrefix,
                                           offlineArgs.param_prefix)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.intrinsicsFile,
                                           offlineArgs.intrinsics_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.extrinsicsFile,
                                           offlineArgs.extrinsics_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.posesByNodeIdFile,
                                           offlineArgs.poses_by_node_id_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.nodesByTimestampFile,
                                           offlineArgs.nodes_by_timestamp_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.rosbagFile, offlineArgs.rosbag_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.longTermMapInput,
                                           offlineArgs.long_term_map_input)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.longTermMapOutput,
                                           offlineArgs.long_term_map_output)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.lowLevelFeatsDir,
                                           offlineArgs.low_level_feats_dir)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.bbAssociationsOutFile,
                                           offlineArgs.bb_associations_out_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.ltmOptJacobianInfoDirectory,
                                           offlineArgs.ltm_opt_jacobian_info_directory)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.visualFeatureResultsFile,
                                           offlineArgs.visual_feature_results_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.debugImagesOutputDirectory,
                                           offlineArgs.debug_images_output_directory)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.paramsConfigFile,
                                           offlineArgs.params_config_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.boundingBoxesByNodeIdFile,
                                           offlineArgs.bounding_boxes_by_node_id_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.ellipsoidsResultsFile,
                                           offlineArgs.ellipsoids_results_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.robotPosesResultsFile,
                                           offlineArgs.robot_poses_results_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.logsDirectory,
                                           offlineArgs.logs_directory)

    cmdToRun = "./bin/offline_object_visual_slam_main " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def recordVisualizationRosbag(topicsPrefix, executionConfig):
    print("Starting recording")
    utVslamOutRootDir = FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.resultsRootDirectory)
    bag_results_dir_name = executionConfig.resultsForBagDirPrefix + executionConfig.rosbagBaseName
    rosbagRecordDir = FileStructureUtils.getAndOptionallyCreateVisualizationRosbagDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)
    bagName = FileStructureUtils.ensureDirectoryEndsWithSlash(rosbagRecordDir) + "visualization_topics.bag";
    cmdArgs = []
    cmdArgs.append("rosbag")
    cmdArgs.append("record")
    cmdArgs.append("-O")
    cmdArgs.append(bagName)
    cmdArgs.append((topicsPrefix + "/est_ellipsoids"))
    cmdArgs.append((topicsPrefix + "/est_/feature_cloud"))
    cmdArgs.append((topicsPrefix + "/init_/feature_cloud"))
    cmdArgs.append((topicsPrefix + "/est_pose"))
    cmdArgs.append((topicsPrefix + "/init_pose"))
    cmdArgs.append((topicsPrefix + "/pending_ellipsoids"))
    cmdArgs.append((topicsPrefix + "/init_ellipsoids"))

    cmdArgs.append((topicsPrefix + "/est_/latest/cam_1_bb/image_raw"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_2_bb/image_raw"))
    cmdArgs.append((topicsPrefix + "/all_observed_bbs/latest/cam_1_bb/image_raw"))
    cmdArgs.append((topicsPrefix + "/all_observed_bbs/latest/cam_2_bb/image_raw"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_1_feats/image_raw"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_2_feats/image_raw"))
    cmdArgs.append((topicsPrefix + "/init_/latest/cam_1_feats/image_raw"))
    cmdArgs.append((topicsPrefix + "/init_/latest/cam_2_feats/image_raw"))

    cmdArgs.append((topicsPrefix + "/est_/latest/cam_1_bb/camera_info"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_2_bb/camera_info"))
    cmdArgs.append((topicsPrefix + "/all_observed_bbs/latest/cam_1_bb/camera_info"))
    cmdArgs.append((topicsPrefix + "/all_observed_bbs/latest/cam_2_bb/camera_info"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_1_feats/camera_info"))
    cmdArgs.append((topicsPrefix + "/est_/latest/cam_2_feats/camera_info"))
    cmdArgs.append((topicsPrefix + "/init_/latest/cam_1_feats/camera_info"))
    cmdArgs.append((topicsPrefix + "/init_/latest/cam_2_feats/camera_info"))
    processReturn = subprocess.Popen(cmdArgs, preexec_fn=os.setsid)
    return processReturn


def runSingleTrajectory(executionConfig):
    paramPrefix, offlineArgs = generateOfflineRunnerArgsFromExecutionConfigAndPreprocessOrbDataIfNecessary(
        executionConfig)
    topicsPrefix = ""
    underscore_based_prefix = ""
    if (len(paramPrefix) != 0):
        topicsPrefix = "/" + paramPrefix + "/"
        underscore_based_prefix = paramPrefix
    if (executionConfig.runRviz):
        rvizCmd = "roslaunch launch/ovslam_rviz.launch topics_prefix:=" + topicsPrefix + " underscore_based_prefix:=" \
                  + underscore_based_prefix + " &"
        os.system(rvizCmd)
    processReturn = None
    if (executionConfig.recordVisualizationRosbag):
        processReturn = recordVisualizationRosbag(topicsPrefix, executionConfig)
    runTrajectoryFromOfflineArgs(offlineArgs)

    if (executionConfig.recordVisualizationRosbag):
        os.killpg(os.getpgid(processReturn.pid), signal.SIGTERM)


def singleTrajectoryArgParse():
    parser = argparse.ArgumentParser(description="Run single trajectory")
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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)

    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsForBagDirPrefixBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.resultsForBagDirPrefixHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.longTermMapBagDirBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.longTermMapBagDirHelp)

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
    args_dict = vars(parser.parse_args())

    return SingleTrajectoryExecutionConfig(
        configFileDirectory=args_dict[CmdLineArgConstants.configFileDirectoryBaseArgName],
        orbSlamOutDirectory=args_dict[CmdLineArgConstants.orbSlamOutDirectoryBaseArgName],
        rosbagDirectory=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
        orbPostProcessBaseDirectory=args_dict[
            CmdLineArgConstants.orbPostProcessBaseDirectoryBaseArgName],
        calibrationFileDirectory=args_dict[
            CmdLineArgConstants.calibrationFileDirectoryBaseArgName],
        resultsRootDirectory=args_dict[CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        configFileBaseName=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
        sequenceFileBaseName=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        rosbagBaseName=args_dict[CmdLineArgConstants.rosbagBaseNameBaseArgName],
        resultsForBagDirPrefix=args_dict[CmdLineArgConstants.resultsForBagDirPrefixBaseArgName],
        longTermMapBagDir=args_dict[CmdLineArgConstants.longTermMapBagDirBaseArgName],
        forceRunOrbSlamPostProcess=args_dict[
            CmdLineArgConstants.forceRunOrbSlamPostProcessBaseArgName],
        outputEllipsoidDebugInfo=args_dict[
            CmdLineArgConstants.outputEllipsoidDebugInfoBaseArgName],
        outputJacobianDebugInfo=args_dict[
            CmdLineArgConstants.outputJacobianDebugInfoBaseArgName],
        outputBbAssocInfo=args_dict[CmdLineArgConstants.outputBbAssocInfoBaseArgName],
        runRviz=args_dict[CmdLineArgConstants.runRvizBaseArgName],
        recordVisualizationRosbag=args_dict[CmdLineArgConstants.recordVisualizationRosbagBaseArgName],
        logToFile=args_dict[CmdLineArgConstants.logToFileBaseArgName])


if __name__ == "__main__":
    # Run arg parse to get command line args
    executionConfig = singleTrajectoryArgParse()
    if (os.system("make") != 0):
        print("Make failed")
    else:
        runSingleTrajectory(executionConfig)
