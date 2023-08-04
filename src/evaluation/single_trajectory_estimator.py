import os
import subprocess
import signal
import argparse
import time
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_interpolation import *
import csv


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
    ground_truth_trajectory_file = "ground_truth_trajectory_file"
    ground_truth_extrinsics_file = "ground_truth_extrinsics_file"
    output_checkpoints_dir = "output_checkpoints_dir"
    input_checkpoints_dir = "input_checkpoints_dir"
    disable_log_to_stderr = "disable_log_to_stderr"


class OrbTrajectorySparsifierParamConstants:
    param_prefix = "param_prefix"
    input_processed_data_path = "input_processed_data_path"
    output_processed_data_path = "output_processed_data_path"
    params_config_file = "params_config_file"
    required_timestamps_file = "required_timestamps_file"
    required_stamps_by_node_file = "required_stamps_by_node_file"


class SingleTrajectoryExecutionConfig:

    def __init__(self,
                 configFileDirectory, orbSlamOutDirectory, rosbagDirectory,
                 orbPostProcessBaseDirectory, calibrationFileDirectory, resultsRootDirectory, configFileBaseName,
                 sequenceFileBaseName, rosbagBaseName, waypointFileBaseName, resultsForBagDirPrefix, longTermMapBagDir,
                 lego_loam_root_dir, odometryTopic,
                 forceRunOrbSlamPostProcess=False, outputEllipsoidDebugInfo=True, outputJacobianDebugInfo=True,
                 outputBbAssocInfo=True, runRviz=False, recordVisualizationRosbag=False, logToFile=False,
                 forceRerunInterpolator=False, outputCheckpoints=False, readCheckpoints=False,
                 disableLogToStdErr=False):
        self.configFileDirectory = configFileDirectory
        self.orbSlamOutDirectory = orbSlamOutDirectory
        self.rosbagDirectory = rosbagDirectory
        self.orbPostProcessBaseDirectory = orbPostProcessBaseDirectory
        self.calibrationFileDirectory = calibrationFileDirectory
        self.resultsRootDirectory = resultsRootDirectory
        self.configFileBaseName = configFileBaseName
        self.sequenceFileBaseName = sequenceFileBaseName
        self.rosbagBaseName = rosbagBaseName
        self.waypointFileBaseName = waypointFileBaseName
        self.resultsForBagDirPrefix = resultsForBagDirPrefix
        self.longTermMapBagDir = longTermMapBagDir
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


class OfflineRunnerArgs:

    def __init__(self, param_prefix, intrinsics_file, extrinsics_file, poses_by_node_id_file, nodes_by_timestamp_file,
                 rosbag_file, long_term_map_input, long_term_map_output, low_level_feats_dir, bb_associations_out_file,
                 ltm_opt_jacobian_info_directory, visual_feature_results_file, debug_images_output_directory,
                 params_config_file, ellipsoids_results_file, robot_poses_results_file, logs_directory, gt_poses_file,
                 gt_extrinsics_file, output_checkpoints_dir, input_checkpoints_dir, disable_log_to_stderr,
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
        self.gt_poses_file = gt_poses_file
        self.gt_extrinsics_file = gt_extrinsics_file
        self.output_checkpoints_dir = output_checkpoints_dir
        self.input_checkpoints_dir = input_checkpoints_dir
        self.disable_log_to_stderr = disable_log_to_stderr


def runOrbPostProcess(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                      configFileName, paramPrefix, fullWaypointFileName=None, forceRunOrbSlamPostProcess=False,
                      deletePrevData=True):
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

    needToRerunOrbReformatter = False
    if (forceRunOrbSlamPostProcess):
        needToRerunOrbReformatter = True
    else:
        if (os.path.exists(unsparsifiedUtVslamInDir)):
            num_files = len(os.listdir(unsparsifiedUtVslamInDir))
            if (num_files < 5):
                # If we have less than this many files there's probably something wrong
                needToRerunOrbReformatter = True
        else:
            needToRerunOrbReformatter = True

    if (needToRerunOrbReformatter):
        cmd1 = "python3 src/data_preprocessing_utils/orb_stereo_reformat_data.py -i " + orbDataDirForBag + " -o " + unsparsifiedUtVslamInDir
        print("Running command: ")
        print(cmd1)
        os.system(cmd1)

        cmd2 = "./bin/initialize_traj_and_feats_from_orb_out --raw_data_path " + orbDataDirForBag + \
               " --calibration_path " + calibrationDir + " --processed_data_path " + unsparsifiedUtVslamInDir
        print("Running command: ")
        print(cmd2)
        os.system(cmd2)

    sparsifierArgsString = ""
    sparsifierArgsString += createCommandStrAddition(OrbTrajectorySparsifierParamConstants.input_processed_data_path,
                                                     unsparsifiedUtVslamInDir)
    sparsifierArgsString += createCommandStrAddition(OrbTrajectorySparsifierParamConstants.output_processed_data_path,
                                                     sparsifiedUtVslamInDir)
    sparsifierArgsString += createCommandStrAddition(OrbTrajectorySparsifierParamConstants.params_config_file,
                                                     configFileName)
    sparsifierArgsString += createCommandStrAddition(OrbTrajectorySparsifierParamConstants.required_timestamps_file,
                                                     fullWaypointFileName)
    sparsifierArgsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.paramPrefix,
                                                     paramPrefix)

    cmd3 = "./bin/orb_trajectory_sparsifier " + sparsifierArgsString
    print("Running command: ")
    print(cmd3)
    os.system(cmd3)


def ensureOrbDataPostProcessed(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                               configFileName, forceRunOrbSlamPostProcess, fullWaypointFileName, paramPrefix):
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
        # Only delete if we're force rerunning
        runOrbPostProcess(orbDataDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir, calibrationDir,
                          configFileName, paramPrefix, fullWaypointFileName, forceRunOrbSlamPostProcess,
                          forceRunOrbSlamPostProcess)


def generateRequiredStampsFileFromNodesAndTimestampsFile(nodesAndTimestampsFile, requiredStampsFile):
    with open(nodesAndTimestampsFile) as nodesAndStampsCsv:
        with open(requiredStampsFile, 'w') as requiredStampsCsv:
            nodesReader = csv.reader(nodesAndStampsCsv, delimiter=',')
            timestampsWriter = csv.writer(requiredStampsCsv, delimiter=',')
            for row in nodesReader:
                timestampsOnly = row[1:]
                timestampsWriter.writerow(timestampsOnly)


def runInterpolatorIfNecessary(executionConfig, postProcessingDir, sparsifiedUtVslamInDir, paramPrefix):
    if (executionConfig.lego_loam_root_dir is None):
        print("Lego loam root directory not provided, so not getting GT")
        return (None, None)
    nodesAndTimestampsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        sparsifiedUtVslamInDir) + FileStructureConstants.nodesByTimestampFileWithinSparsifiedDir
    requiredStampsFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        sparsifiedUtVslamInDir) + FileStructureConstants.timestampsOnlyFileWithinSparsifiedDir
    generateRequiredStampsFileFromNodesAndTimestampsFile(nodesAndTimestampsFile, requiredStampsFile)

    gtExtrinsicsRelBlFile = FileStructureUtils.ensureDirectoryEndsWithSlash(
        executionConfig.calibrationFileDirectory) + CalibrationFileConstants.legoLoamCalibFile
    interpolatedPosesFileName = runInterpolator(executionConfig.rosbagDirectory, executionConfig.rosbagBaseName,
                                                postProcessingDir,
                                                executionConfig.lego_loam_root_dir,
                                                None,
                                                gtExtrinsicsRelBlFile,
                                                FileStructureUtils.ensureDirectoryEndsWithSlash(
                                                    executionConfig.calibrationFileDirectory) + CalibrationFileConstants.odomCalibFile,
                                                executionConfig.odometryTopic,
                                                executionConfig.forceRerunInterpolator,
                                                requiredStampsFile,
                                                paramPrefix)
    return (interpolatedPosesFileName, gtExtrinsicsRelBlFile)


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

    bag_results_dir_name = executionConfig.resultsForBagDirPrefix + executionConfig.rosbagBaseName

    # Param prefix
    param_prefix = executionConfig.sequenceFileBaseName + "_" + executionConfig.configFileBaseName + "_" + \
                   bag_results_dir_name

    fullWaypointFileName = None
    if (executionConfig.waypointFileBaseName is not None):
        fullWaypointFileName = FileStructureUtils.createWaypointTimestampsFileName(executionConfig.rosbagDirectory,
                                                                                   executionConfig.waypointFileBaseName)
    ensureOrbDataPostProcessed(orbOutputDirForBag, unsparsifiedUtVslamInDir, sparsifiedUtVslamInDir,
                               calibrationFileDirectory, fullConfigFileName, executionConfig.forceRunOrbSlamPostProcess,
                               fullWaypointFileName, param_prefix)

    intrinsicsFile = calibrationFileDirectory + FileStructureConstants.intrinsicsBaseName
    extrinsicsFile = calibrationFileDirectory + FileStructureConstants.extrinsicsBaseName

    utVslamOutRootDir = FileStructureUtils.ensureDirectoryEndsWithSlash(executionConfig.resultsRootDirectory)
    utVslamResultsDir = FileStructureUtils.getAndOptionallyCreateUtVslamOutDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)

    postProcessingDir = FileStructureUtils.getAndOptionallyCreateUtVslamPostprocessingDirectory(
        utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
        bag_results_dir_name)

    (interpolatedPosesFileName, gtExtrinsicsRelBlFile) = runInterpolatorIfNecessary(executionConfig, postProcessingDir,
                                                                                    sparsifiedUtVslamInDir,
                                                                                    param_prefix)

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

    outputCheckpointsDir = None
    inputCheckpointsDir = None
    if (executionConfig.outputCheckpoints or executionConfig.readCheckpoints):
        checkpointsDir = FileStructureUtils.getAndOptionallyCreateCheckpointsDirectory(
            utVslamOutRootDir, executionConfig.sequenceFileBaseName, executionConfig.configFileBaseName,
            bag_results_dir_name)
        if (executionConfig.outputCheckpoints):
            outputCheckpointsDir = checkpointsDir
        if (executionConfig.readCheckpoints):
            inputCheckpointsDir = checkpointsDir

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

    gt_poses_file = None
    gt_extrinsics_file = None
    if (interpolatedPosesFileName is not None):
        gt_poses_file = interpolatedPosesFileName
        gt_extrinsics_file = gtExtrinsicsRelBlFile

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
                                    logs_directory=fileLogDir,
                                    gt_poses_file=gt_poses_file,
                                    gt_extrinsics_file=gt_extrinsics_file,
                                    output_checkpoints_dir=outputCheckpointsDir,
                                    input_checkpoints_dir=inputCheckpointsDir,
                                    disable_log_to_stderr=executionConfig.disableLogToStdErr)
    return (param_prefix, offlineArgs)


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
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.ground_truth_trajectory_file,
                                           offlineArgs.gt_poses_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.ground_truth_extrinsics_file,
                                           offlineArgs.gt_extrinsics_file)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.output_checkpoints_dir,
                                           offlineArgs.output_checkpoints_dir)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.input_checkpoints_dir,
                                           offlineArgs.input_checkpoints_dir)
    argsString += createCommandStrAddition(SingleTrajectoryExecutableParamConstants.disable_log_to_stderr,
                                           offlineArgs.disable_log_to_stderr)

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
    bagName = FileStructureUtils.ensureDirectoryEndsWithSlash(rosbagRecordDir) + "pending_visualization_topics.bag"
    permanentBagName = FileStructureUtils.ensureDirectoryEndsWithSlash(rosbagRecordDir) + "visualization_topics.bag"
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
    cmdArgs.append((topicsPrefix + "/gt_pose"))
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
    cmdArgs.append("/tf")
    processReturn = subprocess.Popen(cmdArgs, preexec_fn=os.setsid)
    return (processReturn, bagName, permanentBagName)


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
    pendingVisBagName = None
    permanentVisBagName = None
    if (executionConfig.recordVisualizationRosbag):
        processReturn, pendingVisBagName, permanentVisBagName = recordVisualizationRosbag(topicsPrefix, executionConfig)
    runTrajectoryFromOfflineArgs(offlineArgs)

    if (executionConfig.recordVisualizationRosbag):
        os.killpg(os.getpgid(processReturn.pid), signal.SIGTERM)
        timeStart = time.time()
        while (not os.path.exists(pendingVisBagName)):
            if ((time.time() - timeStart) > 10):
                break
            time.sleep(0.1)

        if (os.path.exists(pendingVisBagName)):
            os.system("mv " + pendingVisBagName + " " + permanentVisBagName)


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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.waypointFileBaseNameBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.waypointFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsForBagDirPrefixBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.resultsForBagDirPrefixHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.longTermMapBagDirBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.longTermMapBagDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.odometryTopicBaseArgName),
                        required=False,
                        help=CmdLineArgConstants.odometryTopicHelp)

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
        waypointFileBaseName=args_dict[CmdLineArgConstants.waypointFileBaseNameBaseArgName],
        resultsForBagDirPrefix=args_dict[CmdLineArgConstants.resultsForBagDirPrefixBaseArgName],
        longTermMapBagDir=args_dict[CmdLineArgConstants.longTermMapBagDirBaseArgName],
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
        disableLogToStdErr=args_dict[CmdLineArgConstants.disableLogToStdErrBaseArgName])


if __name__ == "__main__":
    # Run arg parse to get command line args
    executionConfig = singleTrajectoryArgParse()
    if (os.system("make") != 0):
        print("Make failed")
    else:
        runSingleTrajectory(executionConfig)
