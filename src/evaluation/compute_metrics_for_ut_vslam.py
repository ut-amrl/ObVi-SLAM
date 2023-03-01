import os
import argparse
from cmd_line_arg_utils import *
from compute_metrics_for_approach import *
from file_structure_utils import *


class MetricsForUTVSlamConfig:
    def __init__(self,
                 rosbag_dir,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_root_dir,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 odom_to_bl_extrinsics,
                 results_for_approach_root,
                 ut_vslam_preprocessing_root_dir,
                 config_base_name,
                 force_rerun_interpolator,
                 force_rerun_metrics_generator,
                 force_rerun_trajectory_formatter):
        self.rosbag_dir = rosbag_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_root_dir = lego_loam_root_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.odom_to_bl_extrinsics = odom_to_bl_extrinsics
        self.results_for_approach_root = results_for_approach_root  # UT VSLAM out root
        self.ut_vslam_preprocessing_root_dir = ut_vslam_preprocessing_root_dir
        self.config_base_name = config_base_name
        self.force_rerun_interpolator = force_rerun_interpolator
        self.force_rerun_metrics_generator = force_rerun_metrics_generator
        self.force_rerun_trajectory_formatter = force_rerun_trajectory_formatter


class TrajectoryReformatterParamConstants:
    poses_by_frame_file = "poses_by_frame_file"
    frames_for_timestamps_file = "frames_for_timestamps_file"
    poses_by_timestamp_out_file = "poses_by_timestamp_out_file"


def reformatUTVSLAMOutput(ut_vslam_preprocessing_dir, config_base_name, bag_name, utvslam_out_root_dir,
                          sequence_file_base_name, idx_in_sequence, forceReformatUTVSLAMOutput):
    frames_for_timestamps_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        ut_vslam_preprocessing_dir) + config_base_name + "/" + bag_name + "/" + FileStructureConstants.nodesByTimestampFileWithinSparsifiedDir
    results_for_bag_dir = FileStructureUtils.ensureDirectoryEndsWithSlash(
        utvslam_out_root_dir) + sequence_file_base_name + "/" + config_base_name + "/" + str(
        idx_in_sequence) + "_" + bag_name + "/"
    poses_by_frame_file = results_for_bag_dir + FileStructureConstants.utVslamOutRootDirBaseName + "/" + FileStructureConstants.robotPoseResultsFileBaseName
    poses_by_timestamp_dir = results_for_bag_dir + FileStructureConstants.postprocessingDirBaseName + "/"
    poses_by_timestamp_out_file = poses_by_timestamp_dir + FileStructureConstants.finalTrajectoryFileBaseName
    FileStructureUtils.makeDirectory(poses_by_timestamp_dir)

    needToRunReformatter = False
    if (forceReformatUTVSLAMOutput):
        needToRunReformatter = True
    elif (not os.path.exists(poses_by_timestamp_out_file)):
        needToRunReformatter = True
    if (not needToRunReformatter):
        print("Reformatted trajectory file already exists so skipping generation")
        return

    argsString = ""
    argsString += createCommandStrAddition(TrajectoryReformatterParamConstants.poses_by_frame_file,
                                           poses_by_frame_file)
    argsString += createCommandStrAddition(TrajectoryReformatterParamConstants.frames_for_timestamps_file,
                                           frames_for_timestamps_file)
    argsString += createCommandStrAddition(TrajectoryReformatterParamConstants.poses_by_timestamp_out_file,
                                           poses_by_timestamp_out_file)
    cmdToRun = "./bin/utvslam_out_to_poses_by_timestamp " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def reformatUTVSLAMOutputForSequence(ut_vslam_preprocessing_dir, config_base_name, utvslam_out_root_dir,
                                     sequence_file_dir, sequence_file_base_name, forceReformatUTVSLAMOutput):
    rosbagsSequence = readTrajectorySequence(sequence_file_dir, sequence_file_base_name)

    for idx, bagName in enumerate(rosbagsSequence):
        reformatUTVSLAMOutput(ut_vslam_preprocessing_dir, config_base_name, bagName, utvslam_out_root_dir,
                              sequence_file_base_name, idx, forceReformatUTVSLAMOutput)


def generateMetricsForUTVSLAM(metricsConfig):
    reformatUTVSLAMOutputForSequence(metricsConfig.ut_vslam_preprocessing_root_dir, metricsConfig.config_base_name,
                                     metricsConfig.results_for_approach_root, metricsConfig.sequence_dir,
                                     metricsConfig.sequence_file_base_name,
                                     metricsConfig.force_rerun_trajectory_formatter)
    metrics_for_approach_config = MetricsForApproachConfig(
        rosbag_dir=metricsConfig.rosbag_dir,
        sequence_dir=metricsConfig.sequence_dir,
        sequence_file_base_name=metricsConfig.sequence_file_base_name,
        lego_loam_root_dir=metricsConfig.lego_loam_root_dir,
        lego_loam_frame_to_bl_extrinsics=metricsConfig.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=metricsConfig.comparison_alg_to_bl_extrinsics,
        odom_frame_rel_bl_file=metricsConfig.odom_to_bl_extrinsics,
        results_for_approach_root=metricsConfig.results_for_approach_root,
        within_sequence_dir_subdir=metricsConfig.config_base_name,
        within_bagdir_sub_dir=FileStructureConstants.postprocessingDirBaseName,
        force_rerun_interpolator=metricsConfig.force_rerun_interpolator,
        force_rerun_metrics_generator=metricsConfig.force_rerun_metrics_generator)
    generateMetricsForApproach(metrics_for_approach_config)


def metricsForUTVSlamArgParse():
    parser = argparse.ArgumentParser(description="Generate metrics for UT VSLAM sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.resultsRootDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.resultsRootDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.calibrationFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.calibrationFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.configFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.configFileBaseNameHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbPostProcessBaseDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.orbPostProcessBaseDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRerunInterpolatorBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRerunInterpolatorHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRerunInterpolatorBaseArgName,
                        dest=CmdLineArgConstants.forceRerunInterpolatorBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRerunInterpolatorBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRerunMetricsGeneratorHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName,
                        dest=CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceReformatUTVSLAMOutputBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceReformatUTVSLAMOutputHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceReformatUTVSLAMOutputBaseArgName,
                        dest=CmdLineArgConstants.forceReformatUTVSLAMOutputBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceReformatUTVSLAMOutputBaseArgName)

    args_dict = vars(parser.parse_args())
    calibrationFileDirectory = FileStructureUtils.ensureDirectoryEndsWithSlash(
        args_dict[CmdLineArgConstants.calibrationFileDirectoryBaseArgName])
    lego_loam_frame_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.legoLoamCalibFile
    comparison_alg_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.ovslamCalibFile
    odom_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.odomCalibFile

    ut_vslam_preprocessing_root_dir = FileStructureUtils.ensureDirectoryEndsWithSlash(
        args_dict[
            CmdLineArgConstants.orbPostProcessBaseDirectoryBaseArgName]) + FileStructureConstants.sparsifiedDirectoryRootBaseName
    return MetricsForUTVSlamConfig(rosbag_dir=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
                                   sequence_dir=args_dict[
                                       CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
                                   sequence_file_base_name=args_dict[
                                       CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
                                   results_for_approach_root=args_dict[
                                       CmdLineArgConstants.resultsRootDirectoryBaseArgName],
                                   config_base_name=args_dict[CmdLineArgConstants.configFileBaseNameBaseArgName],
                                   lego_loam_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
                                   lego_loam_frame_to_bl_extrinsics=lego_loam_frame_to_bl_extrinsics,
                                   comparison_alg_to_bl_extrinsics=comparison_alg_to_bl_extrinsics,
                                   ut_vslam_preprocessing_root_dir=ut_vslam_preprocessing_root_dir,
                                   odom_to_bl_extrinsics=odom_to_bl_extrinsics,
                                   force_rerun_trajectory_formatter=args_dict[
                                       CmdLineArgConstants.forceReformatUTVSLAMOutputBaseArgName],
                                   force_rerun_interpolator=args_dict[
                                       CmdLineArgConstants.forceRerunInterpolatorBaseArgName],
                                   force_rerun_metrics_generator=args_dict[
                                       CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName])


if __name__ == "__main__":
    metricsForOrbConfig = metricsForUTVSlamArgParse()
    generateMetricsForUTVSLAM(metricsForOrbConfig)
