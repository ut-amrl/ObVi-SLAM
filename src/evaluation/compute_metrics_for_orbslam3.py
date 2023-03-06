import os
import argparse
from cmd_line_arg_utils import *
from compute_metrics_for_approach import *
from file_structure_utils import *
from trajectory_interpolation import *


class MetricsForOrbSLAM3Config:
    def __init__(self,
                 rosbag_dir,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_root_dir,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 odom_to_bl_extrinsics,
                 results_for_approach_root,
                 force_rerun_interpolator,
                 force_rerun_metrics_generator):
        self.rosbag_dir = rosbag_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_root_dir = lego_loam_root_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.odom_to_bl_extrinsics = odom_to_bl_extrinsics
        self.results_for_approach_root = results_for_approach_root
        self.force_rerun_interpolator = force_rerun_interpolator
        self.force_rerun_metrics_generator = force_rerun_metrics_generator


def generateMetricsForOrbSLAM3(metricsConfig):
    metrics_for_approach_config = MetricsForApproachConfig(
        rosbag_dir=metricsConfig.rosbag_dir,
        sequence_dir=metricsConfig.sequence_dir,
        sequence_file_base_name=metricsConfig.sequence_file_base_name,
        lego_loam_root_dir=metricsConfig.lego_loam_root_dir,
        lego_loam_frame_to_bl_extrinsics=metricsConfig.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=metricsConfig.comparison_alg_to_bl_extrinsics,
        odom_frame_rel_bl_file=metricsConfig.odom_to_bl_extrinsics,
        results_for_approach_root=metricsConfig.results_for_approach_root,
        within_sequence_dir_subdir=None,
        within_bagdir_sub_dir=None,
        force_rerun_interpolator=metricsConfig.force_rerun_interpolator,
        force_rerun_metrics_generator=metricsConfig.force_rerun_metrics_generator)
    generateMetricsForApproach(metrics_for_approach_config)


def metricsForOrbSLAM3ArgParse():
    parser = argparse.ArgumentParser(description="Generate metrics for ORBSLAM3 sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.rosbagDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.rosbagDirectoryHelp)
    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName),
        required=True,
        help=CmdLineArgConstants.trajectorySequenceFileDirectoryHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.sequenceFileBaseNameBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.sequenceFileBaseNameHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.orbSlam3OutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.orbSlam3OutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.legoLoamOutRootDirBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.legoLoamOutRootDirHelp)
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.calibrationFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.calibrationFileDirectoryHelp)
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

    # force_rerun_interpolator,
    # force_rerun_metrics_generator):
    args_dict = vars(parser.parse_args())
    calibrationFileDirectory = FileStructureUtils.ensureDirectoryEndsWithSlash(
        args_dict[CmdLineArgConstants.calibrationFileDirectoryBaseArgName])
    lego_loam_frame_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.legoLoamCalibFile
    comparison_alg_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.orbslam3CalibFile
    odom_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.odomCalibFile
    return MetricsForOrbSLAM3Config(rosbag_dir=args_dict[CmdLineArgConstants.rosbagDirectoryBaseArgName],
                                    sequence_dir=args_dict[
                                        CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
                                    sequence_file_base_name=args_dict[
                                        CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
                                    results_for_approach_root=args_dict[
                                        CmdLineArgConstants.orbSlam3OutRootDirBaseArgName],
                                    lego_loam_root_dir=args_dict[CmdLineArgConstants.legoLoamOutRootDirBaseArgName],
                                    lego_loam_frame_to_bl_extrinsics=lego_loam_frame_to_bl_extrinsics,
                                    comparison_alg_to_bl_extrinsics=comparison_alg_to_bl_extrinsics,
                                    odom_to_bl_extrinsics=odom_to_bl_extrinsics,
                                    force_rerun_interpolator=args_dict[
                                        CmdLineArgConstants.forceRerunInterpolatorBaseArgName],
                                    force_rerun_metrics_generator=args_dict[
                                        CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName])


if __name__ == "__main__":
    metricsForOrbConfig = metricsForOrbSLAM3ArgParse()
    generateMetricsForOrbSLAM3(metricsForOrbConfig)
