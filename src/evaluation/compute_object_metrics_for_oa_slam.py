import argparse
from cmd_line_arg_utils import *
from compute_object_metrics_for_approach import *
from file_structure_utils import *


class ObjectMetricsForOASlamConfig:
    def __init__(self,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 results_for_approach_root,
                 gt_ellipsoids_file,
                 force_rerun_metrics_generator):
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.results_for_approach_root = results_for_approach_root  # OA SLAM out root
        self.gt_ellipsoids_file = gt_ellipsoids_file
        self.force_rerun_metrics_generator = force_rerun_metrics_generator


def generateMetricsForOASLAM(metricsConfig):
    metrics_for_approach_config = ObjectMetricsForApproachConfig(
        sequence_dir=metricsConfig.sequence_dir,
        sequence_file_base_name=metricsConfig.sequence_file_base_name,
        lego_loam_frame_to_bl_extrinsics=metricsConfig.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=metricsConfig.comparison_alg_to_bl_extrinsics,
        results_for_approach_root=metricsConfig.results_for_approach_root,
        within_sequence_dir_subdir=None,
        within_bagdir_sub_dir=None,
        gt_ellipsoids_file=metricsConfig.gt_ellipsoids_file,
        force_rerun_metrics_generator=metricsConfig.force_rerun_metrics_generator)
    generateObjectMetricsForApproach(metrics_for_approach_config)


def objectMetricsForOASlamArgParse():
    parser = argparse.ArgumentParser(description="Generate object metrics for OA-SLAM sequence")
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.gtEllipsoidsBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.gtEllipsoidsHelp)
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
    parser.add_argument(CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.calibrationFileDirectoryBaseArgName),
                        required=True,
                        help=CmdLineArgConstants.calibrationFileDirectoryHelp)

    parser.add_argument(
        CmdLineArgConstants.prefixWithDashDash(CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName),
        default=False,
        action='store_true',
        help=CmdLineArgConstants.forceRerunMetricsGeneratorHelp)
    parser.add_argument('--no-' + CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName,
                        dest=CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName, action='store_false',
                        help="Opposite of " + CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName)

    args_dict = vars(parser.parse_args())
    calibrationFileDirectory = FileStructureUtils.ensureDirectoryEndsWithSlash(
        args_dict[CmdLineArgConstants.calibrationFileDirectoryBaseArgName])
    lego_loam_frame_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.legoLoamCalibFile
    comparison_alg_to_bl_extrinsics = calibrationFileDirectory + CalibrationFileConstants.orbslam3CalibFile

    return ObjectMetricsForOASlamConfig(
        sequence_dir=args_dict[
            CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        sequence_file_base_name=args_dict[
            CmdLineArgConstants.sequenceFileBaseNameBaseArgName],
        lego_loam_frame_to_bl_extrinsics=lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=comparison_alg_to_bl_extrinsics,
        results_for_approach_root=args_dict[
            CmdLineArgConstants.resultsRootDirectoryBaseArgName],
        gt_ellipsoids_file=args_dict[CmdLineArgConstants.gtEllipsoidsBaseArgName],
        force_rerun_metrics_generator=args_dict[
            CmdLineArgConstants.forceRerunMetricsGeneratorBaseArgName])


if __name__ == "__main__":
    objectMetricsConfig = objectMetricsForOASlamArgParse()
    generateMetricsForOASLAM(objectMetricsConfig)
