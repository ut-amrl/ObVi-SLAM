import os
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *
from trajectory_interpolation import *


class ObjectMetricsForApproachConfig:
    def __init__(self,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 results_for_approach_root,
                 within_sequence_dir_subdir,
                 within_bagdir_sub_dir,
                 gt_ellipsoids_file,
                 force_rerun_metrics_generator):
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.results_for_approach_root = results_for_approach_root
        self.within_sequence_dir_subdir = within_sequence_dir_subdir
        self.within_bagdir_sub_dir = within_bagdir_sub_dir
        self.gt_ellipsoids_file = gt_ellipsoids_file
        self.force_rerun_metrics_generator = force_rerun_metrics_generator


class ObjectMetricsGeneratorParamConstants:
    gt_ellipsoids_file = "gt_ellipsoids_file"
    gt_to_bl_extrinsics = "gt_to_bl_extrinsics"
    comparison_alg_est_dir = "comparison_alg_est_dir"
    comparison_alg_to_bl_extrinsics = "comparison_alg_to_bl_extrinsics"
    trajectory_results_dir_suffix = "trajectory_results_dir_suffix"
    sequence_file = "sequence_file"
    metrics_out_file = "metrics_out_file"
    param_prefix = "param_prefix"


class ObjectMetricsGeneratorConfig:
    def __init__(self, gt_ellipsoids_file, gt_to_bl_extrinsics, comparison_alg_est_dir, comparison_alg_to_bl_extrinsics,
                 trajectory_results_dir_suffix, sequence_file, metrics_out_file, param_prefix):
        self.gt_ellipsoids_file = gt_ellipsoids_file
        self.gt_to_bl_extrinsics = gt_to_bl_extrinsics
        self.comparison_alg_est_dir = comparison_alg_est_dir
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.trajectory_results_dir_suffix = trajectory_results_dir_suffix
        self.sequence_file = sequence_file
        self.metrics_out_file = metrics_out_file
        self.param_prefix = param_prefix


def runObjectMetricsGeneratorWithGeneratorConfig(generator_config):
    argsString = ""
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.gt_ellipsoids_file,
                                           generator_config.gt_ellipsoids_file)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.gt_to_bl_extrinsics,
                                           generator_config.gt_to_bl_extrinsics)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.comparison_alg_est_dir,
                                           generator_config.comparison_alg_est_dir)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.comparison_alg_to_bl_extrinsics,
                                           generator_config.comparison_alg_to_bl_extrinsics)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.trajectory_results_dir_suffix,
                                           generator_config.trajectory_results_dir_suffix)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.sequence_file,
                                           generator_config.sequence_file)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.metrics_out_file,
                                           generator_config.metrics_out_file)
    argsString += createCommandStrAddition(ObjectMetricsGeneratorParamConstants.param_prefix,
                                           generator_config.param_prefix)

    cmdToRun = "./bin/object_metrics_generator " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def generateObjectMetricsForApproach(metrics_for_approach_config):
    # Interpolate ground truth and store results
    # Call trajectory metrics generator

    within_seq_sub_dir = metrics_for_approach_config.within_sequence_dir_subdir
    if (within_seq_sub_dir is None):
        within_seq_sub_dir = ""
    if (len(within_seq_sub_dir) != 0):
        within_seq_sub_dir = FileStructureUtils.ensureDirectoryEndsWithSlash(within_seq_sub_dir)

    dirForSequenceResults = FileStructureUtils.ensureDirectoryEndsWithSlash(
        metrics_for_approach_config.results_for_approach_root) + \
                            metrics_for_approach_config.sequence_file_base_name + "/" + \
                            within_seq_sub_dir

    within_bagdir_sub_dir = metrics_for_approach_config.within_bagdir_sub_dir
    if (within_bagdir_sub_dir is None):
        within_bagdir_sub_dir = ""
    if (len(within_bagdir_sub_dir) != 0):
        within_bagdir_sub_dir = FileStructureUtils.ensureDirectoryEndsWithSlash(within_bagdir_sub_dir)

    metrics_out_file = dirForSequenceResults + FileStructureConstants.objectMetricsFileBaseName
    needToRerunMetrics = False
    if (metrics_for_approach_config.force_rerun_metrics_generator):
        needToRerunMetrics = True
    elif (not os.path.exists(metrics_out_file)):
        needToRerunMetrics = True
    if (not needToRerunMetrics):
        print("Metrics file already generated. Skipping metrics generator")
        return

    param_prefix = metrics_for_approach_config.sequence_file_base_name
    if (metrics_for_approach_config.within_sequence_dir_subdir is not None):
        param_prefix = param_prefix + "_" + metrics_for_approach_config.within_sequence_dir_subdir

    metrics_generator_config = ObjectMetricsGeneratorConfig(
        param_prefix=param_prefix,
        gt_ellipsoids_file=metrics_for_approach_config.gt_ellipsoids_file,
        gt_to_bl_extrinsics=metrics_for_approach_config.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=metrics_for_approach_config.comparison_alg_to_bl_extrinsics,
        sequence_file=generateSequenceFilePath(metrics_for_approach_config.sequence_dir,
                                               metrics_for_approach_config.sequence_file_base_name),
        metrics_out_file=metrics_out_file,
        comparison_alg_est_dir=dirForSequenceResults,
        trajectory_results_dir_suffix=within_bagdir_sub_dir)
    runObjectMetricsGeneratorWithGeneratorConfig(metrics_generator_config)

# if __name__ == "__main__":
#     metricsForApproachConfig = metricsForApproachArgParse()
#     generateMetricsForApproach(metricsForApproachConfig)
