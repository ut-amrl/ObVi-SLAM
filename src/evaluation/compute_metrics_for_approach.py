import os
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *
from trajectory_interpolation import *


class MetricsForApproachConfig:
    def __init__(self, rosbag_dir,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_root_dir,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 odom_frame_rel_bl_file,
                 results_for_approach_root,
                 within_sequence_dir_subdir,
                 within_bagdir_sub_dir,
                 odometry_topic,
                 force_rerun_interpolator,
                 force_rerun_metrics_generator):
        self.rosbag_dir = rosbag_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_root_dir = lego_loam_root_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.odom_frame_rel_bl_file = odom_frame_rel_bl_file
        self.results_for_approach_root = results_for_approach_root
        self.within_sequence_dir_subdir = within_sequence_dir_subdir
        self.within_bagdir_sub_dir = within_bagdir_sub_dir
        self.odometry_topic = odometry_topic
        self.force_rerun_interpolator = force_rerun_interpolator
        self.force_rerun_metrics_generator = force_rerun_metrics_generator

        # sequence_dir=args_dict[CmdLineArgConstants.trajectorySequenceFileDirectoryBaseArgName],
        # sequence_file_base_name=args_dict[CmdLineArgConstants.sequenceFileBaseNameBaseArgName],


class TrajectoryMetricsGeneratorParamConstants:
    interpolated_gt_traj_dir = "interpolated_gt_traj_dir"
    lego_loam_frame_to_bl_extrinsics = "lego_loam_frame_to_bl_extrinsics"
    comparison_alg_traj_est_dir = "comparison_alg_traj_est_dir"
    comparison_alg_to_bl_extrinsics = "comparison_alg_to_bl_extrinsics"
    trajectory_results_dir_suffix = "trajectory_results_dir_suffix"
    gt_dir_suffix = "gt_dir_suffix"
    sequence_file = "sequence_file"
    metrics_out_file = "metrics_out_file"
    waypoints_files_directory = "waypoints_files_directory"
    rosbag_files_directory = "rosbag_files_directory"
    odometry_topic = "odometry_topic"
    param_prefix = "param_prefix"


class TrajectoryMetricsGeneratorConfig:
    def __init__(self, interpolated_gt_traj_dir, lego_loam_frame_to_bl_extrinsics, comparison_alg_traj_est_dir,
                 comparison_alg_to_bl_extrinsics, trajectory_results_dir_suffix, gt_dir_suffix, sequence_file,
                 metrics_out_file, waypoints_files_directory,
                 rosbag_files_directory, odometry_topic, param_prefix):
        self.interpolated_gt_traj_dir = interpolated_gt_traj_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_traj_est_dir = comparison_alg_traj_est_dir
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.trajectory_results_dir_suffix = trajectory_results_dir_suffix
        self.gt_dir_suffix = gt_dir_suffix
        self.sequence_file = sequence_file
        self.metrics_out_file = metrics_out_file
        self.waypoints_files_directory = waypoints_files_directory
        self.rosbag_files_directory = rosbag_files_directory
        self.odometry_topic = odometry_topic
        self.param_prefix = param_prefix


def runMetricsGeneratorWithGeneratorConfig(generator_config):
    argsString = ""
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.interpolated_gt_traj_dir,
                                           generator_config.interpolated_gt_traj_dir)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.lego_loam_frame_to_bl_extrinsics,
                                           generator_config.lego_loam_frame_to_bl_extrinsics)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.comparison_alg_traj_est_dir,
                                           generator_config.comparison_alg_traj_est_dir)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.comparison_alg_to_bl_extrinsics,
                                           generator_config.comparison_alg_to_bl_extrinsics)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.trajectory_results_dir_suffix,
                                           generator_config.trajectory_results_dir_suffix)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.gt_dir_suffix,
                                           generator_config.gt_dir_suffix)
    print("Arg for gt dir suffix ")
    print(createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.gt_dir_suffix,
                                   generator_config.gt_dir_suffix))
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.sequence_file,
                                           generator_config.sequence_file)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.metrics_out_file,
                                           generator_config.metrics_out_file)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.waypoints_files_directory,
                                           generator_config.waypoints_files_directory)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.rosbag_files_directory,
                                           generator_config.rosbag_files_directory)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.odometry_topic,
                                           generator_config.odometry_topic)
    argsString += createCommandStrAddition(TrajectoryMetricsGeneratorParamConstants.param_prefix,
                                           generator_config.param_prefix)


    cmdToRun = "./bin/trajectory_metrics_generator " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def generateMetricsForApproach(metrics_for_approach_config):
    # Interpolate ground truth and store results
    # Call trajectory metrics generator

    rosbagsSequence = readTrajectorySequence(metrics_for_approach_config.sequence_dir,
                                             metrics_for_approach_config.sequence_file_base_name)

    interpolatedGtDirs = []
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
    for idx, bagName in enumerate(rosbagsSequence):
        # Run the interpolator
        bagInSeqSubdir = str(idx) + "_" + bagName

        dirForBagResults = dirForSequenceResults + bagInSeqSubdir + "/" + within_bagdir_sub_dir

        interpolation_traj_dir = dirForBagResults
        interpolatedGtDirs.append(interpolation_traj_dir)
        required_timestamps_file_dir = dirForBagResults
        runInterpolator(rosbag_dir=metrics_for_approach_config.rosbag_dir,
                        rosbag_name=bagName,
                        interpolation_traj_dir=interpolation_traj_dir,
                        lego_loam_root_dir=metrics_for_approach_config.lego_loam_root_dir,
                        required_timestamps_file_dir=required_timestamps_file_dir,
                        coarse_trajectory_frame_rel_bl_file=metrics_for_approach_config.lego_loam_frame_to_bl_extrinsics,
                        odom_frame_rel_bl_file=metrics_for_approach_config.odom_frame_rel_bl_file,
                        odometry_topic=metrics_for_approach_config.odometry_topic,
                        forceRerunInterpolator=metrics_for_approach_config.force_rerun_interpolator)

    # metrics_out_file = dirForSequenceResults + "metrics.json"
    metrics_out_file = dirForSequenceResults + "alt_metrics.json"
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
    metrics_generator_config = TrajectoryMetricsGeneratorConfig(
        interpolated_gt_traj_dir=dirForSequenceResults,
        comparison_alg_traj_est_dir=dirForSequenceResults,
        trajectory_results_dir_suffix=within_bagdir_sub_dir,
        gt_dir_suffix=within_bagdir_sub_dir,
        metrics_out_file=metrics_out_file,
        sequence_file=generateSequenceFilePath(metrics_for_approach_config.sequence_dir,
                                               metrics_for_approach_config.sequence_file_base_name),
        lego_loam_frame_to_bl_extrinsics=metrics_for_approach_config.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics=metrics_for_approach_config.comparison_alg_to_bl_extrinsics,
        waypoints_files_directory=metrics_for_approach_config.rosbag_dir,
        rosbag_files_directory=metrics_for_approach_config.rosbag_dir,
        odometry_topic=metrics_for_approach_config.odometry_topic,
        param_prefix=param_prefix)
    runMetricsGeneratorWithGeneratorConfig(metrics_generator_config)


if __name__ == "__main__":
    metricsForApproachConfig = metricsForApproachArgParse()
    generateMetricsForApproach(metricsForApproachConfig)
