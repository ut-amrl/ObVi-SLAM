import os
from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_sequence import *

class MetricsForApproachConstants:
    legoLoamCalibFile = "lego_loam_bl.txt"
    orbslam3CalibFile = "orb_slam3_bl.txt"
    ovslamCalibFile = "ov_slam_bl.txt"

class MetricsForApproachConfig:
    def __init__(self, rosbag_dir,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_root_dir,
                 lego_loam_frame_to_bl_extrinsics,
                 comparison_alg_to_bl_extrinsics,
                 results_for_approach_root,
                 within_sequence_dir_subdir,
                 within_bagdir_sub_dir,
                 force_rerun_interpolator,
                 force_rerun_metrics_generator):
        self.rosbag_dir = rosbag_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_root_dir = lego_loam_root_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.results_for_approach_root = results_for_approach_root
        self.within_sequence_dir_subdir = within_sequence_dir_subdir
        self.within_bagdir_sub_dir = within_bagdir_sub_dir
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


class TrajectoryMetricsGeneratorConfig:
    def __init__(self, interpolated_gt_traj_dir, lego_loam_frame_to_bl_extrinsics, comparison_alg_traj_est_dir,
                 comparison_alg_to_bl_extrinsics, trajectory_results_dir_suffix, gt_dir_suffix, sequence_file,
                 metrics_out_file):
        self.interpolated_gt_traj_dir = interpolated_gt_traj_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.comparison_alg_traj_est_dir = comparison_alg_traj_est_dir
        self.comparison_alg_to_bl_extrinsics = comparison_alg_to_bl_extrinsics
        self.trajectory_results_dir_suffix = trajectory_results_dir_suffix
        self.gt_dir_suffix = gt_dir_suffix
        self.sequence_file = sequence_file
        self.metrics_out_file = metrics_out_file


class InterpolatorParamConstants:
    required_timestamps_file = "required_timestamps_file"
    coarse_trajectory_file = "coarse_trajectory_file"
    rosbag_file = "rosbag_file"
    poses_for_required_timestamps_file = "poses_for_required_timestamps_file"


class InterpolatorConfig:
    def __init__(self, required_timestamps_file, coarse_trajectory_file, rosbag_file,
                 poses_for_required_timestamps_file):
        self.required_timestamps_file = required_timestamps_file
        self.coarse_trajectory_file = coarse_trajectory_file
        self.rosbag_file = rosbag_file
        self.poses_for_required_timestamps_file = poses_for_required_timestamps_file


def runInterpolatorCmd(interpolatorConfig):
    argsString = ""
    argsString += createCommandStrAddition(InterpolatorParamConstants.required_timestamps_file,
                                           interpolatorConfig.required_timestamps_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.coarse_trajectory_file,
                                           interpolatorConfig.coarse_trajectory_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.rosbag_file, interpolatorConfig.rosbag_file)
    argsString += createCommandStrAddition(InterpolatorParamConstants.poses_for_required_timestamps_file,
                                           interpolatorConfig.poses_for_required_timestamps_file)

    cmdToRun = "./bin/interpolate_poses_with_required_nodes " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


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

    cmdToRun = "./bin/trajectory_metrics_generator " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


def runInterpolator(rosbag_dir, rosbag_name, interpolation_traj_dir, lego_loam_root_dir,
                    required_timestamps_file_dir, forceRerunInterpolator):
    poses_for_required_timestamps_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        interpolation_traj_dir) + "lego_loam_poses.csv"
    needToRerunInterpolator = False
    if (forceRerunInterpolator):
        needToRerunInterpolator = True
    elif (not os.path.exists(poses_for_required_timestamps_file)):
        needToRerunInterpolator = True
    if (not needToRerunInterpolator):
        return

    coarse_trajectory_file = FileStructureUtils.ensureDirectoryEndsWithSlash(
        lego_loam_root_dir) + rosbag_name + "/poses/lego_loam_poses.csv"
    interpolatorConfig = InterpolatorConfig(
        required_timestamps_file=(
                FileStructureUtils.ensureDirectoryEndsWithSlash(required_timestamps_file_dir) + "trajectory.csv"),
        coarse_trajectory_file=coarse_trajectory_file,
        rosbag_file=(FileStructureUtils.ensureDirectoryEndsWithSlash(rosbag_dir) + \
                     rosbag_name + FileStructureConstants.bagSuffix),
        poses_for_required_timestamps_file=poses_for_required_timestamps_file)
    runInterpolatorCmd(interpolatorConfig)


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


        dirForBagResults = dirForSequenceResults + bagInSeqSubdir + within_bagdir_sub_dir

        interpolation_traj_dir = dirForBagResults
        interpolatedGtDirs.append(interpolation_traj_dir)
        required_timestamps_file_dir = dirForBagResults
        runInterpolator(rosbag_dir=metrics_for_approach_config.rosbag_dir,
                        rosbag_name=bagName,
                        interpolation_traj_dir=interpolation_traj_dir,
                        lego_loam_root_dir=metrics_for_approach_config.lego_loam_root_dir,
                        required_timestamps_file_dir=required_timestamps_file_dir,
                        forceRerunInterpolator=metrics_for_approach_config.force_rerun_interpolator)

    # class TrajectoryMetricsGeneratorConfig:
    #     def __init__(self, interpolated_gt_traj_dir, lego_loam_frame_to_bl_extrinsics, comparison_alg_traj_est_dir,
    #                  comparison_alg_to_bl_extrinsics, trajectory_results_dir_suffix, gt_dir_suffix, sequence_file,
    #                  metrics_out_file):
    #         self.interpolated_gt_traj_dir = interpolated_gt_traj_dir
    #         self.comparison_alg_traj_est_dir = comparison_alg_traj_est_dir
    #         self.trajectory_results_dir_suffix = trajectory_results_dir_suffix
    #         self.gt_dir_suffix = gt_dir_suffix

    metrics_out_file = dirForSequenceResults + "metrics.json"
    needToRerunMetrics = False
    if (metrics_for_approach_config.force_rerun_metrics_generator):
        needToRerunMetrics = True
    elif (not os.path.exists(metrics_out_file)):
        needToRerunMetrics = True
    if (not needToRerunMetrics):
        return

    metrics_generator_config = TrajectoryMetricsGeneratorConfig(
        interpolated_gt_traj_dir=dirForSequenceResults,
        comparison_alg_traj_est_dir=dirForSequenceResults,
        trajectory_results_dir_suffix=within_bagdir_sub_dir,
        gt_dir_suffix=within_bagdir_sub_dir,
        metrics_out_file=metrics_out_file,
        sequence_file=generateSequenceFilePath(metrics_for_approach_config.sequence_dir,
                                               metrics_for_approach_config.sequence_file_base_name),
        lego_loam_frame_to_bl_extrinsics=metrics_for_approach_config.lego_loam_frame_to_bl_extrinsics,
        comparison_alg_to_bl_extrinsics = metrics_for_approach_config.comparison_alg_to_bl_extrinsics)
    runMetricsGeneratorWithGeneratorConfig(metrics_generator_config)


# def metricsForApproachArgParse():
#     return MetricsForApproachConfig()


if __name__ == "__main__":
    metricsForApproachConfig = metricsForApproachArgParse()
    generateMetricsForApproach(metricsForApproachConfig)
