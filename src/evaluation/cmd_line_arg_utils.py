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
    numberInSequenceBaseArgName = 'number_in_sequence_arg_name'

    # Boolean arg names
    forceRunOrbSlamPostProcessBaseArgName = 'force_run_orb_post_process'
    outputEllipsoidDebugInfoBaseArgName = 'output_ellipsoid_debug'
    outputJacobianDebugInfoBaseArgName = 'output_jacobian_debug'
    outputBbAssocInfoBaseArgName = 'output_bb_assoc'
    runRvizBaseArgName = 'run_rviz'
    recordVisualizationRosbagBaseArgName = 'record_viz_rosbag'
    logToFileBaseArgName = 'log_to_file'
    outputCheckpointsBaseArgName = 'output_checkpoints'
    readCheckpointsBaseArgName = 'read_checkpoints'

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
    numberInSequenceHelp = "Number of the trajectory in the sequence to look at (0-indexed)"

    forceRunOrbSlamPostProcessHelp = "Force running the orb slam post processor even if there is data there"
    outputEllipsoidDebugInfoHelp = "Output the ellipsoid debug data to file while running"
    outputJacobianDebugInfoHelp = "Output the jacobian debug data to file while running"
    outputBbAssocInfoHelp = "Output the bounding box associations at the end of the trajectory"
    runRvizHelp = "Start the rviz visualization while optimizing a trajectory"
    recordVisualizationRosbagHelp = "Record a rosbag containing the debug visualization messages"
    logToFileHelp = "True if the process should log to file, false if only to standard error"
    outputCheckpointsHelp = "True if data needed to resume optimization mid-way through should be output as the process " \
                            "is running, false if the optimization should "
    readFromCheckpointsHelp = "True if the optimization should resume from current checkpoints if available, false if" \
                              " the optimization should start from the beginning"

    # Constants specific to running lego loam--------------------------------
    legoLoamOutRootDirBaseArgName = 'lego_loam_out_root_dir'
    legoLoamOutRootDirHelp = "Root directory where lego-loam output files should be stored. There will be a " \
                             "subdirectory for each bag"
    forceRunLegoLoamBaseArgName = 'force_run_lego_loam'

    codaParserRepoRootDirBaseArgName = 'coda_parser_repo_root_dir'
    codaParserRepoRootDirHelp = "Directory where the CODa repository is (CODa repository contains the executable that" \
                                " generates point clouds from lidar packets messages)"
    forceRunLegoLoamHelp = "Force running lego loam for the bag even if there is data there"

    # Constants specific to ORBSLAM3/ORBSLAM2
    orbSlamConfigurationFileBaseArgName = 'orb_slam_configuration_file'
    orbSlamConfigurationFileHelp = "Location of the configuration file for orbslam"

    orbSlamVocabularyFileBaseArgName = 'orb_slam_vocabulary_file'
    orbSlamVocabularyFileHelp = "Location of the vocabulary file for orbslam"

    orbSlam3OutRootDirBaseArgName = 'orb_slam_3_out_root_dir'
    orbSlam3OutRootDirHelp = "Root directory where ORB-SLAM3 output files should be stored. There will be a " \
                             "subdirectory for each sequence, and within that, a subdirectory for each bag (prefixed " \
                             "with the number in the sequence)"
    orbSlam2OutRootDirBaseArgName = 'orb_slam_2_out_root_dir'
    orbSlam2OutRootDirHelp = "Root directory where ORB-SLAM2 output files should be stored. There will be a" \
                             " subdirectory for each bag"

    forceRunORBSLAMBaseArgName = 'force_run_orb_slam'
    forceRunORBSLAMHelp = "Force running ORB-SLAM for the sequence even if there is data there"

    generateMapFileBaseArgName = 'generate_map_file'
    generateMapFileHelp = "Generate the ORB-SLAM map file in addition to the raw trajectory at the end of each bag"

    # Metrics
    forceRerunMetricsGeneratorBaseArgName = 'force_rerun_metrics_generator'
    forceRerunMetricsGeneratorHelp = "Force run the metrics generator even if there is already a metrics file"

    forceRerunInterpolatorBaseArgName = "force_rerun_interpolator"
    forceRerunInterpolatorHelp = "Force rerun the interpolator even if the interpolated file is already generated"

    forceRerunMetricsGeneratorBaseArgName = 'force_rerun_metrics_generator'
    forceRerunMetricsGeneratorHelp = "Force run the metrics generator even if there is already a metrics file"

    forceReformatUTVSLAMOutputBaseArgName = "force_rerun_trajectory_formatter"
    forceReformatUTVSLAMOutputHelp = "Force rerun the trajectory formatter that converts from JSON with trajectories " \
                                     "by frame to a csv with trajectories stored by timestamp"

    @staticmethod
    def prefixWithDashDash(argName):
        return '--' + argName


def createCommandStrAddition(argumentName, argumentValue):
    if ((argumentValue is None) or (len(argumentValue) == 0)):
        return ""
    return CmdLineArgConstants.prefixWithDashDash(argumentName) + " " + argumentValue + " "
