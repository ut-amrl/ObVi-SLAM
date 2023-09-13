from cmd_line_arg_utils import *
from file_structure_utils import *
from trajectory_interpolation import *
from trajectory_sequence import *


class LegoLOAMOverlayConfig:
    def __init__(self, rosbag_dir,
                 sequence_dir,
                 sequence_file_base_name,
                 lego_loam_root_dir,
                 lego_loam_out_dir,
                 lego_loam_frame_to_bl_extrinsics,
                 odometry_topic):
        self.rosbag_dir = rosbag_dir
        self.sequence_dir = sequence_dir
        self.sequence_file_base_name = sequence_file_base_name
        self.lego_loam_root_dir = lego_loam_root_dir
        self.lego_loam_out_dir = lego_loam_out_dir
        self.lego_loam_frame_to_bl_extrinsics = lego_loam_frame_to_bl_extrinsics
        self.odometry_topic = odometry_topic


class LegoLOAMOverlayConstants:
    lego_loam_frame_to_bl_extrinsics = "lego_loam_frame_to_bl_extrinsics"
    lego_loam_root_dir = "lego_loam_root_dir"
    lego_loam_out_dir = "lego_loam_out_dir"
    sequence_file = "sequence_file"
    waypoints_files_directory = "waypoints_files_directory"
    rosbag_files_directory = "rosbag_files_directory"
    odometry_topic = "odometry_topic"
    param_prefix = "param_prefix"


def runOverlay(config):
    paramPrefix = None
    sequenceFile = generateSequenceFilePath(config.sequence_dir,
                                            config.sequence_file_base_name)

    argsString = ""
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.lego_loam_frame_to_bl_extrinsics,
                                           config.lego_loam_frame_to_bl_extrinsics)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.lego_loam_root_dir,
                                           config.lego_loam_root_dir)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.lego_loam_out_dir,
                                           config.lego_loam_out_dir)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.sequence_file,
                                           sequenceFile)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.waypoints_files_directory,
                                           config.rosbag_dir)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.rosbag_files_directory,
                                           config.rosbag_dir)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.odometry_topic,
                                           config.odometry_topic)
    argsString += createCommandStrAddition(LegoLOAMOverlayConstants.param_prefix,
                                           paramPrefix)

    cmdToRun = "./bin/lego_loam_overlay_generator " + argsString
    print("Running command: ")
    print(cmdToRun)
    os.system(cmdToRun)


if __name__ == "__main__":
    # Ideally get this from args, but for now, easier to code it here
    root_data_dir = "/home/amanda/rosbags/ellipsoid_slam/eer_jackal/"
    rosbag_file_directory = root_data_dir + "original_data/"
    trajectory_sequence_file_directory = "/home/amanda/workspaces/ut_vslam/sequences/"

    lego_loam_out_root_dir = root_data_dir + "lego_loam_out/"
    calibration_file_directory = root_data_dir + "calibration/"

    odometry_topic = "/jackal_velocity_controller/odom"
    
    overlayPlotterConfig = LegoLOAMOverlayConfig(rosbag_dir=rosbag_file_directory,
                                                 sequence_dir=trajectory_sequence_file_directory,
                                                 sequence_file_base_name="evaluation_2023_07_v1",
                                                 lego_loam_root_dir=lego_loam_out_root_dir,
                                                 lego_loam_out_dir=(root_data_dir + "lego_loam_formatted_out/"),
                                                 lego_loam_frame_to_bl_extrinsics=(
                                                         calibration_file_directory + CalibrationFileConstants.legoLoamCalibFile),
                                                 odometry_topic=odometry_topic)
    runOverlay(overlayPlotterConfig)
