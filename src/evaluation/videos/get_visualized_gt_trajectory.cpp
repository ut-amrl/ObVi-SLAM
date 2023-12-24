#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <debugging/ground_truth_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/bounding_box_by_timestamp_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_info_io_utils.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/object_and_reprojection_feature_pose_graph_file_storage_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <file_io/pose_io_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visual_feature_frontend/visual_feature_front_end.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <refactoring/visualization/ros_visualization.h>
#include <refactoring/visualization/save_to_file_visualizer.h>
#include <ros/ros.h>
#include <run_optimization_utils/optimization_runner.h>
#include <sensor_msgs/Image.h>

using namespace pose;
using namespace vslam_types_refactor;
namespace vtr = vslam_types_refactor;

DEFINE_string(ground_truth_trajectory_file,
              "",
              "File containing ground truth for the trajectory");
DEFINE_string(ground_truth_extrinsics_file,
              "",
              "File containing the "
              "extrinsics that relate the ground truth trajectory frame to the"
              " frame that is estimated here.");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(output_file,
              "",
              "File to which to output the interpolated poses "
              "for all timestamps (i.e. also contains smoothed odom)");


int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
  file_io::readNodeIdsAndTimestampsFromFile(FLAGS_nodes_by_timestamp_file,
                                            nodes_by_timestamps_vec);

  std::optional<std::vector<vtr::Pose3D<double>>> gt_trajectory =
    vtr::getGtTrajectory(nodes_by_timestamps_vec.size()-1,
                         FLAGS_ground_truth_trajectory_file,
                         FLAGS_ground_truth_extrinsics_file,
                         FLAGS_nodes_by_timestamp_file);

  std::vector<std::pair<Timestamp, Pose3D<double>>> stamped_poses;
  for (int i = 0; i < nodes_by_timestamps_vec.size(); ++i) {
    stamped_poses.emplace_back(std::make_pair(nodes_by_timestamps_vec.at(i).seconds_, 
                                              nodes_by_timestamps_vec.at(i).nano_seconds_),
                               gt_trajectory.value().at(i));
    

  }

  LOG(INFO) << "nodes_by_timestamps_vec size: " << nodes_by_timestamps_vec.size();
  LOG(INFO) << "gt_trajectory size: " << gt_trajectory.value().size();
  LOG(INFO) << "stamped_poses size: " << stamped_poses.size();

  if (!FLAGS_output_file.empty()) {
    LOG(INFO) << "Writing to " + FLAGS_output_file;
    file_io::writePose3dsWithTimestampToFile(
        FLAGS_output_file, stamped_poses);
  }
}