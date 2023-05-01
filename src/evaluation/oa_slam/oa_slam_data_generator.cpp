#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <ros/ros.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

namespace vtr = vslam_types_refactor;

DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(oa_slam_data_output_directory,
              "",
              "Output directory for OA-SLAM Data");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::string node_prefix = FLAGS_param_prefix;

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(ERROR) << "No nodes by timestamp file";
    exit(1);
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(ERROR) << "No rosbag file provided";
    exit(1);
  }

  ros::init(argc, argv, "a_" + node_prefix + "ellipsoid_estimator_real_data");
  ros::NodeHandle node_handle;

  LOG(INFO) << "Reading images from rosbag";
  const std::unordered_map<std::string, vslam_types_refactor::CameraId>
        camera_topics_to_camera_ids = {{"/zed/zed_node/right/image_rect_color", 2}, 
                                       {"/zed/zed_node/right/image_rect_color/compressed", 2}, 
                                       {"/zed/zed_node/left/image_rect_color", 1}, 
                                       {"/zed/zed_node/left/image_rect_color/compressed", 1}};
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(
          FLAGS_rosbag_file,
          FLAGS_nodes_by_timestamp_file,
          camera_topics_to_camera_ids);
  LOG(INFO) << "Done reading images from rosbag";

  

  return 0;
}