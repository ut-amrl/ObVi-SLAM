#include <analysis/cumulative_timer_constants.h>
#include <analysis/cumulative_timer_factory.h>
#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_timestamp_io.h>
#include <file_io/camera_info_io_utils.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

const std::string kCeresOptInfoLogFile = "ceres_opt_summary.csv";

typedef vtr::IndependentEllipsoidsLongTermObjectMap<
    //    std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
    util::EmptyStruct>
    MainLtm;
typedef std::shared_ptr<MainLtm> MainLtmPtr;
typedef vtr::UnassociatedBoundingBoxOfflineProblemData<
    vtr::StructuredVisionFeatureTrack,
    sensor_msgs::Image::ConstPtr,
    MainLtm>
    MainProbData;
typedef vtr::ObjectAndReprojectionFeaturePoseGraph MainPg;
typedef std::shared_ptr<MainPg> MainPgPtr;
typedef std::pair<vslam_types_refactor::FactorType,
                  vslam_types_refactor::FeatureFactorId>
    MainFactorInfo;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(bounding_boxes_by_timestamp_out_file,
              "",
              "File with bounding box observations by timestamp");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(params_config_file, "", "config file containing tunable params");

std::vector<file_io::BoundingBoxWithTimestamp> getBoundingBoxesForAllImages(
    const std::string &rosbag_file_name,
    const std::unordered_map<std::string, vtr::CameraId>
        &camera_topic_to_camera_id,
    vtr::YoloBoundingBoxQuerier &bb_querier) {
  // Read the images
  rosbag::Bag bag;
  bag.open(rosbag_file_name, rosbag::bagmode::Read);

  std::vector<file_io::BoundingBoxWithTimestamp> bounding_boxes_with_timestamp;

  std::vector<std::string> topics;
  for (const auto &camera_topic_and_id : camera_topic_to_camera_id) {
    topics.emplace_back(camera_topic_and_id.first);
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::unordered_map<vslam_types_refactor::FrameId,
                     std::unordered_map<vslam_types_refactor::CameraId,
                                        sensor_msgs::Image::ConstPtr>>
      images_by_frame_and_cam;
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(image_utils::kCompressedImageSuffix) !=
        std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }

    std::vector<vtr::RawBoundingBox> bounding_boxes_for_image;
    if (!bb_querier.retrieveBoundingBoxesForImage(msg,
                                                  bounding_boxes_for_image)) {
      LOG(ERROR) << "YOLO query failed for topic " << m.getTopic()
                 << " at timestamp " << msg->header.stamp;
      exit(1);
    }

    pose::Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);

    vtr::CameraId cam = camera_topic_to_camera_id.at(m.getTopic());

    for (const vtr::RawBoundingBox &bb_for_img : bounding_boxes_for_image) {
      file_io::BoundingBoxWithTimestamp bb;
      bb.seconds = img_timestamp.first;
      bb.nano_seconds = img_timestamp.second;
      bb.camera_id = cam;
      bb.semantic_class = bb_for_img.semantic_class_;
      bb.detection_confidence = bb_for_img.detection_confidence_;
      bb.min_pixel_x = bb_for_img.pixel_corner_locations_.first.x();
      bb.min_pixel_y = bb_for_img.pixel_corner_locations_.first.y();
      bb.max_pixel_x = bb_for_img.pixel_corner_locations_.second.x();
      bb.max_pixel_y = bb_for_img.pixel_corner_locations_.second.y();
      bounding_boxes_with_timestamp.emplace_back(bb);
    }
  }
  return bounding_boxes_with_timestamp;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::optional<vtr::OptimizationLogger> opt_logger;
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(WARNING) << "No rosbag file provided";
  }

  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, "a_" + node_prefix + "bounding_box_writer");
  ros::NodeHandle node_handle;

  vtr::FullOVSLAMConfig config;
  vtr::readConfiguration(FLAGS_params_config_file, config);

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  std::vector<file_io::BoundingBoxWithTimestamp> bounding_boxes_with_timestamp =
      getBoundingBoxesForAllImages(
          FLAGS_rosbag_file,
          config.camera_info_.camera_topic_to_camera_id_,
          bb_querier);

  file_io::writeBoundingBoxWithTimestampsToFile(
      FLAGS_bounding_boxes_by_timestamp_out_file,
      bounding_boxes_with_timestamp);

  return 0;
}