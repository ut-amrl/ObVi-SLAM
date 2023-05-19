#include <file_io/node_id_and_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

namespace vtr = vslam_types_refactor;
namespace fs = std::filesystem;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(oa_slam_data_output_directory,
              "test",
              "Output directory for OA-SLAM Data");

void setupOutputDirectory(const std::string &output_dir,
                          const bool clean = false) {
  if (!fs::is_directory(output_dir) || !fs::exists(output_dir)) {
    if (!fs::create_directory(output_dir)) {
      LOG(FATAL) << "failed to create directory " << output_dir;
    }
  }
  if (clean) {
    for (const auto &entry : fs::directory_iterator(output_dir)) {
      fs::remove_all(entry.path());
    }
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  const std::unordered_map<std::string, vslam_types_refactor::CameraId>
      camera_topics_to_camera_ids = {
          {"/zed/zed_node/right/image_rect_color", 2},
          {"/zed/zed_node/right/image_rect_color/compressed", 2},
          {"/zed2i/zed_node/right/image_rect_color", 2},
          {"/zed2i/zed_node/right/image_rect_color/compressed", 2},
          {"/zed/zed_node/left/image_rect_color", 1},
          {"/zed/zed_node/left/image_rect_color/compressed", 1},
          {"/zed2i/zed_node/left/image_rect_color", 1},
          {"/zed2i/zed_node/left/image_rect_color/compressed", 1}};
  const std::unordered_map<std::string, size_t> class_names_and_class_ids = {
      {"trashcan", 0}, {"lamppost", 1}, {"treetrunk", 2}, {"bench", 3}};

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(ERROR) << "No nodes by timestamp file";
    exit(1);
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(ERROR) << "No rosbag file provided";
    exit(1);
  }

  if (FLAGS_oa_slam_data_output_directory.empty()) {
    LOG(ERROR) << "No OA-SLAM output directory provided";
    exit(1);
  }

  LOG(INFO) << "Initializing nodes";
  const std::string node_prefix = FLAGS_param_prefix;
  ros::init(argc, argv, "a_" + node_prefix + "oa_slam_data_generator");
  ros::NodeHandle node_handle;

  LOG(INFO) << "Reading images from rosbag";
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(FLAGS_rosbag_file,
                                                FLAGS_nodes_by_timestamp_file,
                                                camera_topics_to_camera_ids);
  LOG(INFO) << "Done reading images from rosbag";

  std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
  file_io::readNodeIdsAndTimestampsFromFile(FLAGS_nodes_by_timestamp_file,
                                            nodes_by_timestamps_vec);

  fs::path root_output_directory(FLAGS_oa_slam_data_output_directory);
  setupOutputDirectory(FLAGS_oa_slam_data_output_directory, true);
  std::unordered_set<vtr::CameraId> camera_ids = {1, 2};
  std::unordered_map<vtr::CameraId, fs::path>
      cam_ids_and_image_output_directories;
  for (const vtr::CameraId cam_id : camera_ids) {
    cam_ids_and_image_output_directories[cam_id] =
        root_output_directory / std::to_string(cam_id);
    setupOutputDirectory(cam_ids_and_image_output_directories.at(cam_id));
  }

  std::unordered_map<vtr::CameraId, std::ofstream> cam_ids_and_ofiles;
  for (const vtr::CameraId cam_id : camera_ids) {
    fs::path ofile_path = cam_ids_and_image_output_directories.at(cam_id) /
                          ("cam_" + std::to_string(cam_id) + "_images.txt");
    cam_ids_and_ofiles[cam_id].open(ofile_path, std::ios::trunc);
    if (!cam_ids_and_ofiles.at(cam_id).is_open()) {
      LOG(ERROR) << "Failed to open " << ofile_path;
      exit(1);
    }
  }

  std::unordered_map<vtr::CameraId, std::ofstream> cam_ids_and_detection_ofiles;
  for (const vtr::CameraId cam_id : camera_ids) {
    fs::path ofile_path =
        cam_ids_and_image_output_directories.at(cam_id) / "detections";
    setupOutputDirectory(ofile_path, false);
    ofile_path /= "detections.txt";
    cam_ids_and_detection_ofiles[cam_id].open(ofile_path, std::ios::trunc);
    if (!cam_ids_and_detection_ofiles.at(cam_id).is_open()) {
      LOG(ERROR) << "Failed to open " << ofile_path;
      exit(1);
    }
  }

  std::vector<vtr::FrameId> ordered_frame_ids;
  for (const auto &frame_id_and_image_info : images) {
    ordered_frame_ids.push_back(frame_id_and_image_info.first);
  }
  std::sort(ordered_frame_ids.begin(), ordered_frame_ids.end());

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  for (const auto frame_id : ordered_frame_ids) {
    const auto &cam_ids_and_images = images[frame_id];
    const auto &node_and_timestamp = nodes_by_timestamps_vec.at(frame_id);
    for (const auto &cam_id_and_image : cam_ids_and_images) {
      fs::path image_path =
          cam_ids_and_image_output_directories.at(cam_id_and_image.first) /
          (std::to_string(frame_id) + ".png");
      fs::path rel_image_path =
          fs::path(std::to_string(cam_id_and_image.first)) /
          (std::to_string(frame_id) + ".png");
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(cam_id_and_image.second,
                                     sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
        exit(1);
      }
      cv::imwrite(image_path, cv_ptr->image);
      cam_ids_and_ofiles.at(cam_id_and_image.first)
          << rel_image_path.string() << std::endl;
    }

    std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>
        cam_ids_and_bboxes;
    bb_querier.retrieveBoundingBoxesFromCamIdsAndImages(
        frame_id, cam_ids_and_images, cam_ids_and_bboxes);
    for (const auto &cam_id_and_bboxes : cam_ids_and_bboxes) {
      fs::path rel_image_path =
          fs::path(std::to_string(cam_id_and_bboxes.first)) /
          (std::to_string(frame_id) + ".png");
      std::ofstream &ofile =
          cam_ids_and_detection_ofiles.at(cam_id_and_bboxes.first);
      if (node_and_timestamp.node_id_ != frame_id) {
        LOG(ERROR) << "Something went wrong with node id and frame id! "
                   << "node id = " << node_and_timestamp.node_id_
                   << "; frame id = " << frame_id;
        exit(1);
      }
      ofile << "file_name" << std::endl;
      // ofile << node_and_timestamp.seconds_ << "_" <<
      // node_and_timestamp.nano_seconds_ << ".png" << std::endl; ofile <<
      // std::to_string(frame_id) << ".png" << std::endl;
      ofile << rel_image_path.string() << std::endl;
      ofile << "detections" << std::endl;
      for (const auto &bbox : cam_id_and_bboxes.second) {
        ofile << "category_id" << std::endl;
        ofile << class_names_and_class_ids.at(bbox.semantic_class_)
              << std::endl;
        ofile << "detection_score" << std::endl;
        ofile << bbox.detection_confidence_ << std::endl;
        ofile << "bbox" << std::endl;
        // min_x, min_y, max_x, max_y
        ofile << bbox.pixel_corner_locations_.first.x() << ","
              << bbox.pixel_corner_locations_.first.y() << ","
              << bbox.pixel_corner_locations_.second.x() << ","
              << bbox.pixel_corner_locations_.second.y() << std::endl;
      }
      ofile << std::endl;
    }
  }

  for (auto &cam_id_and_ofile : cam_ids_and_ofiles) {
    cam_id_and_ofile.second.close();
  }
  for (auto &cam_id_and_detection_ofile : cam_ids_and_detection_ofiles) {
    cam_id_and_detection_ofile.second.close();
  }

  return 0;
}