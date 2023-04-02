#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/cv_file_storage/config_file_storage_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/roshan_bounding_box_front_end_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end_creation_utils.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/configuration/full_ov_slam_config.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/offline/pose_graph_frame_data_adder.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visual_feature_frontend/visual_feature_front_end.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <refactoring/visualization/ceres_callback.h>
#include <refactoring/visualization/ros_visualization.h>
#include <refactoring/visualization/save_to_file_visualizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <visualization/matplotlibcpp.h>

#include <filesystem>
namespace plt = matplotlibcpp;

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

DEFINE_int32(target_frame_id, -1, "");
DEFINE_string(debug_output_directory,
              "",
              "/robodata/taijing/object-slam/vslam/debug/tmp");
DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
DEFINE_string(bounding_boxes_by_node_id_file,
              "",
              "File with bounding box observations by node id");
DEFINE_string(poses_by_node_id_file,
              "",
              "File with initial robot pose estimates");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(
    long_term_map_input,
    "",
    "File name that stores the long-term map to load. If empty, will start "
    "from scratch.");
DEFINE_string(long_term_map_output,
              "",
              "File name to output the long-term map to.");
DEFINE_double(min_confidence, 0.2, "Minimum confidence");
DEFINE_string(low_level_feats_dir,
              "",
              "Directory that contains low level features");
DEFINE_string(vslam_debugger_directory,
              "/robodata/taijing/object-slam/vslam/debug/1668019589/",
              "Output root directory for debugger");
DEFINE_string(bb_associations_out_file,
              "",
              "File to write ellipsoid results and associated bounding boxes "
              "to. Skipped if this param is not set");
DEFINE_string(ltm_opt_jacobian_info_directory,
              "",
              "Directory to write jacobian info from the LTM optimization for");
DEFINE_string(visual_feature_results_file, "", "Visual feature results");
DEFINE_string(debug_images_output_directory,
              "",
              "Directory to output debug images to. If not specified, no debug "
              "images saved");
DEFINE_string(params_config_file, "", "config file containing tunable params");
DEFINE_string(ellipsoids_results_file, "", "File for ellipsoids results");
DEFINE_string(robot_poses_results_file, "", "File for robot pose results");
DEFINE_string(logs_directory,
              "",
              "If specified, where logs are written (in addition to stderr)");
DEFINE_string(ground_truth_trajectory_file,
              "",
              "File containing ground truth for the trajectory");
DEFINE_string(ground_truth_extrinsics_file,
              "",
              "File containing the "
              "extrinsics that relate the ground truth trajectory frame to the"
              " frame that is estimated here.");

namespace fs = std::filesystem;

enum DebugTypeEnum {
  INITIALIZED,
  BEFORE_OPTIM,
  AFTER_OPTIM,
  ORBSLAM,
  OBSERVED,
  ALL,
  NONE
};

std_msgs::ColorRGBA ColorRGBA_init(double const r,
                                   double const g,
                                   const double b,
                                   double const a) {
  std_msgs::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

double alpha = 0.8;
std_msgs::ColorRGBA obs_color = ColorRGBA_init(0, 1, 0, alpha);
std_msgs::ColorRGBA init_color = ColorRGBA_init(1, 1, 0, alpha);
std_msgs::ColorRGBA before_optim_color = ColorRGBA_init(1, 0, 0, alpha);
std_msgs::ColorRGBA after_optim_color = ColorRGBA_init(0, 0, 1, alpha);

std::unordered_map<DebugTypeEnum, std_msgs::ColorRGBA> types_and_ros_colors = {
    {DebugTypeEnum::OBSERVED, obs_color},
    {DebugTypeEnum::INITIALIZED, init_color},
    {DebugTypeEnum::BEFORE_OPTIM, before_optim_color},
    {DebugTypeEnum::AFTER_OPTIM, after_optim_color}};

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

std::tuple<cv_bridge::CvImagePtr, std::map<double, vtr::FeatureId>>
getFeatureReprojections(
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &feat_ids_and_features3d,
    const std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        &feat_ids_and_features2d,
    const vtr::Pose3D<double> &pose,
    const sensor_msgs::Image::ConstPtr &img_msg,
    const vtr::CameraExtrinsics<double> &extrinsics,
    const vtr::CameraIntrinsicsMat<double> &intrinsics) {
  std::function<double(const vtr::PixelCoord<double> &,
                       const vtr::PixelCoord<double> &)>
      getResidual = [&](const vtr::PixelCoord<double> &pixel1,
                        const vtr::PixelCoord<double> &pixel2) {
        return (pixel1 - pixel2).norm();
      };
  std::map<double, vtr::FeatureId> residuals_and_feat_ids;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, vtr::kImageEncoding);
  } catch (cv_bridge::Exception &e) {
    LOG(ERROR) << "cv_bridge exception: " << e.what();
    exit(1);
  }

  for (const auto feat_id_and_feature2d : feat_ids_and_features2d) {
    const vtr::FeatureId feat_id = feat_id_and_feature2d.first;
    if (feat_ids_and_features3d.find(feat_id) ==
        feat_ids_and_features3d.end()) {
      continue;
    }
    const auto &obs_pixel = feat_id_and_feature2d.second;
    vtr::PixelCoord<double> projected_pixel = vtr::getProjectedPixelCoord(
        feat_ids_and_features3d.at(feat_id), pose, extrinsics, intrinsics);
    vtr::RosVisualization::drawTinyCircleOnImage(
        obs_pixel, types_and_ros_colors[DebugTypeEnum::OBSERVED], cv_ptr);
    vtr::RosVisualization::drawTinyCircleOnImage(
        projected_pixel,
        types_and_ros_colors[DebugTypeEnum::INITIALIZED],
        cv_ptr);
    vtr::RosVisualization::drawLineOnImage(
        obs_pixel, projected_pixel, types_and_ros_colors[INITIALIZED], cv_ptr);
    residuals_and_feat_ids[getResidual(projected_pixel, obs_pixel)] = feat_id;
  }
  return std::make_tuple(cv_ptr, residuals_and_feat_ids);
}

std::unordered_map<
    vtr::CameraId,
    std::tuple<cv_bridge::CvImagePtr, std::map<double, vtr::FeatureId>>>
getFeatureReprojections(
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &feat_ids_and_features3d,
    const std::unordered_map<
        vtr::CameraId,
        std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
        &cam_ids_and_observations,
    const vtr::Pose3D<double> &pose,
    const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>
        &cam_ids_and_images,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &cam_ids_and_extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &cam_ids_and_intrinsics,
    const std::vector<vtr::CameraId> &camera_ids) {
  std::unordered_map<
      vtr::CameraId,
      std::tuple<cv_bridge::CvImagePtr, std::map<double, vtr::FeatureId>>>
      cam_ids_and_debug_info;
  for (const auto cam_id : camera_ids) {
    if (cam_ids_and_intrinsics.find(cam_id) == cam_ids_and_intrinsics.end()) {
      LOG(ERROR) << "Cannot find intrinsics for camera " << cam_id;
      continue;
    }
    if (cam_ids_and_extrinsics.find(cam_id) == cam_ids_and_extrinsics.end()) {
      LOG(ERROR) << "Cannot find extrinsics for camera " << cam_id;
      continue;
    }
    if (cam_ids_and_images.find(cam_id) == cam_ids_and_images.end()) {
      LOG(ERROR) << "Cannot find image message for camera " << cam_id;
      continue;
    }
    if (cam_ids_and_observations.find(cam_id) ==
        cam_ids_and_observations.end()) {
      LOG(ERROR) << "Cannot find visual feature observations for camera "
                 << cam_id;
      continue;
    }
    // LOG(INFO) << "before single image getFeatureReprojections";
    cam_ids_and_debug_info[cam_id] =
        getFeatureReprojections(feat_ids_and_features3d,
                                cam_ids_and_observations.at(cam_id),
                                pose,
                                cam_ids_and_images.at(cam_id),
                                cam_ids_and_extrinsics.at(cam_id),
                                cam_ids_and_intrinsics.at(cam_id));
    // LOG(INFO) << "after single image getFeatureReprojections";
  }
  // LOG(INFO) << "returning from multi image getFeatureReprojections";
  return cam_ids_and_debug_info;
}

void debugInitVisualFeaturesByFrameId(
    const fs::path &root_directory,
    const vtr::FrameId &frame_id,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &cam_ids_and_intrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &cam_ids_and_extrinsics,
    const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>
        &cam_ids_and_images,
    const vtr::Pose3D<double> &pose,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &initial_feat_positions,
    const std::unordered_map<
        vtr::CameraId,
        std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
        &cam_ids_and_observations,
    const std::string &directory_prefix = "frames",
    const std::vector<vtr::CameraId> &camera_ids = {1, 2}) {
  std::unordered_map<vtr::CameraId, fs::path> cam_ids_and_output_directories;
  fs::path output_directory = root_directory / directory_prefix;
  setupOutputDirectory(output_directory);
  for (const vtr::CameraId cam_id : camera_ids) {
    cam_ids_and_output_directories[cam_id] =
        output_directory / std::to_string(cam_id);
    setupOutputDirectory(cam_ids_and_output_directories.at(cam_id));
  }

  // LOG(INFO) << "before multi images getFeatureReprojections";
  auto cam_ids_and_debug_info =
      getFeatureReprojections(initial_feat_positions,
                              cam_ids_and_observations,
                              pose,
                              cam_ids_and_images,
                              cam_ids_and_extrinsics,
                              cam_ids_and_intrinsics,
                              camera_ids);
  // LOG(INFO) << "after multi images getFeatureReprojections";
  for (const auto cam_id_and_debug_info : cam_ids_and_debug_info) {
    const vtr::CameraId cam_id = cam_id_and_debug_info.first;
    const auto &debug_info = cam_id_and_debug_info.second;
    const cv_bridge::CvImagePtr &cv_ptr = std::get<0>(debug_info);
    const std::map<double, vtr::FeatureId> &residuals_and_feat_ids =
        std::get<1>(debug_info);

    fs::path output_path = cam_ids_and_output_directories.at(cam_id) /
                           (std::to_string(frame_id) + ".png");
    cv::imwrite(output_path, cv_ptr->image);

    bool print_residuals = true;
    if (print_residuals) {
      for (const auto &residual_and_feat_id : residuals_and_feat_ids) {
        LOG(INFO) << "Feature " << residual_and_feat_id.second << ": "
                  << residual_and_feat_id.first;
      }
    }
  }
}

void debugInitVisualFeaturesByFeatureIds(
    const fs::path &root_directory,
    const std::unordered_set<vtr::FeatureId> &target_feat_ids,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &cam_ids_and_intrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &cam_ids_and_extrinsics,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
        &frame_ids_and_images,
    const std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> &poses,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &initial_feat_positions,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>>
        &frame_ids_and_observations,
    const std::string &directory_prefix = "features",
    const std::vector<vtr::CameraId> &camera_ids = {1, 2}) {
  std::unordered_map<vtr::CameraId, fs::path> cam_ids_and_output_directories;
  fs::path output_directory = root_directory / directory_prefix;
  setupOutputDirectory(output_directory);
  for (const vtr::CameraId cam_id : camera_ids) {
    cam_ids_and_output_directories[cam_id] =
        output_directory / std::to_string(cam_id);
    setupOutputDirectory(cam_ids_and_output_directories.at(cam_id));
  }

  for (const auto &frame_id_and_observations : frame_ids_and_observations) {
    // LOG(INFO) << "start one iteration";
    const vtr::FrameId frame_id = frame_id_and_observations.first;
    const auto &cam_ids_and_observations = frame_id_and_observations.second;
    if (frame_ids_and_images.find(frame_id) == frame_ids_and_images.end()) {
      LOG(ERROR) << "Cannot find images for frame " << frame_id;
      continue;
    }
    const auto &cam_ids_and_images = frame_ids_and_images.at(frame_id);
    if (poses.find(frame_id) == poses.end()) {
      LOG(ERROR) << "Cannot find pose for frame " << frame_id;
      continue;
    }
    const auto &pose = poses.at(frame_id);
    if (frame_ids_and_observations.find(frame_id) ==
        frame_ids_and_observations.end()) {
      LOG(ERROR) << "Cannot find observations for frame " << frame_id;
      continue;
    }
    const auto &feat_ids_and_observations =
        frame_ids_and_observations.at(frame_id);

    // LOG(INFO) << "before building target_cam_ids_and_observations";
    std::unordered_map<
        vtr::CameraId,
        std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
        target_cam_ids_and_observations;
    for (const auto &cam_id_and_observations : cam_ids_and_observations) {
      const vtr::CameraId cam_id = cam_id_and_observations.first;
      const auto &feat_ids_and_pixels = cam_id_and_observations.second;
      for (const auto feat_id_and_pixel : feat_ids_and_pixels) {
        if (target_feat_ids.find(feat_id_and_pixel.first) !=
            target_feat_ids.end()) {
          target_cam_ids_and_observations[cam_id][feat_id_and_pixel.first] =
              feat_id_and_pixel.second;
        }
      }
    }

    // LOG(INFO) << "before deciding visualization";
    bool visualize = false;
    if (target_cam_ids_and_observations.size() != 0) {
      for (const vtr::CameraId cam_id : camera_ids) {
        visualize |= (target_cam_ids_and_observations.at(cam_id).size() != 0);
      }
    }
    // LOG(INFO) << "before visualization";
    if (visualize) {
      // LOG(INFO) << "visualizing... Before getFeatureReprojections";
      std::unordered_map<
          vtr::CameraId,
          std::tuple<cv_bridge::CvImagePtr, std::map<double, vtr::FeatureId>>>
          ret = getFeatureReprojections(initial_feat_positions,
                                        target_cam_ids_and_observations,
                                        pose,
                                        cam_ids_and_images,
                                        cam_ids_and_extrinsics,
                                        cam_ids_and_intrinsics,
                                        camera_ids);
      // LOG(INFO) << "visualizing... After getFeatureReprojections";
      for (const auto &cam_id_and_debug_info : ret) {
        const vtr::CameraId cam_id = cam_id_and_debug_info.first;
        const cv_bridge::CvImagePtr cv_ptr =
            std::get<0>(cam_id_and_debug_info.second);
        fs::path output_path = cam_ids_and_output_directories.at(cam_id) /
                               (std::to_string(frame_id) + ".png");
        cv::imwrite(output_path, cv_ptr->image);
      }
      // LOG(INFO) << "saving images";
    }
  }
}

std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
readCameraIntrinsicsByCameraFromFile(const std::string &file_name) {
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_by_cam_id;
  file_io::readCameraIntrinsicsWithIdsFromFile(file_name,
                                               camera_intrinsics_by_cam_id);
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      cam_intrinsics_map;
  for (const file_io::CameraIntrinsicsWithId &intrinsics_for_cam :
       camera_intrinsics_by_cam_id) {
    vtr::CameraIntrinsicsMat<double> intrinsics_mat;
    intrinsics_mat << intrinsics_for_cam.mat_00, intrinsics_for_cam.mat_01,
        intrinsics_for_cam.mat_02, intrinsics_for_cam.mat_10,
        intrinsics_for_cam.mat_11, intrinsics_for_cam.mat_12,
        intrinsics_for_cam.mat_20, intrinsics_for_cam.mat_21,
        intrinsics_for_cam.mat_22;
    cam_intrinsics_map[intrinsics_for_cam.camera_id] = intrinsics_mat;
    // TODO do we need to include the width/height anywhere?
  }
  return cam_intrinsics_map;
}

std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
readCameraExtrinsicsByCameraFromFile(const std::string &file_name) {
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      extrinsics_map;

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a
  // classic world frame - for the camera +z is the +x world axis, +y is the
  // -z world axis, and +x is the -y world axis
  // TODO Verify that this is correct (and eventually just update the file to
  // have this)
  //  Eigen::Quaterniond extrinsics_orientation_switch_to_cam =
  //      Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)
  //          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_by_cam_id;
  file_io::readCameraExtrinsicsWithIdsFromFile(file_name,
                                               camera_extrinsics_by_cam_id);
  for (const file_io::CameraExtrinsicsWithId &extrinsics_for_cam :
       camera_extrinsics_by_cam_id) {
    vtr::Position3d<double> extrinsics_pos(extrinsics_for_cam.transl_x,
                                           extrinsics_for_cam.transl_y,
                                           extrinsics_for_cam.transl_z);

    vtr::Orientation3D<double> extrinsics_orient(
        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
                           extrinsics_for_cam.quat_x,
                           extrinsics_for_cam.quat_y,
                           extrinsics_for_cam.quat_z));
    vtr::CameraExtrinsics<double> extrinsics_obj(extrinsics_pos,
                                                 extrinsics_orient);

    extrinsics_map[extrinsics_for_cam.camera_id] = extrinsics_obj;
  }
  return extrinsics_map;
}

std::unordered_map<
    vtr::FrameId,
    std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
readBoundingBoxesFromFile(const std::string &file_name) {
  std::vector<file_io::BoundingBoxWithNodeId> bounding_boxes_by_node_id;
  file_io::readBoundingBoxesWithNodeIdFromFile(file_name,
                                               bounding_boxes_by_node_id);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bb_map;
  for (const file_io::BoundingBoxWithNodeId &raw_bb :
       bounding_boxes_by_node_id) {
    if (raw_bb.detection_confidence >= FLAGS_min_confidence) {
      vtr::RawBoundingBox bb;
      bb.pixel_corner_locations_ = std::make_pair(
          vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
          vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
      bb.semantic_class_ = raw_bb.semantic_class;
      bb.detection_confidence_ = raw_bb.detection_confidence;
      bb_map[raw_bb.node_id][raw_bb.camera_id].emplace_back(bb);
    }
  }
  return bb_map;
}

std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> readRobotPosesFromFile(
    const std::string &file_name) {
  std::vector<std::pair<uint64_t, pose::Pose3d>> robot_poses_by_node_id;
  file_io::readPose3dsAndNodeIdFromFile(file_name, robot_poses_by_node_id);
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses_by_node_num;
  for (const std::pair<uint64, pose::Pose3d> &pose_3d_with_frame :
       robot_poses_by_node_id) {
    vtr::Pose3D<double> pose(
        pose_3d_with_frame.second.first,
        vtr::Orientation3D<double>(pose_3d_with_frame.second.second));
    robot_poses_by_node_num[pose_3d_with_frame.first] = pose;
  }
  return robot_poses_by_node_num;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::optional<vtr::OptimizationLogger> opt_logger;
  if (FLAGS_logs_directory.empty()) {
    FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  } else {
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = FLAGS_logs_directory;
    opt_logger = vtr::OptimizationLogger(
        file_io::ensureDirectoryPathEndsWithSlash(FLAGS_logs_directory) +
        kCeresOptInfoLogFile);
  }
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_extrinsics_file.empty()) {
    LOG(ERROR) << "No extrinsics file provided";
    exit(1);
  }

  if (FLAGS_intrinsics_file.empty()) {
    LOG(ERROR) << "No intrinsics file provided";
    exit(1);
  }

  if (FLAGS_poses_by_node_id_file.empty()) {
    LOG(ERROR) << "No robot poses file provided";
    exit(1);
  }

  if (FLAGS_long_term_map_output.empty()) {
    LOG(ERROR) << "No long-term map output file provided";
    exit(1);
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(WARNING) << "No rosbag file provided";
  }

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(WARNING) << "No nodes by timestamp file";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, "a_" + node_prefix + "ellipsoid_estimator_real_data");
  ros::NodeHandle node_handle;

  vtr::FullOVSLAMConfig config;
  vtr::readConfiguration(FLAGS_params_config_file, config);

  // Hard-coded values -----------------------------------------------------

  // TODO modify convergence thresholds
  pose_graph_optimization::OptimizationSolverParams local_ba_solver_params =
      config.local_ba_solver_params_;

  pose_graph_optimization::OptimizationSolverParams global_ba_solver_params =
      config.global_ba_solver_params_;

  pose_graph_optimization::OptimizationSolverParams final_opt_solver_params =
      config.final_ba_solver_params_;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params =
      config.object_visual_pose_graph_residual_params_;

  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera =
          readCameraIntrinsicsByCameraFromFile(FLAGS_intrinsics_file);
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera =
          readCameraExtrinsicsByCameraFromFile(FLAGS_extrinsics_file);
  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      init_bounding_boxes;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  for (const auto &frame_id_bb_entry : init_bounding_boxes) {
    for (const auto &camera_bb_entry : frame_id_bb_entry.second) {
      for (const vtr::RawBoundingBox &bb : camera_bb_entry.second) {
        if ((bb.pixel_corner_locations_.second.x() < 1280) &&
            (bb.pixel_corner_locations_.second.y() < 720)) {
          bounding_boxes[frame_id_bb_entry.first][camera_bb_entry.first]
              .emplace_back(bb);
        }
      }
    }
  }
  std::shared_ptr<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::vector<std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>>
      all_observed_corner_locations_with_uncertainty =
          std::make_shared<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::vector<std::pair<vtr::BbCornerPair<double>,
                                        std::optional<double>>>>>>();
  for (const auto &bounding_boxes_for_frame : bounding_boxes) {
    for (const auto &bounding_boxes_for_frame_and_cam :
         bounding_boxes_for_frame.second) {
      std::vector<std::pair<vtr::BbCornerPair<double>, std::optional<double>>>
          observed_corners_for_frame_and_cam;
      for (const vtr::RawBoundingBox &bb :
           bounding_boxes_for_frame_and_cam.second) {
        observed_corners_for_frame_and_cam.emplace_back(std::make_pair(
            bb.pixel_corner_locations_, bb.detection_confidence_));
      }
      (*all_observed_corner_locations_with_uncertainty)
          [bounding_boxes_for_frame.first]
          [bounding_boxes_for_frame_and_cam.first] =
              observed_corners_for_frame_and_cam;
    }
  }
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      readRobotPosesFromFile(FLAGS_poses_by_node_id_file);
  LOG(INFO) << "Reading images from rosbag";
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(
          FLAGS_rosbag_file,
          FLAGS_nodes_by_timestamp_file,
          config.camera_info_.camera_topic_to_camera_id_);
  LOG(INFO) << "Done reading images from rosbag";
  std::unordered_map<vtr::CameraId, std::pair<double, double>>
      img_heights_and_widths;
  for (const auto &frame_and_imgs : images) {
    for (const auto &cam_and_img : frame_and_imgs.second) {
      if (img_heights_and_widths.find(cam_and_img.first) ==
          img_heights_and_widths.end()) {
        sensor_msgs::Image::ConstPtr img = cam_and_img.second;
        img_heights_and_widths[cam_and_img.first] =
            std::make_pair(img->height, img->width);
      }
    }
  }
  MainLtmPtr long_term_map;
  if (!FLAGS_long_term_map_input.empty()) {
    cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
    vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
        //        std::unordered_map<vtr::ObjectId,
        //        vtr::RoshanAggregateBbInfo>,
        //        vtr::SerializableMap<vtr::ObjectId,
        //                             vtr::SerializableObjectId,
        //                             vtr::RoshanAggregateBbInfo,
        //                             vtr::SerializableRoshanAggregateBbInfo>>
        util::EmptyStruct,
        vtr::SerializableEmptyStruct>
        serializable_ltm;
    ltm_in_fs["long_term_map"] >> serializable_ltm;
    ltm_in_fs.release();
    MainLtm ltm_from_serializable = serializable_ltm.getEntry();
    vtr::EllipsoidResults ellipsoid_results_ltm;
    ltm_from_serializable.getLtmEllipsoidResults(ellipsoid_results_ltm);
    LOG(INFO) << "Long term map size "
              << ellipsoid_results_ltm.ellipsoids_.size();
    long_term_map = std::make_shared<MainLtm>(ltm_from_serializable);
  }
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<
          vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>>
      low_level_features_map;
  std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
      initial_feat_positions;

  if (!FLAGS_low_level_feats_dir.empty()) {
    LOG(INFO) << "Reading low level features";
    vtr::OrbOutputLowLevelFeatureReader orb_feat_reader(
        FLAGS_low_level_feats_dir, {}, config.limit_traj_eval_params_);
    orb_feat_reader.getLowLevelFeatures(visual_features);
    for (const auto &feature_track : visual_features) {
      vtr::FeatureId feat_id = feature_track.first;
      initial_feat_positions[feat_id] = feature_track.second.feature_pos_;
      for (const auto &feat_obs_by_frame :
           feature_track.second.feature_track.feature_observations_) {
        vtr::FrameId frame_id = feat_obs_by_frame.first;
        for (const auto &feat_obs_for_cam :
             feat_obs_by_frame.second.pixel_by_camera_id) {
          vtr::CameraId cam_id = feat_obs_for_cam.first;
          low_level_features_map[frame_id][cam_id][feat_id] =
              feat_obs_for_cam.second;
        }
      }
    }
  }

  LOG(INFO) << "Ready to debug";
  if (FLAGS_target_frame_id >= 0) {
    LOG(INFO) << "Before debugInitVisualFeaturesByFrameId";
    debugInitVisualFeaturesByFrameId(
        FLAGS_debug_output_directory,
        FLAGS_target_frame_id,
        camera_intrinsics_by_camera,
        camera_extrinsics_by_camera,
        images.at(FLAGS_target_frame_id),
        robot_poses.at(FLAGS_target_frame_id),
        initial_feat_positions,
        low_level_features_map.at(FLAGS_target_frame_id));
    LOG(INFO) << "After debugInitVisualFeaturesByFrameId";
  } else {
    std::unordered_set<vtr::FeatureId> abnormal_feat_ids;
    abnormal_feat_ids = {53376, 52731, 53688, 55719, 53022, 46146};
    LOG(INFO) << "Before debugInitVisualFeaturesByFeatureIds";
    debugInitVisualFeaturesByFeatureIds(FLAGS_debug_output_directory,
                                        abnormal_feat_ids,
                                        camera_intrinsics_by_camera,
                                        camera_extrinsics_by_camera,
                                        images,
                                        robot_poses,
                                        initial_feat_positions,
                                        low_level_features_map);
    LOG(INFO) << "After debugInitVisualFeaturesByFeatureIds";
  }
}