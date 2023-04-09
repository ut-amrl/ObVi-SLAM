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

DEFINE_int32(target_frame_id_1, 0, "");
DEFINE_int32(target_frame_id_2, 1, "");
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

double alpha = 0.9;
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

cv_bridge::CvImagePtr getEpipolarErrorBestInlierAndOutlierVisualization(
    const std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        &feat_ids_and_features2d_1,
    const std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        &feat_ids_and_features2d_2,
    const sensor_msgs::Image::ConstPtr &img_msg,
    const vtr::Pose3D<double> &pose_1,
    const vtr::Pose3D<double> &pose_2,
    const vtr::CameraExtrinsics<double> &extrinsics_1,
    const vtr::CameraExtrinsics<double> &extrinsics_2,
    vtr::CameraIntrinsicsMat<double> &intrinsics_1,
    vtr::CameraIntrinsicsMat<double> &intrinsics_2) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, vtr::kImageEncoding);
  } catch (cv_bridge::Exception &e) {
    LOG(ERROR) << "cv_bridge exception: " << e.what();
    exit(1);
  }

  struct EpipolarInfo {
    vtr::FeatureId feat_id;
    vtr::PixelCoord<double> p2;
    vtr::PixelCoord<double> epipole_in2;
    vtr::PixelCoord<double> x1_in2_2d;
    EpipolarInfo() {}
    EpipolarInfo(const vtr::FeatureId &feat_id,
                 const vtr::PixelCoord<double> p2,
                 const vtr::PixelCoord<double> epipole_in2,
                 const vtr::PixelCoord<double> x1_in2_2d)
        : feat_id(feat_id),
          p2(p2),
          epipole_in2(epipole_in2),
          x1_in2_2d(x1_in2_2d) {}
  };

  std::map<double, EpipolarInfo, std::greater<double>>
      ordered_errs_and_epipolar_info;
  for (const auto &feat_id_and_feature_2 : feat_ids_and_features2d_2) {
    const vtr::FeatureId feat_id = feat_id_and_feature_2.first;
    if (feat_ids_and_features2d_1.find(feat_id) ==
        feat_ids_and_features2d_1.end()) {
      continue;
    }
    const vtr::PixelCoord<double> &pixel_2 = feat_id_and_feature_2.second;
    const vtr::PixelCoord<double> &pixel_1 =
        feat_ids_and_features2d_1.at(feat_id);
    Eigen::Vector2d epipole_in2, x1_in2_2d;
    Eigen::Vector2d epipolar_error_vec =
        getNormalizedEpipolarErrorVec(intrinsics_1,
                                      intrinsics_2,
                                      extrinsics_1,
                                      extrinsics_2,
                                      pixel_1,
                                      pixel_2,
                                      pose_1,
                                      pose_2,
                                      &epipole_in2,
                                      &x1_in2_2d);
    // LOG(INFO) << "Feature " << feat_id << " : err1 = " << epipolar_error
    //           << ", err2 = " << epipolar_error_vec.norm();
    double epipolar_error;
    epipolar_error = epipolar_error_vec.norm();
    ordered_errs_and_epipolar_info[epipolar_error] =
        EpipolarInfo(feat_id, pixel_2, epipole_in2, x1_in2_2d);
  }
  std_msgs::ColorRGBA inlier_color = ColorRGBA_init(0, 1, 0, alpha);
  std_msgs::ColorRGBA outlier_color = ColorRGBA_init(1, 0, 0, alpha);
  // std_msgs::ColorRGBA default_color = ColorRGBA_init(0, 0, 0, alpha);
  const EpipolarInfo &inlier_info =
      ordered_errs_and_epipolar_info.rbegin()->second;
  vtr::RosVisualization::drawLineOnImage(
      inlier_info.epipole_in2, inlier_info.x1_in2_2d, inlier_color, cv_ptr);
  vtr::RosVisualization::drawTinyCircleOnImage(
      inlier_info.p2, inlier_color, cv_ptr);

  const EpipolarInfo &outlier_info =
      ordered_errs_and_epipolar_info.begin()->second;
  vtr::RosVisualization::drawLineOnImage(
      outlier_info.epipole_in2, outlier_info.x1_in2_2d, outlier_color, cv_ptr);
  vtr::RosVisualization::drawTinyCircleOnImage(
      outlier_info.p2, outlier_color, cv_ptr);
  return cv_ptr;
}

std::tuple<cv_bridge::CvImagePtr, std::map<double, vtr::FeatureId>>
getEpipolarErrorVisualization(
    const std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        &feat_ids_and_features2d_1,
    const std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        &feat_ids_and_features2d_2,
    const sensor_msgs::Image::ConstPtr &img_msg,
    const vtr::Pose3D<double> &pose_1,
    const vtr::Pose3D<double> &pose_2,
    const vtr::CameraExtrinsics<double> &extrinsics_1,
    const vtr::CameraExtrinsics<double> &extrinsics_2,
    vtr::CameraIntrinsicsMat<double> &intrinsics_1,
    vtr::CameraIntrinsicsMat<double> &intrinsics_2) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, vtr::kImageEncoding);
  } catch (cv_bridge::Exception &e) {
    LOG(ERROR) << "cv_bridge exception: " << e.what();
    exit(1);
  }

  struct EpipolarInfo {
    vtr::FeatureId feat_id;
    vtr::PixelCoord<double> p1;
    vtr::PixelCoord<double> p2;
    EpipolarInfo() {}
    EpipolarInfo(const vtr::FeatureId &feat_id,
                 const vtr::PixelCoord<double> &p1,
                 const vtr::PixelCoord<double> &p2)
        : feat_id(feat_id), p1(p1), p2(p2) {}
  };
  std::map<double, EpipolarInfo, std::greater<double>>
      ordered_errs_and_epipolar_info;
  for (const auto &feat_id_and_feature_2 : feat_ids_and_features2d_2) {
    const vtr::FeatureId feat_id = feat_id_and_feature_2.first;
    if (feat_ids_and_features2d_1.find(feat_id) ==
        feat_ids_and_features2d_1.end()) {
      continue;
    }
    const vtr::PixelCoord<double> &pixel_2 = feat_id_and_feature_2.second;
    const vtr::PixelCoord<double> &pixel_1 =
        feat_ids_and_features2d_1.at(feat_id);
    double epipolar_error;
    epipolar_error = getNormalizedEpipolarError(intrinsics_1,
                                                intrinsics_2,
                                                extrinsics_1,
                                                extrinsics_2,
                                                pixel_1,
                                                pixel_2,
                                                pose_1,
                                                pose_2);
    Eigen::Vector2d epipolar_error_vec =
        getNormalizedEpipolarErrorVec(intrinsics_1,
                                      intrinsics_2,
                                      extrinsics_1,
                                      extrinsics_2,
                                      pixel_1,
                                      pixel_2,
                                      pose_1,
                                      pose_2);
    LOG(INFO) << "Feature " << feat_id << " : err1 = " << epipolar_error
              << ", err2 = " << epipolar_error_vec.norm();
    epipolar_error = epipolar_error_vec.norm();
    ordered_errs_and_epipolar_info[epipolar_error] =
        EpipolarInfo(feat_id, pixel_1, pixel_2);
  }
  std_msgs::ColorRGBA inlier_color = ColorRGBA_init(0, 1, 0, alpha);
  std_msgs::ColorRGBA outlier_color = ColorRGBA_init(1, 0, 0, alpha);
  std_msgs::ColorRGBA default_color = ColorRGBA_init(0, 0, 0, alpha);
  // double outlier_lbound = 1e-4;
  // double inlier_ubound = 1e-6;
  double outlier_lbound = 3;
  double inlier_ubound = 0.1;

  size_t n_inlier, n_outlier;
  n_inlier = n_outlier = 0;
  for (const auto &err_and_epipolar_info : ordered_errs_and_epipolar_info) {
    const double err = err_and_epipolar_info.first;
    const EpipolarInfo &epipolar_info = err_and_epipolar_info.second;
    std_msgs::ColorRGBA color;
    if (err <= inlier_ubound) {
      // LOG(INFO) << "Feature " << epipolar_info.feat_id << " has error " <<
      // err; LOG(INFO) << "err: " << err << ", inlier_ubound: " <<
      // inlier_ubound;
      color = inlier_color;
      ++n_inlier;
    } else if (err >= outlier_lbound) {
      color = outlier_color;
      ++n_outlier;
    } else {
      color = default_color;
    }
    vtr::RosVisualization::drawTinyCircleOnImage(
        epipolar_info.p1, color, cv_ptr);
    vtr::RosVisualization::drawTinyCircleOnImage(
        epipolar_info.p2, color, cv_ptr);
    vtr::RosVisualization::drawLineOnImage(
        epipolar_info.p1, epipolar_info.p2, color, cv_ptr);
  }

  std::map<double, vtr::FeatureId> ordered_errors_and_feat_ids;
  for (const auto &err_and_epipolar_info : ordered_errs_and_epipolar_info) {
    ordered_errors_and_feat_ids[err_and_epipolar_info.first] =
        err_and_epipolar_info.second.feat_id;
  }
  LOG(INFO) << "n_inlier: " << n_inlier << ", n_outlier: " << n_outlier;
  return std::make_tuple(cv_ptr, ordered_errors_and_feat_ids);
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

  vtr::CameraId cam_id_1 = 1;
  vtr::CameraId cam_id_2 = 1;

  auto summary =
      getEpipolarErrorBestInlierAndOutlierVisualization(
          low_level_features_map.at(FLAGS_target_frame_id_1).at(cam_id_1),
          low_level_features_map.at(FLAGS_target_frame_id_2).at(cam_id_2),
          images.at(FLAGS_target_frame_id_1).at(cam_id_1),
          robot_poses.at(FLAGS_target_frame_id_1),
          robot_poses.at(FLAGS_target_frame_id_2),
          camera_extrinsics_by_camera.at(cam_id_1),
          camera_extrinsics_by_camera.at(cam_id_2),
          camera_intrinsics_by_camera.at(cam_id_1),
          camera_intrinsics_by_camera.at(cam_id_2))
          ->image;

  // auto ret_1 = getEpipolarErrorVisualization(
  //     low_level_features_map.at(FLAGS_target_frame_id_1).at(cam_id_1),
  //     low_level_features_map.at(FLAGS_target_frame_id_2).at(cam_id_2),
  //     images.at(FLAGS_target_frame_id_1).at(cam_id_1),
  //     robot_poses.at(FLAGS_target_frame_id_1),
  //     robot_poses.at(FLAGS_target_frame_id_2),
  //     camera_extrinsics_by_camera.at(cam_id_1),
  //     camera_extrinsics_by_camera.at(cam_id_2),
  //     camera_intrinsics_by_camera.at(cam_id_1),
  //     camera_intrinsics_by_camera.at(cam_id_2));
  // auto cv_ptr_1 = std::get<0>(ret_1);
  // auto errs_and_feat_ids_1 = std::get<1>(ret_1);

  // auto ret_2 = getEpipolarErrorVisualization(
  //     low_level_features_map.at(FLAGS_target_frame_id_1).at(cam_id_1),
  //     low_level_features_map.at(FLAGS_target_frame_id_2).at(cam_id_2),
  //     images.at(FLAGS_target_frame_id_2).at(cam_id_2),
  //     robot_poses.at(FLAGS_target_frame_id_1),
  //     robot_poses.at(FLAGS_target_frame_id_2),
  //     camera_extrinsics_by_camera.at(cam_id_1),
  //     camera_extrinsics_by_camera.at(cam_id_2),
  //     camera_intrinsics_by_camera.at(cam_id_1),
  //     camera_intrinsics_by_camera.at(cam_id_2));
  // auto cv_ptr_2 = std::get<0>(ret_2);
  // auto errs_and_feat_ids_2 = std::get<1>(ret_2);

  // util::BoostHashMap<vtr::FrameId, cv::Mat> ids_and_visualizations = {
  //     {vtr::FrameId(FLAGS_target_frame_id_1), cv_ptr_1->image},
  //     {vtr::FrameId(FLAGS_target_frame_id_2), cv_ptr_2->image}};
  // cv::Mat summary = vtr::generateMosaic(ids_and_visualizations);

  std::string directory_perfix = "epipolar";
  fs::path output_directory =
      fs::path(FLAGS_debug_output_directory) / directory_perfix;
  setupOutputDirectory(output_directory);
  std::string filename = std::to_string((int)FLAGS_target_frame_id_1) + "_" +
                         std::to_string((int)FLAGS_target_frame_id_2) + ".png";
  fs::path output_path = output_directory / filename;
  LOG(INFO) << "Writing to path " << output_path;
  cv::imwrite(output_path, summary);
  // for (const auto err_and_feat_id : errs_and_feat_ids) {
  //   LOG(INFO) << "Feature " << err_and_feat_id.second << " has epipolar error
  //   "
  //             << err_and_feat_id.first;
  // }
}