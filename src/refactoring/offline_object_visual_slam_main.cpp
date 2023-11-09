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
DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
DEFINE_string(bounding_boxes_by_timestamp_file,
              "",
              "File with bounding box observations by timestamp");
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
DEFINE_string(low_level_feats_dir,
              "",
              "Directory that contains low level features");
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
DEFINE_string(
    output_checkpoints_dir,
    "",
    "Directory to output checkpoints to. If not specified, none are saved");
// TODO use this
DEFINE_string(input_checkpoints_dir,
              "",
              "Directory to read checkpoints from. If not specified, "
              "optimization should start from the beginning.");
DEFINE_bool(disable_log_to_stderr,
            false,
            "Set to true if the logging to standard error should be disabled");

std::unordered_map<
    vtr::FrameId,
    std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
readBoundingBoxesByTimestampFromFile(
    const std::string &bounding_boxes_file_name,
    const std::string &nodes_by_timestamp_file) {
  std::vector<file_io::BoundingBoxWithTimestamp> bounding_boxes_by_timestamp;
  file_io::readBoundingBoxWithTimestampsFromFile(bounding_boxes_file_name,
                                                 bounding_boxes_by_timestamp);
  LOG(INFO) << bounding_boxes_by_timestamp.size() << " bounding boxes read";

  std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
  util::BoostHashMap<pose::Timestamp, vslam_types_refactor::FrameId>
      nodes_for_timestamps_map;
  file_io::readNodeIdsAndTimestampsFromFile(nodes_by_timestamp_file,
                                            nodes_by_timestamps_vec);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bb_map;
  for (const file_io::NodeIdAndTimestamp &raw_node_id_and_timestamp :
       nodes_by_timestamps_vec) {
    nodes_for_timestamps_map[std::make_pair(
        raw_node_id_and_timestamp.seconds_,
        raw_node_id_and_timestamp.nano_seconds_)] =
        raw_node_id_and_timestamp.node_id_;
    bb_map[raw_node_id_and_timestamp.node_id_] = {};
  }

  for (const file_io::BoundingBoxWithTimestamp &raw_bb :
       bounding_boxes_by_timestamp) {
    vtr::RawBoundingBox bb;
    pose::Timestamp stamp_for_bb =
        std::make_pair(raw_bb.seconds, raw_bb.nano_seconds);
    if (nodes_for_timestamps_map.find(stamp_for_bb) ==
        nodes_for_timestamps_map.end()) {
      // No frame for timestamp
      continue;
    }

    bb.pixel_corner_locations_ = std::make_pair(
        vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
        vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
    bb.semantic_class_ = raw_bb.semantic_class;
    bb.detection_confidence_ = raw_bb.detection_confidence;

    bb_map[nodes_for_timestamps_map.at(stamp_for_bb)][raw_bb.camera_id]
        .emplace_back(bb);
  }

  return bb_map;
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
    vtr::RawBoundingBox bb;
    bb.pixel_corner_locations_ = std::make_pair(
        vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
        vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
    bb.semantic_class_ = raw_bb.semantic_class;
    bb.detection_confidence_ = raw_bb.detection_confidence;
    bb_map[raw_bb.node_id][raw_bb.camera_id].emplace_back(bb);
  }
  return bb_map;
}

void createPoseGraph(
    const MainProbData &input_problem_data,
    const std::function<bool(
        const std::unordered_set<vtr::ObjectId> &,
        util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>>
            &)> &long_term_map_factor_provider,
    MainPgPtr &pose_graph) {
  std::unordered_map<vtr::ObjectId,
                     std::pair<std::string, vtr::RawEllipsoid<double>>>
      ltm_objects;
  vtr::EllipsoidResults ellipsoids_in_map;
  if (input_problem_data.getLongTermObjectMap() != nullptr) {
    input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
        ellipsoids_in_map);
  }
  LOG(INFO) << "Ellipsoids size " << ellipsoids_in_map.ellipsoids_.size();
  for (const auto &ellipsoid_entry : ellipsoids_in_map.ellipsoids_) {
    ltm_objects[ellipsoid_entry.first] = std::make_pair(
        ellipsoid_entry.second.first,
        vtr::convertToRawEllipsoid(ellipsoid_entry.second.second));
  }
  LOG(INFO) << "Ltm objects size " << ltm_objects.size();
  LOG(INFO) << "Creating pose graph ";
  pose_graph =
      std::make_shared<MainPg>(input_problem_data.getObjDimMeanAndCovByClass(),
                               input_problem_data.getCameraExtrinsicsByCamera(),
                               input_problem_data.getCameraIntrinsicsByCamera(),
                               ltm_objects,
                               long_term_map_factor_provider);
}

void publishLowLevelFeaturesLatestImages(
    const std::shared_ptr<vtr::RosVisualization> &vis_manager,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &intrinsics,
    const std::unordered_map<vtr::CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>
        &images,
    const vtr::FrameId &latest_frame_num,
    const vtr::Pose3D<double> &pose_at_frame,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &feature_ests,
    const std::unordered_map<
        vtr::CameraId,
        std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
        &observed_feats_for_frame,
    const vtr::PlotType &plot_type) {
  for (const auto &obs_feat_and_cam : observed_feats_for_frame) {
    vtr::CameraId cam_id = obs_feat_and_cam.first;
    if (img_heights_and_widths.find(cam_id) == img_heights_and_widths.end()) {
      LOG(ERROR) << "Didn't have height and width for camera with observed "
                    "features (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    std::pair<double, double> img_height_and_width =
        img_heights_and_widths.at(cam_id);
    if (extrinsics.find(cam_id) == extrinsics.end()) {
      LOG(ERROR) << "Didn't have extrinsics for camera with observed features"
                    " (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    vtr::CameraExtrinsics<double> extrinsics_for_cam = extrinsics.at(cam_id);

    if (intrinsics.find(cam_id) == intrinsics.end()) {
      LOG(ERROR) << "Didn't have intrinsics for camera with observed features"
                    " (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    vtr::CameraIntrinsicsMat<double> intrinsics_for_cam = intrinsics.at(cam_id);

    std::optional<sensor_msgs::Image::ConstPtr> img_for_cam = std::nullopt;
    if (images.find(cam_id) != images.end()) {
      img_for_cam = images.at(cam_id);
    }

    std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        projected_pixels;
    for (const auto &feat_est : feature_ests) {
      if (obs_feat_and_cam.second.find(feat_est.first) ==
          obs_feat_and_cam.second.end()) {
        // Feature wasn't observed, don't want to visualize it
        continue;
      }

      projected_pixels[feat_est.first] =
          vtr::getProjectedPixelCoord(feat_est.second,
                                      pose_at_frame,
                                      extrinsics_for_cam,
                                      intrinsics_for_cam);
    }

    vis_manager->publishLatestImageWithReprojectionResiduals(
        latest_frame_num,
        cam_id,
        intrinsics_for_cam,
        plot_type,
        obs_feat_and_cam.second,
        projected_pixels,
        img_for_cam,
        img_height_and_width,
        true);
  }
}

void visualizationStub(
    const std::shared_ptr<vtr::RosVisualization> &vis_manager,
    vtr::SaveToFileVisualizer &save_to_file_visualizer,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &intrinsics,
    const std::unordered_map<vtr::CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
        &images,
    const std::shared_ptr<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId,
                           std::vector<std::pair<vtr::BbCornerPair<double>,
                                                 std::optional<double>>>>>>
        &all_observed_corner_locations_with_uncertainty,
    const std::shared_ptr<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<
                vtr::ObjectId,
                std::pair<vtr::BbCornerPair<double>, std::optional<double>>>>>>
        &observed_corner_locations,
    const std::shared_ptr<std::vector<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId,
                           std::pair<vtr::BbCornerPair<double>, double>>>>>
        &bounding_boxes_for_pending_object,
    const std::shared_ptr<std::vector<
        std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
        &pending_objects,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>>
        &observed_features,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &initial_feat_positions,
    const MainProbData &input_problem_data,
    std::shared_ptr<
        vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
        &pose_graph,
    const vtr::FrameId &min_frame_optimized,
    const vtr::FrameId &max_frame_optimized,
    const vtr::VisualizationTypeEnum &visualization_stage,
    const double &near_edge_threshold,
    const size_t &pending_obj_min_obs_threshold,
    const std::optional<std::vector<vtr::Pose3D<double>>> &gt_trajectory,
    const vtr::FrameId &final_frame_id,
    const std::string &output_checkpoints_dir,
    const int &attempt = 0) {
#ifdef RUN_TIMERS
  CumulativeFunctionTimer::Invocation invoc(
      vtr::CumulativeTimerFactory::getInstance()
          .getOrCreateFunctionTimer(vtr::kTimerNameVisFunction)
          .get());
#endif
  bool pgo_opt = false;
  switch (visualization_stage) {
    case vtr::BEFORE_ANY_OPTIMIZATION:
      //      vis_manager->publishTransformsForEachCamera(
      //          input_problem_data.getMaxFrameId(),
      //          input_problem_data.getRobotPoseEstimates(),
      //          input_problem_data.getCameraExtrinsicsByCamera());

      sleep(3);
      break;
    case vtr::BEFORE_EACH_OPTIMIZATION:
      if ((max_frame_optimized == final_frame_id) &&
          (!output_checkpoints_dir.empty())) {
        LOG(INFO) << "Dumping pose graph before final opt";
        MainPgPtr derived_pose_graph =
            std::dynamic_pointer_cast<MainPg>(pose_graph);
        vtr::outputPoseGraphToFile(
            derived_pose_graph,
            file_io::ensureDirectoryPathEndsWithSlash(
                FLAGS_output_checkpoints_dir) +
                vtr::kPreOptimizationCheckpointOutputFileBaseName +
                std::to_string(final_frame_id) + vtr::kAttemptSuffix +
                std::to_string(attempt) + file_io::kJsonExtension);
      }
      break;
    case vtr::AFTER_PGO_PLUS_OBJ_OPTIMIZATION:
      pgo_opt = true;
    case vtr::AFTER_EACH_OPTIMIZATION: {
      std::unordered_map<vtr::FrameId, vtr::RawPose3d<double>>
          optimized_robot_pose_estimates;
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          initial_robot_pose_estimates =
              input_problem_data.getRobotPoseEstimates();
      pose_graph->getRobotPoseEstimates(optimized_robot_pose_estimates);
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          optimized_trajectory;
      for (const auto &frame_raw_pose : optimized_robot_pose_estimates) {
        optimized_trajectory[frame_raw_pose.first] =
            vtr::convertToPose3D(frame_raw_pose.second);
      }

      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::RawEllipsoid<double>>>
          object_estimates;
      pose_graph->getObjectEstimates(object_estimates);
      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::EllipsoidState<double>>>
          optimized_ellipsoid_estimates_with_classes;
      std::unordered_map<vtr::ObjectId, vtr::EllipsoidState<double>>
          optimized_ellipsoid_estimates;
      for (const auto &obj_and_raw_est : object_estimates) {
        vtr::EllipsoidState<double> ellipsoid_state =
            vtr::convertToEllipsoidState(obj_and_raw_est.second.second);
        optimized_ellipsoid_estimates_with_classes[obj_and_raw_est.first] =
            std::make_pair(obj_and_raw_est.second.first, ellipsoid_state);
        optimized_ellipsoid_estimates[obj_and_raw_est.first] = ellipsoid_state;
      }
      std_msgs::ColorRGBA optimized_ellipsoid_color;
      optimized_ellipsoid_color.a = 0.5;
      //      optimized_ellipsoid_color.r = 1.0;
      optimized_ellipsoid_color.g = 1;
      vis_manager->visualizeEllipsoids(
          optimized_ellipsoid_estimates_with_classes, vtr::ESTIMATED);
      std::unordered_map<
          vtr::ObjectId,
          std::pair<
              std::string,
              std::pair<
                  vtr::EllipsoidState<double>,
                  vtr::Covariance<double, vtr::kEllipsoidParamterizationSize>>>>
          ltm_ellipsoids;
      if (input_problem_data.getLongTermObjectMap() != nullptr) {
        vtr::EllipsoidResults ellipsoids_in_map;
        input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
            ellipsoids_in_map);

        // TODO do this outside of visualization loop
        std::unordered_map<
            vtr::ObjectId,
            vtr::Covariance<double, vtr::kEllipsoidParamterizationSize>>
            ellipsoid_covariances = input_problem_data.getLongTermObjectMap()
                                        ->getEllipsoidCovariances();
        for (const auto &ltm_ellipsoid : ellipsoids_in_map.ellipsoids_) {
          ltm_ellipsoids[ltm_ellipsoid.first] = std::make_pair(
              ltm_ellipsoid.second.first,
              std::make_pair(ltm_ellipsoid.second.second,
                             ellipsoid_covariances.at(ltm_ellipsoid.first)));
        }
      }

      vis_manager->publishLongTermMap(ltm_ellipsoids);

      std::vector<size_t> num_obs_per_pending_obj;
      for (const auto &pending_obj_obs : *bounding_boxes_for_pending_object) {
        size_t num_obs_for_pending_obj = 0;
        for (const auto &obs_for_frame : pending_obj_obs) {
          for (const auto &obs_for_cam : obs_for_frame.second) {
            num_obs_for_pending_obj++;
          }
        }
        num_obs_per_pending_obj.emplace_back(num_obs_for_pending_obj);
      }
      vis_manager->visualizePendingEllipsoids(pending_objects,
                                              num_obs_per_pending_obj,
                                              pending_obj_min_obs_threshold);
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests;
      pose_graph->getVisualFeatureEstimates(feature_ests);
      std::unordered_map<
          vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
          observed_feats_for_frame = observed_features.at(max_frame_optimized);
      if (images.find(max_frame_optimized) == images.end()) {
        LOG(INFO) << "No images for frame " << max_frame_optimized;
      }
      if (optimized_trajectory.find(max_frame_optimized) ==
          optimized_trajectory.end()) {
        LOG(INFO) << "No opt trajectory for frame " << max_frame_optimized;
      }

      publishLowLevelFeaturesLatestImages(
          vis_manager,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images.at(max_frame_optimized),
          max_frame_optimized,
          optimized_trajectory.at(max_frame_optimized),
          feature_ests,
          observed_feats_for_frame,
          vtr::PlotType::ESTIMATED);

      publishLowLevelFeaturesLatestImages(
          vis_manager,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images.at(max_frame_optimized),
          max_frame_optimized,
          input_problem_data.getRobotPoseEstimates().at(max_frame_optimized),
          initial_feat_positions,
          observed_feats_for_frame,
          vtr::PlotType::INITIAL);

      vis_manager->visualizeCameraObservations(
          max_frame_optimized,
          initial_robot_pose_estimates,
          optimized_trajectory,
          std::nullopt,
          std::nullopt,  // TODO should we extract the initial ellipsoid
                         // estimates from the front end?
          optimized_ellipsoid_estimates,
          std::nullopt,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images,
          *observed_corner_locations,
          {},
          false,
          near_edge_threshold);

      vis_manager->publishDetectedBoundingBoxesWithUncertainty(
          max_frame_optimized,
          *all_observed_corner_locations_with_uncertainty,
          images,
          intrinsics,
          img_heights_and_widths,
          vtr::PlotType::ESTIMATED);

      std::vector<vtr::Pose3D<double>> est_trajectory_vec;
      std::vector<vtr::Pose3D<double>> init_trajectory_vec;
      //      for (size_t i = 0; i <= (max_frame_optimized -
      //      min_frame_optimized);
      //           i++) {
      //        vtr::FrameId frame_id = i + min_frame_optimized;
      for (vtr::FrameId frame_id = 0; frame_id <= max_frame_optimized;
           frame_id++) {
        est_trajectory_vec.emplace_back(optimized_trajectory.at(frame_id));
        init_trajectory_vec.emplace_back(
            initial_robot_pose_estimates.at(frame_id));
      }
      vis_manager->visualizeTrajectory(init_trajectory_vec,
                                       vtr::PlotType::INITIAL);
      vis_manager->visualizeTrajectory(est_trajectory_vec,
                                       vtr::PlotType::ESTIMATED);
      if (gt_trajectory.has_value()) {
        std::vector<vtr::Pose3D<double>> truncated_gt_traj(
            gt_trajectory.value().begin(),
            gt_trajectory.value().begin() + (max_frame_optimized + 1));
        vis_manager->visualizeTrajectory(truncated_gt_traj,
                                         vtr::PlotType::GROUND_TRUTH);
      }

      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
          curr_frame_initial_feature_ests;
      for (const auto &cam_and_feats : observed_feats_for_frame) {
        for (const auto &feats_for_cam : cam_and_feats.second) {
          if (initial_feat_positions.find(feats_for_cam.first) !=
              initial_feat_positions.end()) {
            curr_frame_initial_feature_ests[feats_for_cam.first] =
                initial_feat_positions.at(feats_for_cam.first);
          }
        }
      }
      vis_manager->visualizeFeatureEstimates(curr_frame_initial_feature_ests,
                                             vtr::PlotType::INITIAL);
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
          curr_frame_est_feature_ests;
      for (const auto &cam_and_feats : observed_feats_for_frame) {
        for (const auto &feats_for_cam : cam_and_feats.second) {
          if (feature_ests.find(feats_for_cam.first) != feature_ests.end()) {
            curr_frame_est_feature_ests[feats_for_cam.first] =
                feature_ests.at(feats_for_cam.first);
          }
        }
      }
//      vis_manager->visualizeFeatureEstimates(curr_frame_est_feature_ests,
//                                             vtr::PlotType::ESTIMATED);
      vis_manager->visualizeFeatureEstimates(feature_ests,
                                             vtr::PlotType::ESTIMATED);
      vis_manager->publishTfsForLatestPose(
          est_trajectory_vec.back(), vtr::PlotType::ESTIMATED, extrinsics);
      vis_manager->publishTfsForLatestPose(
          init_trajectory_vec.back(), vtr::PlotType::INITIAL, extrinsics);

      save_to_file_visualizer.boundingBoxFrontEndVisualization(
          images,
          bounding_boxes_for_pending_object,
          num_obs_per_pending_obj,
          extrinsics,
          intrinsics,
          all_observed_corner_locations_with_uncertainty,
          observed_corner_locations,
          observed_features,
          img_heights_and_widths,
          min_frame_optimized,
          max_frame_optimized,
          pgo_opt);

      break;
    }
    case vtr::AFTER_ALL_POSTPROCESSING:
      if (!output_checkpoints_dir.empty()) {
        LOG(INFO) << "Dumping pose graph after all pose graph adjustments";
        MainPgPtr derived_pose_graph =
            std::dynamic_pointer_cast<MainPg>(pose_graph);
        vtr::outputPoseGraphToFile(
            derived_pose_graph,
            file_io::ensureDirectoryPathEndsWithSlash(
                FLAGS_output_checkpoints_dir) +
                vtr::kPostPostprocessingCheckpointOutputFileBaseName +
                file_io::kJsonExtension);
      }
      break;
    case vtr::AFTER_ALL_OPTIMIZATION:
      if (!output_checkpoints_dir.empty()) {
        LOG(INFO)
            << "Dumping pose graph after all data, before post processing";
        MainPgPtr derived_pose_graph =
            std::dynamic_pointer_cast<MainPg>(pose_graph);
        vtr::outputPoseGraphToFile(
            derived_pose_graph,
            file_io::ensureDirectoryPathEndsWithSlash(
                FLAGS_output_checkpoints_dir) +
                vtr::kPostFrameAddCheckpointOutputFileBaseName +
                file_io::kJsonExtension);
      }
      break;
    default:
      break;
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::optional<vtr::OptimizationLogger> opt_logger;
  if (FLAGS_logs_directory.empty()) {
    if (!FLAGS_disable_log_to_stderr) {
      FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
    }
  } else {
    if (!FLAGS_disable_log_to_stderr) {
      FLAGS_alsologtostderr = true;
    }
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

#ifdef RUN_TIMERS
  // Create an instance so that the factory never goes out of scope
  vtr::CumulativeTimerFactory &instance =
      vtr::CumulativeTimerFactory::getInstance();

  CumulativeFunctionTimer::Invocation full_opt_invoc(
      instance.getOrCreateFunctionTimer(vtr::kTimerNameFullTrajectoryExecution)
          .get());
#endif

  vtr::FullOVSLAMConfig config;
  vtr::readConfiguration(FLAGS_params_config_file, config);

  // Hard-coded values -----------------------------------------------------

  // TODO modify convergence thresholds
  pose_graph_optimization::OptimizationIterationParams
      local_ba_iteration_params = config.local_ba_iteration_params_;

  pose_graph_optimization::OptimizationIterationParams
      global_ba_iteration_params = config.global_ba_iteration_params_;

  pose_graph_optimization::OptimizationIterationParams
      final_opt_iteration_params = config.final_ba_iteration_params_;

  pose_graph_optimization::ObjectVisualPoseGraphResidualParams residual_params =
      config.object_visual_pose_graph_residual_params_;

  // Read necessary data in from file -----------------------------------------
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera =
          file_io::readCameraIntrinsicsByCameraFromFile(FLAGS_intrinsics_file);
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera =
          file_io::readCameraExtrinsicsByCameraFromFile(FLAGS_extrinsics_file);
  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  if (!FLAGS_bounding_boxes_by_timestamp_file.empty()) {
    if (std::filesystem::exists(FLAGS_bounding_boxes_by_timestamp_file)) {
      bounding_boxes = readBoundingBoxesByTimestampFromFile(
          FLAGS_bounding_boxes_by_timestamp_file,
          FLAGS_nodes_by_timestamp_file);
      //            readBoundingBoxesFromFile(FLAGS_bounding_boxes_by_node_id_file);
    } else {
      LOG(WARNING) << "Bounding box file was provided but does not exist "
                   << FLAGS_bounding_boxes_by_timestamp_file;
    }
  }

  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      file_io::readRobotPosesFromFile(FLAGS_poses_by_node_id_file);

  vtr::FrameId max_frame_id = vtr::getMaxFrame(robot_poses);

  LOG(INFO) << "Reading images from rosbag";
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(
          FLAGS_rosbag_file,
          FLAGS_nodes_by_timestamp_file,
          config.camera_info_.camera_topic_to_camera_id_);
  LOG(INFO) << "Done reading images from rosbag";

  MainLtmPtr long_term_map;
  if (!FLAGS_long_term_map_input.empty()) {
    cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
    vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
        //        std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>,
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
    }
  }

  vtr::SaveToFileVisualizerConfig save_to_file_visualizer_config;
  save_to_file_visualizer_config.bb_assoc_visualizer_config_
      .bounding_box_inflation_size_ =
      config.bounding_box_front_end_params_.feature_based_bb_association_params_
          .bounding_box_inflation_size_;
  save_to_file_visualizer_config.bb_assoc_visualizer_config_
      .feature_validity_window_ =
      config.bounding_box_front_end_params_.feature_based_bb_association_params_
          .feature_validity_window_;

  // Connect up functions needed for the optimizer --------------------------
  std::shared_ptr<vtr::RosVisualization> vis_manager =
      std::make_shared<vtr::RosVisualization>(
          node_handle, param_prefix, node_prefix);
  vtr::SaveToFileVisualizer save_to_file_visualizer(
      FLAGS_debug_images_output_directory, save_to_file_visualizer_config);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<util::EmptyStruct,
                                                           util::EmptyStruct>
      ltm_factor_creator(long_term_map);

  vtr::FrameId effective_max_frame_id = max_frame_id;
  if (config.limit_traj_eval_params_.should_limit_trajectory_evaluation_) {
    effective_max_frame_id =
        std::min(max_frame_id, config.limit_traj_eval_params_.max_frame_id_);
  }

  std::function<bool(
      const std::unordered_set<vtr::ObjectId> &,
      util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>> &)>
      long_term_map_factor_provider =
          [&](const std::unordered_set<vtr::ObjectId> &objects_to_include,
              util::BoostHashMap<MainFactorInfo,
                                 std::unordered_set<vtr::ObjectId>>
                  &factor_data) {
            return ltm_factor_creator.getFactorsToInclude(objects_to_include,
                                                          factor_data);
          };
  std::function<void(const MainProbData &, MainPgPtr &)> pose_graph_creator =
      std::bind(createPoseGraph,
                std::placeholders::_1,
                long_term_map_factor_provider,
                std::placeholders::_2);

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  std::function<bool(
      const vtr::FrameId &,
      const MainProbData &input_prob_data,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>> &)>
      bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
                         const MainProbData &input_prob_data,
                         std::unordered_map<vtr::CameraId,
                                            std::vector<vtr::RawBoundingBox>>
                             &bounding_boxes_by_cam) {
#ifdef RUN_TIMERS
        CumulativeFunctionTimer::Invocation invoc(
            vtr::CumulativeTimerFactory::getInstance()
                .getOrCreateFunctionTimer(vtr::kTimerNameBbQuerier)
                .get());
#endif
        if (vtr::retrievePrecomputedBoundingBoxes(frame_id_to_query_for,
                                                  input_prob_data,
                                                  bounding_boxes_by_cam)) {
          return true;

        } else {
#ifdef RUN_TIMERS
          CumulativeFunctionTimer::Invocation invoc(
              vtr::CumulativeTimerFactory::getInstance()
                  .getOrCreateFunctionTimer(vtr::kTimerNameFromYoloBbQuerier)
                  .get());
#endif
          return bb_querier.retrieveBoundingBoxes(
              frame_id_to_query_for, input_prob_data, bounding_boxes_by_cam);
        }
      };

  std::optional<std::vector<vtr::Pose3D<double>>> gt_trajectory =
      vtr::getGtTrajectory(max_frame_id,
                           FLAGS_ground_truth_trajectory_file,
                           FLAGS_ground_truth_extrinsics_file,
                           FLAGS_nodes_by_timestamp_file);

  std::function<void(
      const std::unordered_map<vtr::CameraId, std::pair<double, double>> &,
      const std::shared_ptr<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<vtr::CameraId,
                             std::vector<std::pair<vtr::BbCornerPair<double>,
                                                   std::optional<double>>>>>> &,
      const std::shared_ptr<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<
              vtr::CameraId,
              std::unordered_map<vtr::ObjectId,
                                 std::pair<vtr::BbCornerPair<double>,
                                           std::optional<double>>>>>> &,
      const std::shared_ptr<std::vector<std::unordered_map<
          vtr::FrameId,
          std::unordered_map<vtr::CameraId,
                             std::pair<vtr::BbCornerPair<double>, double>>>>> &,
      const std::shared_ptr<std::vector<
          std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
          &,
      const std::unordered_map<
          vtr::FrameId,
          std::unordered_map<
              vtr::CameraId,
              std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>> &,
      const MainProbData &,
      const MainPgPtr &,
      const vtr::FrameId &,
      const vtr::FrameId &,
      const vtr::VisualizationTypeEnum &,
      const int &)>
      visualization_callback =
          [&](const std::unordered_map<vtr::CameraId, std::pair<double, double>>
                  &img_heights_and_widths,
              const std::shared_ptr<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::vector<std::pair<vtr::BbCornerPair<double>,
                                            std::optional<double>>>>>>
                  &all_observed_corner_locations_with_uncertainty,
              const std::shared_ptr<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::unordered_map<vtr::ObjectId,
                                         std::pair<vtr::BbCornerPair<double>,
                                                   std::optional<double>>>>>>
                  &associated_observed_corner_locations,
              const std::shared_ptr<std::vector<std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::pair<vtr::BbCornerPair<double>, double>>>>>
                  &bounding_boxes_for_pending_object,
              const std::shared_ptr<std::vector<
                  std::pair<std::string,
                            std::optional<vtr::EllipsoidState<double>>>>>
                  &pending_objects,
              const std::unordered_map<
                  vtr::FrameId,
                  std::unordered_map<
                      vtr::CameraId,
                      std::unordered_map<vtr::FeatureId,
                                         vtr::PixelCoord<double>>>>
                  &low_level_features_map,
              const MainProbData &input_problem_data,
              const MainPgPtr &pose_graph,
              const vtr::FrameId &min_frame_id,
              const vtr::FrameId &max_frame_id_to_opt,
              const vtr::VisualizationTypeEnum &visualization_type,
              const int &attempt_num) {
            std::shared_ptr<vtr::ObjAndLowLevelFeaturePoseGraph<
                vtr::ReprojectionErrorFactor>>
                superclass_ptr = pose_graph;
            visualizationStub(
                vis_manager,
                save_to_file_visualizer,
                camera_extrinsics_by_camera,
                camera_intrinsics_by_camera,
                img_heights_and_widths,
                images,
                all_observed_corner_locations_with_uncertainty,
                associated_observed_corner_locations,
                bounding_boxes_for_pending_object,
                pending_objects,
                low_level_features_map,
                initial_feat_positions,
                input_problem_data,
                superclass_ptr,
                min_frame_id,
                max_frame_id_to_opt,
                visualization_type,
                config.bounding_box_covariance_generator_params_
                    .near_edge_threshold_,
                config.bounding_box_front_end_params_
                    .feature_based_bb_association_params_.min_observations_,
                gt_trajectory,
                effective_max_frame_id,
                FLAGS_output_checkpoints_dir,
                attempt_num);
          };
  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for global ba; review/fix your config";
    exit(1);
  }
  if ((!config.optimization_factors_enabled_params_
            .use_visual_features_on_final_global_ba_) &&
      (!config.optimization_factors_enabled_params_
            .use_pose_graph_on_final_global_ba_)) {
    LOG(ERROR) << "Must have either visual features or pose graph (or both) "
                  "for final global ba; review/fix your config";
    exit(1);
  }

  vtr::LongTermObjectMapAndResults<MainLtm> output_results;

  if (!runFullOptimization(opt_logger,
                           config,
                           camera_intrinsics_by_camera,
                           camera_extrinsics_by_camera,
                           bounding_boxes,
                           visual_features,
                           robot_poses,
                           long_term_map,
                           pose_graph_creator,
                           images,
                           FLAGS_output_checkpoints_dir,
                           FLAGS_ltm_opt_jacobian_info_directory,
                           bb_retriever,
                           visualization_callback,
                           ltm_factor_creator,
                           output_results)) {
    LOG(ERROR) << "Optimization failed";
  }

  if (!FLAGS_visual_feature_results_file.empty()) {
    cv::FileStorage visual_feature_fs(FLAGS_visual_feature_results_file,
                                      cv::FileStorage::WRITE);

    visual_feature_fs << "visual_feats"
                      << vtr::SerializableVisualFeatureResults(
                             output_results.visual_feature_results_);
    visual_feature_fs.release();
  }

  MainLtm output_long_term_map = output_results.long_term_map_;
  if (config.ltm_tunable_params_.fallback_to_prev_for_failed_extraction_) {
    vtr::EllipsoidResults ltm_ellipsoid_results;
    output_long_term_map.getEllipsoidResults(ltm_ellipsoid_results);
    if (ltm_ellipsoid_results.ellipsoids_.empty()) {
      LOG(ERROR) << "Long term map extraction failed; falling back to previous "
                    "long-term map if provided";
      if (long_term_map != nullptr) {
        output_long_term_map = *long_term_map;
      }
    }
  }

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs << "long_term_map"
             << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
                    util::EmptyStruct,
                    vtr::SerializableEmptyStruct>(output_long_term_map);
  ltm_out_fs.release();
  LOG(INFO) << "Num ellipsoids "
            << output_results.ellipsoid_results_.ellipsoids_.size();

  // TODO fix long term map
  // Output robot poses
  // Output ellipsoids

  if (!FLAGS_bb_associations_out_file.empty()) {
    cv::FileStorage bb_associations_out(FLAGS_bb_associations_out_file,
                                        cv::FileStorage::WRITE);
    vtr::ObjectDataAssociationResults data_assoc_results;
    data_assoc_results.ellipsoid_pose_results_ =
        output_results.ellipsoid_results_;
    data_assoc_results.associated_bounding_boxes_ =
        *(output_results.associated_observed_corner_locations_);
    bb_associations_out << "bounding_box_associations"
                        << vtr::SerializableObjectDataAssociationResults(
                               data_assoc_results);
    bb_associations_out.release();
  }

  if (!FLAGS_ellipsoids_results_file.empty()) {
    vtr::writeEllipsoidResults(FLAGS_ellipsoids_results_file,
                               output_results.ellipsoid_results_);
  }

  if (!FLAGS_robot_poses_results_file.empty()) {
    vtr::writeRobotPoseResults(FLAGS_robot_poses_results_file,
                               output_results.robot_pose_results_);
  }

  return 0;
}
