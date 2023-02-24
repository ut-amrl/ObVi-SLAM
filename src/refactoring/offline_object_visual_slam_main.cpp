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
#include <file_io/pose_3d_with_node_id_io.h>
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
#include <refactoring/visualization/ros_visualization.h>
#include <refactoring/visualization/save_to_file_visualizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

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

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
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

    //    vtr::Orientation3D<double> extrinsics_orient(
    //        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
    //                           extrinsics_for_cam.quat_x,
    //                           extrinsics_for_cam.quat_y,
    //                           extrinsics_for_cam.quat_z) *
    //        extrinsics_orientation_switch_to_cam);
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

std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(const MainProbData &input_problem_data,
                          const MainPgPtr &pose_graph,
                          const vtr::FrameId &min_frame_optimized,
                          const vtr::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(const MainFactorInfo &factor,
                        const MainPgPtr &,
                        const util::EmptyStruct &) {
  return false;
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
    const size_t &pending_obj_min_obs_threshold) {
  switch (visualization_stage) {
    case vtr::BEFORE_ANY_OPTIMIZATION:
      vis_manager->publishTransformsForEachCamera(
          input_problem_data.getMaxFrameId(),
          input_problem_data.getRobotPoseEstimates(),
          input_problem_data.getCameraExtrinsicsByCamera());

      sleep(3);
      break;
    case vtr::BEFORE_EACH_OPTIMIZATION:
      break;
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
      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::EllipsoidState<double>>>
          ltm_ellipsoids;
      if (input_problem_data.getLongTermObjectMap() != nullptr) {
        vtr::EllipsoidResults ellipsoids_in_map;
        input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
            ellipsoids_in_map);
        for (const auto &ltm_ellipsoid : ellipsoids_in_map.ellipsoids_) {
          ltm_ellipsoids[ltm_ellipsoid.first] = ltm_ellipsoid.second;
        }
      }
      vis_manager->visualizeEllipsoids(ltm_ellipsoids, vtr::INITIAL, false);

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
      vis_manager->publishTfForLatestPose(est_trajectory_vec.back(),
                                          vtr::PlotType::ESTIMATED);
      vis_manager->publishTfForLatestPose(init_trajectory_vec.back(),
                                          vtr::PlotType::INITIAL);

      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests;
      std::unordered_map<
          vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
          observed_feats_for_frame = observed_features.at(max_frame_optimized);
      pose_graph->getVisualFeatureEstimates(feature_ests);
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
      vis_manager->visualizeFeatureEstimates(curr_frame_est_feature_ests,
                                             vtr::PlotType::ESTIMATED);
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
          max_frame_optimized);

      break;
    }
    case vtr::AFTER_ALL_OPTIMIZATION:
      break;
    default:
      break;
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_logs_directory.empty()) {
    FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  } else {
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = FLAGS_logs_directory;
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

  // Read necessary data in from file -----------------------------------------
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
  if (!FLAGS_bounding_boxes_by_node_id_file.empty()) {
    init_bounding_boxes =
        readBoundingBoxesFromFile(FLAGS_bounding_boxes_by_node_id_file);
  }
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  // tODO fix this
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

  //  LOG(INFO) << "Bounding boxes for " << bounding_boxes.size() << " frames ";
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      readRobotPosesFromFile(FLAGS_poses_by_node_id_file);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(
          FLAGS_rosbag_file,
          FLAGS_nodes_by_timestamp_file,
          config.camera_info_.camera_topic_to_camera_id_);

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

  MainProbData input_problem_data(
      camera_intrinsics_by_camera,
      camera_extrinsics_by_camera,
      visual_features,
      robot_poses,
      config.shape_dimension_priors_.mean_and_cov_by_semantic_class_,
      bounding_boxes,
      long_term_map,
      images);

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
  //    vtr::RawEllipsoid<double> ellipsoid;
  //    ellipsoid << -0.164291, 0.41215, -0.0594742, 79.9495, 209.015, 248.223,
  //        0.432929, 0.450756, 2.05777;
  //    vtr::RawPose3d<double> robot_pose;
  //    robot_pose << 0.135177, -0.000860353, 0.0109102, 0.00145096,
  //    -0.000676748,
  //        -0.00533544;
  //    vtr::EllipsoidState<double> publishable_ellipsoid =
  //        vtr::convertToEllipsoidState(ellipsoid);
  //    vtr::Pose3D<double> publishable_robot_pose =
  //    vtr::convertToPose3D(robot_pose);
  //    vis_manager->visualizeEllipsoids({{1,std::make_pair("chair",
  //    publishable_ellipsoid)}},
  //                                     vtr::PlotType::INITIAL, false);
  //    for (const auto &extrinsics_entry : camera_extrinsics_by_camera) {
  //      LOG(INFO) << "Publishing transforms for camera " <<
  //      extrinsics_entry.first; vis_manager->publishTransformsForEachCamera(
  //          0, {{0, publishable_robot_pose}}, camera_extrinsics_by_camera,
  //          "init_");
  //      LOG(INFO) << "Publishing empty image for camera " <<
  //      extrinsics_entry.first;
  //      vis_manager->publishLatestImageWithReprojectionResiduals(
  //          0,
  //          extrinsics_entry.first,
  //          camera_intrinsics_by_camera.at(extrinsics_entry.first),
  //          vtr::PlotType::INITIAL,
  //          {},
  //          {},
  //          std::nullopt,
  //          img_heights_and_widths.at(extrinsics_entry.first),
  //          true);
  //      vis_manager->visualizeFrustum(publishable_robot_pose,
  //                            camera_intrinsics_by_camera.at(extrinsics_entry.first),
  //                            extrinsics_entry.second,
  //                            img_heights_and_widths.at(extrinsics_entry.first),
  //                            vtr::PlotType::INITIAL);
  //      LOG(INFO) << "Done with camera " << extrinsics_entry.first;
  //    }
  //    ros::Duration(2).sleep();
  //    exit(1);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<
      util::EmptyStruct,
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_factor_creator(long_term_map);

  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  vtr::FrameId max_frame_id = 0;
  for (const auto &frame_and_pose : robot_poses) {
    max_frame_id = std::max(max_frame_id, frame_and_pose.first);
  }

  std::function<vtr::FrameId(const vtr::FrameId &)> window_provider_func =
      [&](const vtr::FrameId &max_frame_to_opt) -> vtr::FrameId {
    // Optimize the full trajectory when we're on the last frame
    if (max_frame_to_opt == max_frame_id) {
      return 0;
    }
    if ((max_frame_to_opt %
         config.sliding_window_params_.global_ba_frequency_) == 0) {
      return 0;
    }
    if (max_frame_to_opt <
        config.sliding_window_params_.local_ba_window_size_) {
      return 0;
    }
    return max_frame_to_opt -
           config.sliding_window_params_.local_ba_window_size_;
  };
  std::function<bool(const vtr::FrameId &)> gba_checker =
      [&](const vtr::FrameId &max_frame_to_opt) -> bool {
    vtr::FrameId min_frame_to_opt = window_provider_func(max_frame_to_opt);
    if (max_frame_to_opt - min_frame_to_opt <=
        config.sliding_window_params_.local_ba_window_size_) {
      return false;
    }
    return true;
  };
  std::function<pose_graph_optimization::OptimizationSolverParams(
      const vtr::FrameId &)>
      solver_params_provider_func = [&](const vtr::FrameId &max_frame_to_opt)
      -> pose_graph_optimization::OptimizationSolverParams {
    if (max_frame_to_opt == max_frame_id) {
      return final_opt_solver_params;
    } else if ((max_frame_to_opt %
                config.sliding_window_params_.global_ba_frequency_) == 0) {
      return global_ba_solver_params;
    } else {
      return local_ba_solver_params;
    }
  };

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)>
      cached_info_creator = [](const MainFactorInfo &factor_info,
                               const MainPgPtr &pose_graph,
                               util::EmptyStruct &cached_info) {
        return true;  // TODO maybe fill in with real info some day
      };
  std::function<bool(
      const MainFactorInfo &,
      const MainPgPtr &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::function<bool(
          const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      long_term_map_residual_creator_func =
          [&](const MainFactorInfo &factor_info,
              const MainPgPtr &pose_graph,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &residual_params,
              const std::function<bool(const MainFactorInfo &,
                                       const MainPgPtr &,
                                       util::EmptyStruct &)> &cached_inf_create,
              ceres::Problem *problem,
              ceres::ResidualBlockId &res_id,
              util::EmptyStruct &cached_inf) {
            return ltm_factor_creator.createResidual(factor_info,
                                                     pose_graph,
                                                     residual_params,
                                                     cached_inf_create,
                                                     problem,
                                                     res_id,
                                                     cached_inf);
          };
  std::function<bool(
      const MainFactorInfo &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const MainPgPtr &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return vtr::createResidual(factor_id,
                                       pose_graph,
                                       solver_residual_params,
                                       cached_info_creator,
                                       long_term_map_residual_creator_func,
                                       problem,
                                       residual_id,
                                       cached_info);
          };

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

  std::function<double(const MainProbData &,
                       const MainPgPtr &,
                       const vtr::FrameId &,
                       const vtr::FeatureId &,
                       const vtr::CameraId &)>
      reprojection_error_provider = [&](const MainProbData &input_problem,
                                        const MainPgPtr &pose_graph,
                                        const vtr::FrameId &frame_id,
                                        const vtr::FeatureId &feature_id,
                                        const vtr::CameraId &camera_id) {
        // Probably need to do something more sophisticated here --
        // ORB-SLAM has a more advanced thing that I haven't looked
        // into
        return config.visual_feature_params_.reprojection_error_std_dev_;
      };
  vtr::VisualFeatureFrontend<MainProbData> visual_feature_fronted(
      gba_checker,
      reprojection_error_provider,
      config.visual_feature_params_
          .min_visual_feature_parallax_pixel_requirement_,
      config.visual_feature_params_
          .min_visual_feature_parallax_robot_transl_requirement_,
      config.visual_feature_params_
          .min_visual_feature_parallax_robot_orient_requirement_,
      config.visual_feature_params_.enforce_min_pixel_parallax_requirement_,
      config.visual_feature_params_
          .enforce_min_robot_pose_parallax_requirement_);
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &)>
      visual_feature_frame_data_adder = [&](const MainProbData &input_problem,
                                            const MainPgPtr &pose_graph,
                                            const vtr::FrameId &min_frame_id,
                                            const vtr::FrameId &max_frame_id) {
        visual_feature_fronted.addVisualFeatureObservations(
            input_problem_data, pose_graph, min_frame_id, max_frame_id);
      };

  //  std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>
  //      long_term_map_front_end_data;
  std::unordered_map<vtr::ObjectId, util::EmptyStruct>
      long_term_map_front_end_data;
  if (long_term_map != nullptr) {
    long_term_map->getFrontEndObjMapData(long_term_map_front_end_data);
  }
  std::shared_ptr<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::unordered_map<vtr::ObjectId,
                                            std::pair<vtr::BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      associated_observed_corner_locations =
          std::make_shared<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::unordered_map<vtr::ObjectId,
                                     std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>>();

  std::shared_ptr<std::vector<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::pair<vtr::BbCornerPair<double>, double>>>>>
      bounding_boxes_for_pending_object =
          std::make_shared<std::vector<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::pair<vtr::BbCornerPair<double>, double>>>>>();
  std::shared_ptr<std::vector<
      std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
      pending_objects = std::make_shared<
          std::vector<std::pair<std::string,
                                std::optional<vtr::EllipsoidState<double>>>>>();

  //    std::function<std::pair<bool,
  //    std::optional<sensor_msgs::Image::ConstPtr>>(
  //        const vtr::FrameId &, const vtr::CameraId &, const MainProbData &)>
  //        bb_context_retriever = [](const vtr::FrameId &frame_id,
  //                                  const vtr::CameraId &camera_id,
  //                                  const MainProbData &problem_data) {
  //      //            LOG(INFO) << "Getting image for frame " << frame_id
  //      //                      << " and camera " << camera_id;
  //      std::optional<sensor_msgs::Image::ConstPtr> image =
  //          problem_data.getImageForFrameAndCamera(frame_id, camera_id);
  //      return std::make_pair(image.has_value(), image);
  //    };
  //    vtr::RoshanBbFrontEndCreator<vtr::ReprojectionErrorFactor>
  //        roshan_associator_creator = vtr::generateRoshanBbCreator(
  //            associated_observed_corner_locations,
  //            all_observed_corner_locations_with_uncertainty,
  //            img_heights_and_widths,
  //            cov_gen_params,
  //            long_term_map_front_end_data);
  //    std::function<std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
  //        vtr::ReprojectionErrorFactor,
  //        vtr::RoshanAggregateBbInfo,
  //        std::optional<sensor_msgs::Image::ConstPtr>,
  //        vtr::RoshanImageSummaryInfo,
  //        vtr::RoshanBbInfo,
  //        std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>>(
  //        const MainPgPtr &, const MainProbData &)>
  //        bb_associator_retriever =
  //        [&](const MainPgPtr &pg, const MainProbData &input_prob) {
  //          return roshan_associator_creator.getDataAssociator(pg);
  //        };
  std::function<std::pair<bool, vtr::FeatureBasedContextInfo>(
      const vtr::FrameId &, const vtr::CameraId &, const MainProbData &)>
      bb_context_retriever = [&](const vtr::FrameId &frame_id,
                                 const vtr::CameraId &camera_id,
                                 const MainProbData &problem_data) {
        vtr::FeatureBasedContextInfo context;
        bool success = false;
        if (low_level_features_map.find(frame_id) !=
            low_level_features_map.end()) {
          if (low_level_features_map.at(frame_id).find(camera_id) !=
              low_level_features_map.at(frame_id).end()) {
            context.observed_features_ =
                low_level_features_map.at(frame_id).at(camera_id);
            success = true;
          }
        }
        return std::make_pair(success, context);
      };
  vtr::FeatureBasedBoundingBoxFrontEndCreator<vtr::ReprojectionErrorFactor>
      feature_based_associator_creator = vtr::generateFeatureBasedBbCreator(
          associated_observed_corner_locations,
          all_observed_corner_locations_with_uncertainty,
          bounding_boxes_for_pending_object,
          pending_objects,
          img_heights_and_widths,
          config.bounding_box_covariance_generator_params_,
          config.bounding_box_front_end_params_
              .geometric_similarity_scorer_params_,
          config.bounding_box_front_end_params_
              .feature_based_bb_association_params_,
          long_term_map_front_end_data);
  std::function<std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
      vtr::ReprojectionErrorFactor,
      vtr::FeatureBasedFrontEndObjAssociationInfo,
      vtr::FeatureBasedFrontEndPendingObjInfo,
      vtr::FeatureBasedContextInfo,
      vtr::FeatureBasedContextInfo,
      vtr::FeatureBasedSingleBbContextInfo,
      util::EmptyStruct>>(const MainPgPtr &, const MainProbData &)>
      bb_associator_retriever =
          [&](const MainPgPtr &pg, const MainProbData &input_prob) {
            return feature_based_associator_creator.getDataAssociator(pg);
          };

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  std::function<bool(
      const vtr::FrameId &,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>> &)>
      bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
                         std::unordered_map<vtr::CameraId,
                                            std::vector<vtr::RawBoundingBox>>
                             &bounding_boxes_by_cam) {
        return bb_querier.retrieveBoundingBoxes(
            frame_id_to_query_for, input_problem_data, bounding_boxes_by_cam);
      };
  //  std::function<bool(
  //      const vtr::FrameId &,
  //      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>
  //      &)> bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
  //                         std::unordered_map<vtr::CameraId,
  //                                            std::vector<vtr::RawBoundingBox>>
  //                             &bounding_boxes_by_cam) {
  //        return vtr::retrievePrecomputedBoundingBoxes(
  //            frame_id_to_query_for, input_problem_data,
  //            bounding_boxes_by_cam);
  //      };
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &)>
      frame_data_adder = [&](const MainProbData &problem_data,
                             const MainPgPtr &pose_graph,
                             const vtr::FrameId &min_frame_id,
                             const vtr::FrameId &frame_to_add) {
        vtr::addFrameDataAssociatedBoundingBox(problem_data,
                                               pose_graph,
                                               min_frame_id,
                                               frame_to_add,
                                               reprojection_error_provider,
                                               visual_feature_frame_data_adder,
                                               bb_retriever,
                                               bb_associator_retriever,
                                               bb_context_retriever);
      };

  //  std::function<void(
  //      const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //          vtr::StructuredVisionFeatureTrack,
  //          sensor_msgs::Image::ConstPtr> &,
  //      const std::shared_ptr<const
  //      MainPg> &, ceres::Problem *,
  //      vtr::SpatialEstimateOnlyResults &)>
  //      output_data_extractor =
  //          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //                 vtr::StructuredVisionFeatureTrack,
  //                 sensor_msgs::Image::ConstPtr> &input_problem_data,
  //             const std::shared_ptr<
  //                 const MainPg>
  //                 &pose_graph,
  //             ceres::Problem *problem,
  //             vtr::SpatialEstimateOnlyResults &output_problem_data) {
  //            vtr::extractSpatialEstimateOnlyResults(pose_graph,
  //                                                   output_problem_data);
  //          };

  vtr::CovarianceExtractorParams ltm_covariance_params;

  // TODO maybe replace params with something that will yield more accurate
  // results
  std::function<bool(
      const vtr::FactorType &, const vtr::FeatureFactorId &, vtr::ObjectId &)>
      long_term_map_obj_retriever = [&](const vtr::FactorType &factor_type,
                                        const vtr::FeatureFactorId &factor_id,
                                        vtr::ObjectId &object_id) {
        std::unordered_set<vtr::ObjectId> obj_ids;
        if (!ltm_factor_creator.getObjectIdsForFactor(
                factor_type, factor_id, obj_ids)) {
          return false;
        }
        if (obj_ids.size() != 1) {
          LOG(ERROR) << "This only works with factors that have one object";
          exit(1);
        }
        object_id = *obj_ids.begin();
        return true;
      };

  vtr::IndependentEllipsoidsLongTermObjectMapExtractor<
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_extractor(ltm_covariance_params,
                    residual_creator,
                    long_term_map_obj_retriever,
                    config.ltm_tunable_params_,
                    config.ltm_solver_residual_params_,
                    config.ltm_solver_params_);

  std::function<void(
      const MainProbData &,
      const MainPgPtr &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      vtr::LongTermObjectMapAndResults<MainLtm> &)>
      output_data_extractor =
          [&](const MainProbData &input_problem_data,
              const MainPgPtr &pose_graph,
              const pose_graph_optimizer::OptimizationFactorsEnabledParams
                  &optimization_factors_enabled_params,
              vtr::LongTermObjectMapAndResults<MainLtm> &output_problem_data) {
            //                    std::function<bool(
            //                        std::unordered_map<vtr::ObjectId,
            //                        vtr::RoshanAggregateBbInfo> &)>
            //                        front_end_map_data_extractor =
            //                            [&](std::unordered_map<vtr::ObjectId,
            //                                                   vtr::RoshanAggregateBbInfo>
            //                                    &front_end_data) {
            //                              roshan_associator_creator.getDataAssociator(pose_graph)
            //                                  ->getFrontEndObjMapData(front_end_data);
            //                              return true;
            //                            };
            std::function<bool(
                std::unordered_map<vtr::ObjectId, util::EmptyStruct> &)>
                front_end_map_data_extractor =
                    [&](std::unordered_map<vtr::ObjectId, util::EmptyStruct>
                            &front_end_data) {
                      feature_based_associator_creator
                          .getDataAssociator(pose_graph)
                          ->getFrontEndObjMapData(front_end_data);
                      return true;
                    };

            std::function<bool(
                const MainPgPtr &,
                const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
                MainLtm &)>
                long_term_object_map_extractor =
                    [&](const MainPgPtr &ltm_pose_graph,
                        const pose_graph_optimizer::
                            OptimizationFactorsEnabledParams
                                &ltm_optimization_factors_enabled_params,
                        MainLtm &ltm_extractor_out) {
                      return ltm_extractor.extractLongTermObjectMap(
                          ltm_pose_graph,
                          ltm_optimization_factors_enabled_params,
                          front_end_map_data_extractor,
                          FLAGS_ltm_opt_jacobian_info_directory,
                          ltm_extractor_out);
                    };
            vtr::extractLongTermObjectMapAndResults(
                pose_graph,
                optimization_factors_enabled_params,
                long_term_object_map_extractor,
                output_problem_data);
          };

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const MainProbData &,
      const MainPgPtr &,
      const vtr::FrameId &,
      const vtr::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &,
                     const vtr::VisualizationTypeEnum &)>
      visualization_callback = [&](const MainProbData &input_problem_data,
                                   const MainPgPtr &pose_graph,
                                   const vtr::FrameId &min_frame_id,
                                   const vtr::FrameId &max_frame_id,
                                   const vtr::VisualizationTypeEnum
                                       &visualization_type) {
        std::shared_ptr<
            vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
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
            max_frame_id,
            visualization_type,
            config.bounding_box_covariance_generator_params_
                .near_edge_threshold_,
            config.bounding_box_front_end_params_
                .feature_based_bb_association_params_.min_observations_);
      };
  vtr::OfflineProblemRunner<MainProbData,
                            vtr::ReprojectionErrorFactor,
                            vtr::LongTermObjectMapAndResults<MainLtm>,
                            util::EmptyStruct,
                            MainPg>
      offline_problem_runner(residual_params,
                             config.limit_traj_eval_params_,
                             continue_opt_checker,
                             window_provider_func,
                             refresh_residual_checker,
                             residual_creator,
                             pose_graph_creator,
                             frame_data_adder,
                             output_data_extractor,
                             ceres_callback_creator,
                             visualization_callback,
                             solver_params_provider_func);

  //  vtr::SpatialEstimateOnlyResults output_results;
  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  offline_problem_runner.runOptimization(
      input_problem_data,
      config.optimization_factors_enabled_params_,
      output_results);

  if (!FLAGS_visual_feature_results_file.empty()) {
    cv::FileStorage visual_feature_fs(FLAGS_visual_feature_results_file,
                                      cv::FileStorage::WRITE);

    visual_feature_fs << "visual_feats"
                      << vtr::SerializableVisualFeatureResults(
                             output_results.visual_feature_results_);
    visual_feature_fs.release();
    //    for (const auto &feat_and_pos :
    //         output_results.visual_feature_results_.visual_feature_positions_)
    //         {
    //      if (feat_and_pos.second.norm() > 10000) {
    //        LOG(WARNING) << "High norm for feature " << feat_and_pos.first <<
    //        ": "
    //                     << feat_and_pos.second.norm();
    //      }
    //    }
  }

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs
      << "long_term_map"
      << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
             //             std::unordered_map<vtr::ObjectId,
             //             vtr::RoshanAggregateBbInfo>,
             //             vtr::SerializableMap<vtr::ObjectId,
             //                                  vtr::SerializableObjectId,
             //                                  vtr::RoshanAggregateBbInfo,
             //                                  vtr::SerializableRoshanAggregateBbInfo>>(
             util::EmptyStruct,
             vtr::SerializableEmptyStruct>(output_results.long_term_map_);
  ltm_out_fs.release();
  //  LOG(INFO) << "Num ellipsoids "
  //            << output_results.ellipsoid_results_.ellipsoids_.size();

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
        *associated_observed_corner_locations;
    bb_associations_out << "bounding_box_associations"
                        << vtr::SerializableObjectDataAssociationResults(
                               data_assoc_results);
    bb_associations_out.release();
  }

  if (!FLAGS_ellipsoids_results_file.empty()) {
    cv::FileStorage ellipsoids_results_out(FLAGS_ellipsoids_results_file,
                                           cv::FileStorage::WRITE);
    vtr::EllipsoidResults ellipsoid_results = output_results.ellipsoid_results_;
    ellipsoids_results_out << "ellipsoids"
                           << vtr::SerializableEllipsoidResults(
                                  ellipsoid_results);
    ellipsoids_results_out.release();
  }

  if (!FLAGS_robot_poses_results_file.empty()) {
    vtr::writeRobotPoseResults(FLAGS_robot_poses_results_file,
                               output_results.robot_pose_results_);
  }

  return 0;
}