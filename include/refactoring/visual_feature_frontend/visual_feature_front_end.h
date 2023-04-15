//
// Created by taijing on 2/07/22.
//
#ifndef UT_VSLAM_VISUAL_FEATURE_FRONT_END_H
#define UT_VSLAM_VISUAL_FEATURE_FRONT_END_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

// double getNormalizedEpipolarError(
//     const CameraIntrinsicsMat<double> &intrinsics1,
//     const CameraIntrinsicsMat<double> &intrinsics2,
//     const CameraExtrinsics<double> &extrinsics1,
//     const CameraExtrinsics<double> &extrinsics2,
//     const PixelCoord<double> &feature_pixel1,
//     const PixelCoord<double> &feature_pixel2,
//     const Pose3D<double> &pose1,
//     const Pose3D<double> &pose2) {
//   Eigen::Affine3d cam_to_robot_tf_1 =
//       Eigen::Translation3d(extrinsics1.transl_) * extrinsics1.orientation_;
//   Eigen::Affine3d cam_to_robot_tf_2 =
//       Eigen::Translation3d(extrinsics2.transl_) * extrinsics2.orientation_;

//   Pose3D<double> normalized_pose1(pose1.transl_.normalized(),
//                                   pose1.orientation_);
//   Pose3D<double> normalized_pose2(pose2.transl_.normalized(),
//                                   pose2.orientation_);
//   RawPose3d<double> raw_normalized_pose1, raw_normalized_pose2;
//   convertPoseToArray(normalized_pose1, raw_normalized_pose1);
//   convertPoseToArray(normalized_pose2, raw_normalized_pose2);
//   Eigen::Matrix3d E = CalcEssentialMatrix(raw_normalized_pose1.data(),
//                                           raw_normalized_pose2.data(),
//                                           cam_to_robot_tf_1,
//                                           cam_to_robot_tf_2);
//   Eigen::Vector3d normalized_x1 =
//       intrinsics1.inverse() * pixelToHomogeneous(feature_pixel1);
//   Eigen::Vector3d normalized_x2 =
//       intrinsics2.inverse() * pixelToHomogeneous(feature_pixel2);
//   return std::fabs(normalized_x2.transpose() * E * normalized_x1);
// }

// Adapted from CalculateEpipolarErrorVec from IV_SLAM (feature_evaluator.cpp)
// https://github.com/ut-amrl/IV_SLAM/blob/main/introspective_ORB_SLAM/src/feature_evaluator.cpp#L2754
Eigen::Vector2d getNormalizedEpipolarErrorVec(
    const CameraIntrinsicsMat<double> &intrinsics1,
    const CameraIntrinsicsMat<double> &intrinsics2,
    const CameraExtrinsics<double> &extrinsics1,
    const CameraExtrinsics<double> &extrinsics2,
    const PixelCoord<double> &feature_pixel1,
    const PixelCoord<double> &feature_pixel2,
    const Pose3D<double> &pose1,
    const Pose3D<double> &pose2,
    Eigen::Vector2d *epipole_ptr = nullptr,
    Eigen::Vector2d *x1_in2_2d_ptr = nullptr) {
  Eigen::Affine3d cam_to_robot_tf_1 =
      Eigen::Translation3d(extrinsics1.transl_) * extrinsics1.orientation_;
  Eigen::Affine3d cam_to_robot_tf_2 =
      Eigen::Translation3d(extrinsics2.transl_) * extrinsics2.orientation_;
  Eigen::Affine3d robot_current_to_world_1 = convertToAffine(pose1);
  Eigen::Affine3d robot_current_to_world_2 = convertToAffine(pose2);
  Eigen::Affine3d cam_current_to_world_1 =
      robot_current_to_world_1 * cam_to_robot_tf_1;
  Eigen::Affine3d cam_current_to_world_2 =
      robot_current_to_world_2 * cam_to_robot_tf_2;
  // not using getPose2RelativeToPose1 to avoid redundant type conversion
  // overhead; world_to_cam2 * cam1_to_world = cam1_to_cam2
  Eigen::Affine3d cam1_to_cam2 =
      cam_current_to_world_2.inverse() * cam_current_to_world_1;

  Eigen::Vector3d c1_in1_3d(0, 0, 0);
  Eigen::Vector3d c1_in2_3d = cam1_to_cam2 * c1_in1_3d;
  Eigen::Vector3d homogeneous_epipole = intrinsics2 * c1_in2_3d;
  Eigen::Vector2d epipole(homogeneous_epipole.x() / homogeneous_epipole.z(),
                          homogeneous_epipole.y() / homogeneous_epipole.z());

  Eigen::Vector3d normalized_x1 =
      intrinsics1.inverse() * pixelToHomogeneous(feature_pixel1);
  Eigen::Vector3d x1_in2_3d = cam1_to_cam2 * normalized_x1;
  Eigen::Vector3d homogeneous_x1_in2_2d = intrinsics2 * x1_in2_3d;
  Eigen::Vector2d x1_in2_2d(
      homogeneous_x1_in2_2d.x() / homogeneous_x1_in2_2d.z(),
      homogeneous_x1_in2_2d.y() / homogeneous_x1_in2_2d.z());

  // Unit vector of the epipolar line
  Eigen::Vector2d u_hat = (x1_in2_2d - epipole).normalized();

  // Find the projection vector of feature_pixel2 on the epipolar line
  Eigen::Vector2d x2_epipolar_projection =
      epipole + (feature_pixel2 - epipole).dot(u_hat) * u_hat;
  // Eigen::Vector2d x2_epipolar_projection = epipole + u_hat;

  if (epipole_ptr != nullptr) {
    *epipole_ptr = epipole;
  }
  if (x1_in2_2d_ptr != nullptr) {
    *x1_in2_2d_ptr = x1_in2_2d;
  }

  return x2_epipolar_projection - feature_pixel2;
}

// TODO need to change ReprojectionErrorFactor to shared_ptrs to avoid redundant
// data copies
// TODO need to change it to a class so that variables like inlier_factors_ can
// be hidden
struct VisualFeatureCachedInfo {
  bool is_cache_cleaned_;
  std::map<FrameId, std::vector<ReprojectionErrorFactor>>
      frame_ids_and_reprojection_err_factors_;
  std::map<FrameId, std::optional<Pose3D<double>>> frame_ids_and_poses_;

  // std::vector<ReprojectionErrorFactor> inlier_factors_;

  VisualFeatureCachedInfo() : is_cache_cleaned_(false) {}

  void addFactorsAndRobotPose(
      const FrameId &frame_id,
      const std::vector<ReprojectionErrorFactor> &reprojection_err_factors,
      const std::optional<Pose3D<double>> &robot_pose) {
    frame_ids_and_reprojection_err_factors_[frame_id] =
        reprojection_err_factors;
    frame_ids_and_poses_[frame_id] = robot_pose;
  }

  FrameId getMinFrameId() const {
    return frame_ids_and_reprojection_err_factors_.begin()->first;
  }

  std::vector<FrameId> getOrderedFrameIdsGreaterThan(
      const FrameId &min_frame_id) const {
    std::vector<FrameId> frame_ids;
    for (auto iter =
             frame_ids_and_reprojection_err_factors_.lower_bound(min_frame_id);
         iter != frame_ids_and_reprojection_err_factors_.end();
         ++iter) {
      frame_ids.emplace_back(iter->first);
    }
    return frame_ids;
  }

  std::unordered_map<CameraId, PixelCoord<double>> getCamIdsAndPixelsByFrame(
      const FrameId &frame_id) const {
    std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels;
    for (const auto &factor :
         frame_ids_and_reprojection_err_factors_.at(frame_id)) {
      cam_ids_and_pixels[factor.camera_id_] = factor.feature_pos_;
    }
    return cam_ids_and_pixels;
  }
};

// TODO support general pose graph
template <typename ProblemDataType>
class VisualFeatureFrontend {
 public:
  VisualFeatureFrontend(
      const std::function<bool(const FrameId &)> &gba_checker,
      const std::function<
          double(const ProblemDataType &,
                 const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
                 const FrameId &,
                 const FeatureId &,
                 const CameraId &)> &reprojection_error_provider,
      const double &min_visual_feature_parallax_pixel_requirement,
      const double &min_visual_feature_parallax_robot_transl_requirement,
      const double &min_visual_feature_parallax_robot_orient_requirement,
      const bool enforce_min_pixel_parallax_requirement,
      const bool enforce_min_robot_pose_parallax_requirement,
      const double &inlier_epipolar_err_thresh)
      : gba_checker_(gba_checker),
        reprojection_error_provider_(reprojection_error_provider),
        min_visual_feature_parallax_pixel_requirement_(
            min_visual_feature_parallax_pixel_requirement),
        min_visual_feature_parallax_robot_transl_requirement_(
            min_visual_feature_parallax_robot_transl_requirement),
        min_visual_feature_parallax_robot_orient_requirement_(
            min_visual_feature_parallax_robot_orient_requirement),
        enforce_min_pixel_parallax_requirement_(
            enforce_min_pixel_parallax_requirement),
        enforce_min_robot_pose_parallax_requirement_(
            enforce_min_robot_pose_parallax_requirement),
        inlier_epipolar_err_thresh_(inlier_epipolar_err_thresh) {}

  /**
   * @brief
   *
   * @param min_frame_id
   * @param max_frame_id assume this is the same as frame_to_add
   * @param pose_graph
   * @param input_problem_data
   */
  void addVisualFeatureObservations(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id) {
    std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        visual_features = input_problem_data.getVisualFeatures();

    for (const auto &feat_id_to_track : visual_features) {
      FeatureId feature_id = feat_id_to_track.first;
      StructuredVisionFeatureTrack feature_track = feat_id_to_track.second;
      if (feature_track.feature_track.feature_observations_.find(
              max_frame_id) ==
          feature_track.feature_track.feature_observations_.end()) {
        continue;
      }
      VisionFeature feature =
          feature_track.feature_track.feature_observations_.at(max_frame_id);
      if (feature.frame_id_ != max_frame_id) {
        continue;
      }
      bool is_feature_added_to_pose_graph = false;
      if (added_feature_ids_.find(feature_id) != added_feature_ids_.end()) {
        is_feature_added_to_pose_graph = true;
      }
      std::vector<ReprojectionErrorFactor> reprojection_error_factors;
      for (const auto &obs_by_camera : feature.pixel_by_camera_id) {
        ReprojectionErrorFactor vis_factor;
        vis_factor.camera_id_ = obs_by_camera.first;
        vis_factor.feature_id_ = feature_id;
        vis_factor.frame_id_ = feature.frame_id_;
        vis_factor.feature_pos_ = obs_by_camera.second;
        vis_factor.reprojection_error_std_dev_ =
            reprojection_error_provider_(input_problem_data,
                                         pose_graph,
                                         max_frame_id,
                                         vis_factor.feature_id_,
                                         vis_factor.camera_id_);
        reprojection_error_factors.push_back(vis_factor);
      }
      if (is_feature_added_to_pose_graph) {
        for (const auto &vis_factor : reprojection_error_factors) {
          pose_graph->addVisualFactor(vis_factor);
        }
      } else {
        // add reprojection factors and initial poses to pendding list
        std::optional<Pose3D<double>> init_robot_pose;
        Pose3D<double> init_pose_est;
        std::optional<RawPose3d<double>> curr_pose_est_raw =
            pose_graph->getRobotPose(max_frame_id);
        if (input_problem_data.getRobotPoseEstimateForFrame(max_frame_id,
                                                            init_pose_est)) {
          init_robot_pose = std::make_optional(init_pose_est);
        }
        if (pending_feature_factors_.find(feature_id) ==
            pending_feature_factors_.end()) {
          pending_feature_factors_[feature_id] = VisualFeatureCachedInfo();
        }
        addFactorsAndRobotPoseToCache(input_problem_data,
                                      pose_graph,
                                      max_frame_id,
                                      reprojection_error_factors,
                                      init_robot_pose,
                                      pending_feature_factors_[feature_id],
                                      enforce_epipolar_error_requirement_);
        // pending_feature_factors_[feature_id].addFactorsAndRobotPose(
        //     max_frame_id, reprojection_error_factors, init_robot_pose);
        // handling pending poses
        const auto &visual_feature_cache =
            pending_feature_factors_.at(feature_id);
        if (checkMinParallaxRequirements_(
                min_frame_id, max_frame_id, visual_feature_cache)) {
          Position3d<double> initial_position;
          getInitialFeaturePosition_(input_problem_data,
                                     pose_graph,
                                     feature_id,
                                     feat_id_to_track.second.feature_pos_,
                                     initial_position);
          pose_graph->addFeature(feature_id, initial_position);
          for (const auto &frame_id_and_factors :
               visual_feature_cache.frame_ids_and_reprojection_err_factors_) {
            for (const auto &factor : frame_id_and_factors.second) {
              pose_graph->addVisualFactor(factor);
            }
          }
          pending_feature_factors_.erase(feature_id);
          added_feature_ids_.insert(feature_id);
        }
      }
    }
    if (gba_checker_(max_frame_id)) {
      std::unordered_set<FeatureId> feat_ids_to_change;
      for (const auto &feat_id_and_cache : pending_feature_factors_) {
        const FeatureId &feature_id = feat_id_and_cache.first;
        const auto &visual_feature_cache = feat_id_and_cache.second;
        if (checkMinParallaxRequirements_(
                min_frame_id, max_frame_id, visual_feature_cache)) {
          Position3d<double> initial_position;
          getInitialFeaturePosition_(
              input_problem_data,
              pose_graph,
              feature_id,
              visual_features.at(feature_id).feature_pos_,
              initial_position);
          pose_graph->addFeature(feature_id, initial_position);
          for (const auto &frame_id_and_factors :
               visual_feature_cache.frame_ids_and_reprojection_err_factors_) {
            for (const auto &factor : frame_id_and_factors.second) {
              pose_graph->addVisualFactor(factor);
            }
          }
          feat_ids_to_change.insert(feature_id);
        }
      }
      for (const auto &feat_id : feat_ids_to_change) {
        pending_feature_factors_.erase(feat_id);
        added_feature_ids_.insert(feat_id);
      }
    }
  }

 protected:
  std::function<bool(const FrameId &)> gba_checker_;
  std::function<double(
      const ProblemDataType &,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &,
      const FrameId &,
      const FeatureId &,
      const CameraId &)>
      reprojection_error_provider_;
  std::unordered_map<FeatureId, FrameId> first_observed_frame_by_feature_;
  std::unordered_set<FeatureId> added_feature_ids_;
  std::unordered_map<FeatureId, VisualFeatureCachedInfo>
      pending_feature_factors_;

  double min_visual_feature_parallax_pixel_requirement_ = 5;
  double min_visual_feature_parallax_robot_transl_requirement_ = 0.1;
  double min_visual_feature_parallax_robot_orient_requirement_ = 0.05;
  bool enforce_min_pixel_parallax_requirement_ = true;
  bool enforce_min_robot_pose_parallax_requirement_ = true;

  double inlier_epipolar_err_thresh_ = 8.0;
  size_t check_pase_n_frames_for_epipolar_err_ = 5;
  bool enforce_epipolar_error_requirement_ = false;

 private:
  bool isReprojectionErrorFacotrInlier_(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &frame_id,
      const ReprojectionErrorFactor &candidate_factor,
      const VisualFeatureCachedInfo &cache_info) {
    double votes = 0.0;
    double n_voters = 0.0;

    CameraIntrinsicsMat<double> candidate_intrinsics;
    if (!pose_graph->getIntrinsicsForCamera(candidate_factor.camera_id_,
                                            candidate_intrinsics)) {
      LOG(WARNING) << "Failed to find camera intrinsics for camera "
                   << candidate_factor.camera_id_;
      return false;
    }
    CameraExtrinsics<double> candidate_extrinsics;
    if (!pose_graph->getExtrinsicsForCamera(candidate_factor.camera_id_,
                                            candidate_extrinsics)) {
      LOG(WARNING) << "Failed to find camera extrinsics for camera "
                   << candidate_factor.camera_id_;
      return false;
    }
    Pose3D<double> candidate_pose;
    if (!input_problem_data.getRobotPoseEstimateForFrame(
            candidate_factor.frame_id_, candidate_pose)) {
      LOG(WARNING) << "Could not find initial pose estimate "
                      "for robot for frame "
                   << candidate_factor.frame_id_;
      return false;
    }
    const PixelCoord<double> &candidate_pixel_obs =
        candidate_factor.feature_pos_;

    for (const auto frame_id_and_factors :
         cache_info.frame_ids_and_reprojection_err_factors_) {
      for (const auto &ref_factor : frame_id_and_factors.second) {
        if (ref_factor == candidate_factor) {
          continue;
        }
        CameraIntrinsicsMat<double> ref_intrinsics;
        if (!pose_graph->getIntrinsicsForCamera(ref_factor.camera_id_,
                                                ref_intrinsics)) {
          LOG(WARNING) << "Failed to find camera intrinsics for camera "
                       << ref_factor.camera_id_;
          return false;
        }
        CameraExtrinsics<double> ref_extrinsics;
        if (!pose_graph->getExtrinsicsForCamera(ref_factor.camera_id_,
                                                ref_extrinsics)) {
          LOG(WARNING) << "Failed to find camera extrinsics for camera "
                       << ref_factor.camera_id_;
          return false;
        }
        Pose3D<double> ref_pose;
        if (!input_problem_data.getRobotPoseEstimateForFrame(
                ref_factor.frame_id_, ref_pose)) {
          LOG(WARNING) << "Could not find initial pose estimate "
                          "for robot for frame "
                       << ref_factor.frame_id_;
          return false;
        }
        const PixelCoord<double> &ref_pixel_obs = ref_factor.feature_pos_;
        Eigen::Vector2d epipolar_err_vec =
            getNormalizedEpipolarErrorVec(ref_intrinsics,
                                          candidate_intrinsics,
                                          ref_extrinsics,
                                          candidate_extrinsics,
                                          ref_pixel_obs,
                                          candidate_pixel_obs,
                                          ref_pose,
                                          candidate_pose);
        if (epipolar_err_vec.norm() < inlier_epipolar_err_thresh_) {
          votes += 1.0;
        }
        n_voters += 1.0;
      }
      // TODO maybe add this to configuration as well
      const double inlier_majority_percentage = 0.5;
      return (votes / n_voters) > inlier_majority_percentage;
    }
  }

  void addFactorsAndRobotPoseToCache(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &frame_id,
      const std::vector<ReprojectionErrorFactor> &reprojection_err_factors,
      const std::optional<Pose3D<double>> &robot_pose,
      VisualFeatureCachedInfo &cache_info,
      const bool use_epipolar_outlier_rejection = false) {
    if (!use_epipolar_outlier_rejection) {
      cache_info.addFactorsAndRobotPose(
          frame_id, reprojection_err_factors, robot_pose);
      return;
    }

    if (cache_info.is_cache_cleaned_) {
      std::vector<ReprojectionErrorFactor> factors_to_add;
      for (const auto &factor : reprojection_err_factors) {
        if (isReprojectionErrorFacotrInlier_(
                input_problem_data, pose_graph, frame_id, factor, cache_info)) {
          factors_to_add.push_back(factor);
        }
      }
      if (!factors_to_add.empty()) {
        cache_info.addFactorsAndRobotPose(frame_id, factors_to_add, robot_pose);
      }
    } else {
      cache_info.addFactorsAndRobotPose(
          frame_id, reprojection_err_factors, robot_pose);
      std::map<FrameId, std::vector<ReprojectionErrorFactor>>
          cleaned_frame_ids_and_factors;
      for (const auto &frame_id_and_factors :
           cache_info.frame_ids_and_reprojection_err_factors_) {
        const FrameId frame_id = frame_id_and_factors.first;
        for (const auto &factor : frame_id_and_factors.second) {
          if (isReprojectionErrorFacotrInlier_(input_problem_data,
                                               pose_graph,
                                               frame_id,
                                               factor,
                                               cache_info)) {
            cleaned_frame_ids_and_factors[frame_id].push_back(factor);
          }
        }
      }
      if (!cleaned_frame_ids_and_factors.empty()) {
        cache_info.frame_ids_and_reprojection_err_factors_ =
            cleaned_frame_ids_and_factors;
        cache_info.is_cache_cleaned_ = true;
      }
    }
  }

  bool getInitialFeaturePosition_(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FeatureId &feature_id,
      const Position3d<double> &unadjusted_feature_pos,
      Position3d<double> &adjusted_initial_position) {
    FrameId first_frame_id =
        pending_feature_factors_.at(feature_id).getMinFrameId();
    Pose3D<double> init_first_pose;
    std::optional<RawPose3d<double>> optim_first_raw_pose =
        pose_graph->getRobotPose(first_frame_id);
    if ((!input_problem_data.getRobotPoseEstimateForFrame(first_frame_id,
                                                          init_first_pose)) ||
        (!optim_first_raw_pose.has_value())) {
      adjusted_initial_position = unadjusted_feature_pos;
      return false;
    } else {
      Position3d<double> relative_initial_position =
          getPositionRelativeToPose(init_first_pose, unadjusted_feature_pos);
      Pose3D<double> optim_first_pose =
          convertToPose3D(optim_first_raw_pose.value());
      adjusted_initial_position =
          combinePoseAndPosition(optim_first_pose, relative_initial_position);
      return true;
    }
  }

  bool checkMinParallaxRequirements_(
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const VisualFeatureCachedInfo &visualFeatureCachedInfo) const {
    std::vector<FrameId> frame_ids =
        visualFeatureCachedInfo.getOrderedFrameIdsGreaterThan(min_frame_id);
    if (frame_ids.size() <= 1) {
      return false;
    }

    for (size_t i = 0; i < frame_ids.size() - 1; ++i) {
      FrameId frame_id1 = frame_ids[i];
      std::optional<Pose3D<double>> robot_pose1 =
          visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id1);
      std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels1 =
          visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id1);
      for (size_t j = i + 1; j < frame_ids.size(); ++j) {
        FrameId frame_id2 = frame_ids[j];
        std::optional<Pose3D<double>> robot_pose2 =
            visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id2);
        std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels2 =
            visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id2);

        bool pixel_req_satisfied, pose_req_satisfied;
        pixel_req_satisfied = pose_req_satisfied = false;
        if (enforce_min_robot_pose_parallax_requirement_) {
          if (robot_pose1.has_value() && robot_pose2.has_value()) {
            Pose3D<double> relative_pose = getPose2RelativeToPose1(
                robot_pose1.value(), robot_pose2.value());
            // TODO consider the camera extrinsics when checking the pose
            // requirement
            if ((relative_pose.transl_.norm() >=
                 min_visual_feature_parallax_robot_transl_requirement_) ||
                (relative_pose.orientation_.angle() >=
                 min_visual_feature_parallax_robot_orient_requirement_)) {
              pose_req_satisfied = true;
            }
          }
        }
        if (enforce_min_pixel_parallax_requirement_) {
          for (const auto &cam_id_and_pixel1 : cam_ids_and_pixels1) {
            const PixelCoord<double> &pixel1 = cam_id_and_pixel1.second;
            for (const auto &cam_id_and_pixel2 : cam_ids_and_pixels2) {
              const PixelCoord<double> &pixel2 = cam_id_and_pixel2.second;
              double pixel_displacement = (pixel1 - pixel2).norm();
              if (pixel_displacement >=
                  min_visual_feature_parallax_pixel_requirement_) {
                pixel_req_satisfied = true;
              }
            }
          }
        }
        bool req_satisfied = false;
        if (enforce_min_robot_pose_parallax_requirement_ &&
            !enforce_min_pixel_parallax_requirement_) {
          req_satisfied = pose_req_satisfied;
        } else if (!enforce_min_robot_pose_parallax_requirement_ &&
                   enforce_min_pixel_parallax_requirement_) {
          req_satisfied = pixel_req_satisfied;
        } else if (enforce_min_robot_pose_parallax_requirement_ &&
                   enforce_min_pixel_parallax_requirement_) {
          req_satisfied = (pose_req_satisfied && pixel_req_satisfied);
        } else {  // !enforce_min_robot_pose_parallax_requirement_ &&
                  // !enforce_min_robot_pose_parallax_requirement_
          req_satisfied = true;
        }
        if (req_satisfied) {
          return true;
        }
      }
    }
    return false;
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_VISUAL_FEATURE_FRONT_END_H