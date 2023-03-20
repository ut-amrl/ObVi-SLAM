//
// Created by taijing on 2/07/22.
//
#ifndef UT_VSLAM_VISUAL_FEATURE_FRONT_END_H
#define UT_VSLAM_VISUAL_FEATURE_FRONT_END_H

#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/optimization/object_pose_graph.h>
#include <refactoring/types/vslam_types_math_util.h>

namespace vslam_types_refactor {

double getEssentialMatrixCorrespondanceResidual(
    const CameraIntrinsicsMat<double> &intrinsics1,
    const CameraIntrinsicsMat<double> &intrinsics2,
    const CameraExtrinsics<double> &extrinsics1,
    const CameraExtrinsics<double> &extrinsics2,
    const PixelCoord<double> &feature_pixel1,
    const PixelCoord<double> &feature_pixel2,
    const Pose3D<double> &pose1,
    const Pose3D<double> &pose2) {
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

  Pose3D<double> pose2_rel_to_pose1 =
      getPose2RelativeToPose1(convertAffineToPose3D(cam_current_to_world_1),
                              convertAffineToPose3D(cam_current_to_world_2));
  // LOG(INFO) << "pose1: " << pose1.transl_.transpose() << "; "
  //           << pose1.orientation_.axis().transpose() *
  //                  pose1.orientation_.angle();
  // LOG(INFO) << "pose2: " << pose2.transl_.transpose() << "; "
  //           << pose2.orientation_.axis().transpose() *
  //                  pose2.orientation_.angle();
  // LOG(INFO) << "pose2_rel_to_pose1: " <<
  // pose2_rel_to_pose1.transl_.transpose()
  //           << "; "
  //           << pose2_rel_to_pose1.orientation_.axis().transpose() *
  //                  pose2_rel_to_pose1.orientation_.angle();

  // source: Multiple View Geometry in computer vision, second edition,
  // chapter 9.6 The essential matrix
  Eigen::Matrix3d E = SkewSymmetric(pose2_rel_to_pose1.transl_) *
                      pose2_rel_to_pose1.orientation_;
  Eigen::Vector3d normalized_x1 =
      intrinsics1.inverse() * pixelToHomogeneous(feature_pixel1);
  Eigen::Vector3d normalized_x2 =
      intrinsics2.inverse() * pixelToHomogeneous(feature_pixel2);
  // LOG(INFO) << "feature_pixel1: " << feature_pixel1.transpose()
  //           << "; feature_pixel2: " << feature_pixel2.transpose();
  // LOG(INFO) << "normalized_x1: " << normalized_x1.transpose()
  //           << "; normalized_x2: " << normalized_x2.transpose();
  // LOG(INFO) << "SkewSymmetric(pose2_rel_to_pose1.transl_): "
  //           << SkewSymmetric(pose2_rel_to_pose1.transl_);
  // LOG(INFO) << "E: " << E;
  // LOG(INFO) << "getEssentialMatrixCorrespondanceResidual: "
  //           << normalized_x2.transpose() * E * normalized_x1 << std::endl;
  return normalized_x2.transpose() * E * normalized_x1;
}

struct VisualFeatureCachedInfo {
  std::map<FrameId, std::vector<ReprojectionErrorFactor>>
      frame_ids_and_reprojection_err_factors_;
  std::map<FrameId, std::optional<Pose3D<double>>> frame_ids_and_poses_;
  // If the information stored in the cache has outlier rejected, etc.
  bool is_cache_cleaned_;

  VisualFeatureCachedInfo() : is_cache_cleaned_(false) {}

  void addFactorsAndRobotPose(
      const FrameId &frame_id,
      const std::vector<ReprojectionErrorFactor> &reprojection_err_factors,
      const std::optional<Pose3D<double>> &robot_pose) {
    frame_ids_and_reprojection_err_factors_[frame_id] =
        reprojection_err_factors;
    frame_ids_and_poses_[frame_id] = robot_pose;
  }

  size_t getFactorNum() const {
    size_t cnt = 0;
    for (const auto &frame_id_and_reprojection_err_factors :
         frame_ids_and_reprojection_err_factors_) {
      cnt += frame_id_and_reprojection_err_factors.second.size();
    }
    return cnt;
  }

  void getAllFactors(std::vector<ReprojectionErrorFactor> &factors) const {
    for (const auto &frame_id_and_reprojection_err_factors :
         frame_ids_and_reprojection_err_factors_) {
      factors.insert(factors.end(),
                     frame_id_and_reprojection_err_factors.second.begin(),
                     frame_id_and_reprojection_err_factors.second.end());
    }
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
      const bool &enforce_min_pixel_parallax_requirement,
      const bool &enforce_min_robot_pose_parallax_requirement)
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
            enforce_min_robot_pose_parallax_requirement) {}

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
      const FrameId &max_frame_id) {  // max_frame_id is the frame_to_add
    std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        visual_features = input_problem_data.getVisualFeatures();

    // iterate over all visual features to find the ones that observed at
    // max_frame_id
    for (const auto &feat_id_to_track : visual_features) {
      FeatureId feature_id = feat_id_to_track.first;
      StructuredVisionFeatureTrack feature_track = feat_id_to_track.second;
      // check if this feature is observed at max_frame_id
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

      // check status of the current feature
      bool is_feature_added_to_pose_graph =
          (added_feature_ids_.find(feature_id) != added_feature_ids_.end());
      bool is_feature_in_cache = (pending_feature_factors_.find(feature_id) !=
                                  pending_feature_factors_.end());
      // feature cannot be added to the pose graph and also be in the cache
      if (is_feature_added_to_pose_graph && is_feature_in_cache) {
        LOG(ERROR) << "Visual frontend's pending cache info is inconsistent "
                      "with pose graph info";
        continue;
      }

      util::BoostHashSet<std::pair<FactorType, FeatureFactorId>>
          matching_factors;
      pose_graph->getFactorsForFeature(feature_id, matching_factors);
      std::unordered_set<FeatureFactorId> matching_factor_ids;
      for (const auto &matching_factor : matching_factors) {
        if (matching_factor.first == kReprojectionErrorFactorTypeId) {
          matching_factor_ids.insert(matching_factor.second);
        }
      }
      // If (matching_factors.size() > 0) xor is_feature_added_to_pose_graph has
      // different signs, something is wrong
      if ((matching_factors.size() > 0) ^ is_feature_added_to_pose_graph) {
        LOG(ERROR) << "Visual frontend's pending cache info is inconsistent "
                      "with pose graph info";
        continue;
      }

      // TODO add it to configuration
      int check_n_visual_feature_correspondance = 3;
      // TODO add this to configuration
      double inlier_essential_matrix_threshold = 1e-3;
      // check if we need to clean outliers from the cache; If so, ifwe have
      // enough information stored in cache to perform outlier rejection
      if (is_feature_in_cache) {
        const VisualFeatureCachedInfo &cache_info =
            pending_feature_factors_.at(feature_id);
        if ((cache_info.getFactorNum() >=
             check_n_visual_feature_correspondance) &&
            (!cache_info.is_cache_cleaned_)) {
          removeOutliersInCache_(input_problem_data,
                                 pose_graph,
                                 inlier_essential_matrix_threshold,
                                 pending_feature_factors_[feature_id]);
        }
      }
      is_feature_in_cache = (pending_feature_factors_.find(feature_id) !=
                             pending_feature_factors_.end());

      std::vector<ReprojectionErrorFactor> ref_factors;
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
        ref_factors.push_back(vis_factor);
      }
      check_n_visual_feature_correspondance = std::max(
          check_n_visual_feature_correspondance, (int)ref_factors.size() - 1);
      check_n_visual_feature_correspondance -= (ref_factors.size() - 1);
      // backtrace the latest frame_ids (to reduce impact of wrongly
      // estimated poses)
      if (is_feature_added_to_pose_graph) {
        std::map<FrameId, std::unordered_set<FeatureFactorId>>
            frame_ids_and_factor_ids;
        for (const auto &factor_id : matching_factor_ids) {
          ReprojectionErrorFactor factor;
          pose_graph->getVisualFactor(factor_id, factor);
          if (frame_ids_and_factor_ids.find(factor.frame_id_) ==
              frame_ids_and_factor_ids.end()) {
            frame_ids_and_factor_ids[factor.frame_id_] =
                std::unordered_set<FeatureFactorId>();
          }
          frame_ids_and_factor_ids[factor.frame_id_].insert(factor_id);
        }
        for (auto it = frame_ids_and_factor_ids.rbegin();
             it != frame_ids_and_factor_ids.rend();
             ++it) {
          for (const auto &factor_id : it->second) {
            if (check_n_visual_feature_correspondance == 0) {
              break;
            }
            ReprojectionErrorFactor factor;
            pose_graph->getVisualFactor(factor_id, factor);
            ref_factors.push_back(factor);
            --check_n_visual_feature_correspondance;
          }
        }
      } else if (is_feature_in_cache) {
        // TODO: have a flag to deal with correspondance matchings in the
        // pending list that are left unchecked
        const auto &visual_feature_cache =
            pending_feature_factors_.at(feature_id);
        if (visual_feature_cache.getFactorNum() > 2) {
          for (auto it = visual_feature_cache
                             .frame_ids_and_reprojection_err_factors_.rbegin();
               it != visual_feature_cache
                         .frame_ids_and_reprojection_err_factors_.rend();
               ++it) {
            for (const auto &factor : it->second) {
              if (check_n_visual_feature_correspondance == 0) {
                break;
              }
              ref_factors.push_back(factor);
              --check_n_visual_feature_correspondance;
            }
          }
        }
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
        if (isReprojectionErrorFacotrInlier_(
                input_problem_data,
                pose_graph,
                vis_factor,
                ref_factors,
                inlier_essential_matrix_threshold)) {
          reprojection_error_factors.push_back(vis_factor);
        }
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
        pending_feature_factors_[feature_id].addFactorsAndRobotPose(
            max_frame_id, reprojection_error_factors, init_robot_pose);
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

 private:
  bool isReprojectionErrorFacotrInlier_(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const ReprojectionErrorFactor &factor,
      const std::vector<ReprojectionErrorFactor> &ref_factors,
      const double inlier_essential_matrix_threshold) {
    double votes = 0.0;
    double n_voters = 0.0;
    CameraIntrinsicsMat<double> intrinsics;
    if (!pose_graph->getIntrinsicsForCamera(factor.camera_id_, intrinsics)) {
      LOG(WARNING) << "Failed to find camera intrinsics for camera "
                   << factor.camera_id_;
      return false;
    }
    CameraExtrinsics<double> extrinsics;
    if (!pose_graph->getExtrinsicsForCamera(factor.camera_id_, extrinsics)) {
      LOG(WARNING) << "Failed to find camera extrinsics for camera "
                   << factor.camera_id_;
      return false;
    }
    Pose3D<double> pose;
    if (!input_problem_data.getRobotPoseEstimateForFrame(factor.frame_id_,
                                                         pose)) {
      LOG(WARNING) << "Could not find initial pose estimate "
                      "for robot for frame "
                   << factor.frame_id_;
      return false;
    }
    const PixelCoord<double> &pixel_obs = factor.feature_pos_;
    for (const auto &ref_factor : ref_factors) {
      if (ref_factor == factor) {
        continue;
      }
      CameraIntrinsicsMat<double> intrinsics_ref;
      if (!pose_graph->getIntrinsicsForCamera(ref_factor.camera_id_,
                                              intrinsics_ref)) {
        LOG(WARNING) << "Failed to find camera intrinsics for camera "
                     << factor.camera_id_;
        return false;
      }
      CameraExtrinsics<double> extrinsics_ref;
      if (!pose_graph->getExtrinsicsForCamera(ref_factor.camera_id_,
                                              extrinsics_ref)) {
        LOG(WARNING) << "Failed to find camera extrinsics for camera "
                     << factor.camera_id_;
        return false;
      }
      Pose3D<double> pose_ref;
      if (!input_problem_data.getRobotPoseEstimateForFrame(ref_factor.frame_id_,
                                                           pose_ref)) {
        LOG(WARNING) << "Could not find initial pose estimate "
                        "for robot for frame "
                     << ref_factor.frame_id_;
        return false;
      }
      const PixelCoord<double> &pixel_obs_ref = ref_factor.feature_pos_;
      if (getEssentialMatrixCorrespondanceResidual(intrinsics_ref,
                                                   intrinsics,
                                                   extrinsics_ref,
                                                   extrinsics,
                                                   pixel_obs_ref,
                                                   pixel_obs,
                                                   pose_ref,
                                                   pose) <
          inlier_essential_matrix_threshold) {
        votes += 1.0;
      }
      n_voters += 1.0;
    }
    // TODO maybe add this to configuration as well
    const double inlier_majority_percentage = 0.5;
    return (votes / n_voters) > inlier_majority_percentage;
  }

  bool removeOutliersInCache_(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const double inlier_essential_matrix_threshold,
      VisualFeatureCachedInfo &cache_info) {
    std::vector<ReprojectionErrorFactor> ref_factors;
    cache_info.getAllFactors(ref_factors);
    for (const auto &frame_id_and_factors :
         cache_info.frame_ids_and_reprojection_err_factors_) {
      const FrameId &frame_id = frame_id_and_factors.first;
      const auto &factors_at_curr_frame = frame_id_and_factors.second;
      std::unordered_set<size_t> factor_indices_to_remove;
      for (size_t i = 0; i < factors_at_curr_frame.size(); ++i) {
        if (!isReprojectionErrorFacotrInlier_(
                input_problem_data,
                pose_graph,
                factors_at_curr_frame[i],
                ref_factors,
                inlier_essential_matrix_threshold)) {
          factor_indices_to_remove.insert(i);
        }
      }
      if (factor_indices_to_remove.size() == factors_at_curr_frame.size()) {
        cache_info.frame_ids_and_reprojection_err_factors_.erase(frame_id);
        cache_info.frame_ids_and_poses_.erase(frame_id);
      }
      std::vector<ReprojectionErrorFactor> new_factors_vec(
          factors_at_curr_frame.size() - factor_indices_to_remove.size());
      for (size_t i = 0; i < factors_at_curr_frame.size(); ++i) {
        if (factor_indices_to_remove.find(i) ==
            factor_indices_to_remove.end()) {
          new_factors_vec.push_back(factors_at_curr_frame.at(i));
        }
      }
      cache_info.frame_ids_and_reprojection_err_factors_[frame_id] =
          new_factors_vec;
    }
    cache_info.is_cache_cleaned_ = true;
    return cache_info.is_cache_cleaned_;
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
      std::optional<Pose3D<double>> robot_pose1;
      if (visualFeatureCachedInfo.frame_ids_and_poses_.find(frame_id1) !=
          visualFeatureCachedInfo.frame_ids_and_poses_.end()) {
        robot_pose1 =
            visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id1);
      }
      std::unordered_map<CameraId, PixelCoord<double>> cam_ids_and_pixels1 =
          visualFeatureCachedInfo.getCamIdsAndPixelsByFrame(frame_id1);
      for (size_t j = i + 1; j < frame_ids.size(); ++j) {
        FrameId frame_id2 = frame_ids[j];
        std::optional<Pose3D<double>> robot_pose2;
        if (visualFeatureCachedInfo.frame_ids_and_poses_.find(frame_id2) !=
            visualFeatureCachedInfo.frame_ids_and_poses_.end()) {
          robot_pose2 =
              visualFeatureCachedInfo.frame_ids_and_poses_.at(frame_id2);
        }
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
        if (enforce_min_robot_pose_parallax_requirement_) {
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
            !enforce_min_robot_pose_parallax_requirement_) {
          req_satisfied = pose_req_satisfied;
        } else if (!enforce_min_robot_pose_parallax_requirement_ &&
                   enforce_min_robot_pose_parallax_requirement_) {
          req_satisfied = pixel_req_satisfied;
        } else if (enforce_min_robot_pose_parallax_requirement_ &&
                   enforce_min_robot_pose_parallax_requirement_) {
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