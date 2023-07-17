//
// Created by amanda on 6/18/22.
//

#ifndef UT_VSLAM_OUTPUT_PROBLEM_DATA_H
#define UT_VSLAM_OUTPUT_PROBLEM_DATA_H

#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {
struct EllipsoidResults {
  std::unordered_map<ObjectId, std::pair<std::string, EllipsoidState<double>>>
      ellipsoids_;
};

struct RobotPoseResults {
  std::unordered_map<FrameId, Pose3D<double>> robot_poses_;
};

struct VisualFeatureResults {
  std::unordered_map<FeatureId, Position3d<double>> visual_feature_positions_;
};

struct SpatialEstimateOnlyResults {
  EllipsoidResults ellipsoid_results_;
  RobotPoseResults robot_pose_results_;
  VisualFeatureResults visual_feature_results_;
};

template <typename LongTermObjectMap>
struct LongTermObjectMapAndResults {
  LongTermObjectMap
      long_term_map_;  // The LTM object estimates may differ from the ellipsoid
                       // results because a different set of factors is used to
                       // estimate the long-term map
  RobotPoseResults robot_pose_results_;
  VisualFeatureResults visual_feature_results_;
  EllipsoidResults ellipsoid_results_;
  std::shared_ptr<std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      associated_observed_corner_locations_;
};

struct ObjectDataAssociationResults {
  EllipsoidResults ellipsoid_pose_results_;
  std::unordered_map<
      FrameId,
      std::unordered_map<CameraId,
                         std::unordered_map<ObjectId,
                                            std::pair<BbCornerPair<double>,
                                                      std::optional<double>>>>>
      associated_bounding_boxes_;
};

}  // namespace vslam_types_refactor
#endif  // UT_VSLAM_OUTPUT_PROBLEM_DATA_H
