//
// Created by amanda on 6/18/22.
//

#ifndef UT_VSLAM_OUTPUT_PROBLEM_DATA_H
#define UT_VSLAM_OUTPUT_PROBLEM_DATA_H

#include <refactoring/types/vslam_obj_opt_types_refactor.h>

namespace vslam_types_refactor {
struct EllipsoidResults {
  std::unordered_map<ObjectId, EllipsoidState<double>> ellipsoids_;
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
}
#endif  // UT_VSLAM_OUTPUT_PROBLEM_DATA_H
