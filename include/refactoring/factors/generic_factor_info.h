//
// Created by amanda on 1/18/23.
//

#ifndef UT_VSLAM_GENERIC_FACTOR_INFO_H
#define UT_VSLAM_GENERIC_FACTOR_INFO_H

#include <refactoring/optimization/object_pose_graph.h>

namespace vslam_types_refactor {

struct ParameterBlockInfo {
  std::optional<FrameId> frame_id_;
  std::optional<ObjectId> obj_id_;
  std::optional<FeatureId> feature_id_;
};

struct GenericFactorInfo {
  // NOTE: This only supports factors that operate on one object/feature at a
  // time. If we move to more complex long-term maps or pairwise feature
  // factors, we'll need to rework this.
  FactorType factor_type_;
  std::optional<FrameId> frame_id_;
  std::optional<CameraId> camera_id_;
  std::optional<ObjectId> obj_id_;
  std::optional<FeatureId> feature_id_;
  std::optional<double> final_residual_val_;

  GenericFactorInfo() = default;

  GenericFactorInfo(const FactorType &factor_type,
                    const std::optional<FrameId> &frame_id,
                    const std::optional<CameraId> &camera_id,
                    const std::optional<ObjectId> &obj_id,
                    const std::optional<FeatureId> &feature_id,
                    const std::optional<double> &final_residual_val)
      : factor_type_(factor_type),
        frame_id_(frame_id),
        camera_id_(camera_id),
        obj_id_(obj_id),
        feature_id_(feature_id),
        final_residual_val_(final_residual_val){}
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_GENERIC_FACTOR_INFO_H
