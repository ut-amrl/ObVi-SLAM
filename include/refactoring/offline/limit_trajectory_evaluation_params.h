//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_LIMIT_TRAJECTORY_EVALUATION_PARAMS_H
#define UT_VSLAM_LIMIT_TRAJECTORY_EVALUATION_PARAMS_H

#include <refactoring/types/vslam_basic_types_refactor.h>

namespace vslam_types_refactor {
/**
 * Debug only -- used to limit the number of frames in the trajectory to
 * evaluate
 */
struct LimitTrajectoryEvaluationParams {
  bool should_limit_trajectory_evaluation_ = false;

  // Ignored if should_limit_trajectory_evaluation_ is false
  FrameId max_frame_id_;

  bool operator==(const LimitTrajectoryEvaluationParams &rhs) const {
    return (should_limit_trajectory_evaluation_ ==
            rhs.should_limit_trajectory_evaluation_) &&
           (max_frame_id_ == rhs.max_frame_id_);
  }

  bool operator!=(const LimitTrajectoryEvaluationParams &rhs) const {
    return !operator==(rhs);
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LIMIT_TRAJECTORY_EVALUATION_PARAMS_H
