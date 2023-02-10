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
  bool should_limit_trajectory_evaluation_;

  // Ignored if should_limit_trajectory_evaluation_ is false
  FrameId max_frame_id_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_LIMIT_TRAJECTORY_EVALUATION_PARAMS_H
