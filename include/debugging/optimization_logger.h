//
// Created by amanda on 2/28/23.
//

#ifndef UT_VSLAM_OPTIMIZATION_LOGGER_H
#define UT_VSLAM_OPTIMIZATION_LOGGER_H

#include <ceres/solver.h>
#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <string>
#include <unordered_set>

namespace vslam_types_refactor {

struct CurrentOptimizationInfo {
  FrameId max_frame_id_;
  bool outliers_excluded_opt_;
  bool local_ba_;
  bool global_ba_;
  bool global_pgo_;
  size_t num_poses_;
  size_t num_objects_;
  size_t num_visual_features_;
  double total_ceres_time_;
  double linear_solver_time_;
  double jacobian_time_;
  double residual_time_;
  size_t num_ceres_iterations_;
};

class OptimizationLogger {
 public:
  OptimizationLogger(const std::string &output_file_path)
      : output_file_path_(output_file_path) {}

  void setOptimizationTypeParams(const FrameId &max_frame_id,
                                 const bool &global_ba,
                                 const bool &global_pgo,
                                 const bool &outliers_excluded) {
    current_opt_info_.max_frame_id_ = max_frame_id;
    current_opt_info_.global_ba_ = global_ba;
    current_opt_info_.global_pgo_ = global_pgo;
    current_opt_info_.local_ba_ = !global_ba;
    current_opt_info_.outliers_excluded_opt_ = outliers_excluded;
  }

  void setOptimizationParams(
      const std::unordered_set<ObjectId> &optimized_objects,
      const std::unordered_set<FeatureId> &optimized_features,
      const std::unordered_set<FrameId> &optimized_frames) {
    current_opt_info_.num_visual_features_ = optimized_features.size();
    current_opt_info_.num_objects_ = optimized_objects.size();
    current_opt_info_.num_poses_ = optimized_frames.size();
  }

  void extractOptimizationTimingResults(
      const ceres::Solver::Summary &ceres_solver_summary) {
    current_opt_info_.total_ceres_time_ =
        ceres_solver_summary.total_time_in_seconds;
    current_opt_info_.linear_solver_time_ =
        ceres_solver_summary.linear_solver_time_in_seconds;
    current_opt_info_.jacobian_time_ =
        ceres_solver_summary.jacobian_evaluation_time_in_seconds;
    current_opt_info_.residual_time_ =
        ceres_solver_summary.residual_evaluation_time_in_seconds;
    current_opt_info_.num_ceres_iterations_ =
        ceres_solver_summary.iterations.size();
  }

  void writeCurrentOptInfo() {
    std::ofstream csv_file(output_file_path_, std::ios::app);
    file_io::writeCommaSeparatedStringsLineToFile(
        {std::to_string(current_opt_info_.max_frame_id_),
         std::to_string(current_opt_info_.outliers_excluded_opt_ ? 1 : 0),
         std::to_string(current_opt_info_.local_ba_ ? 1 : 0),
         std::to_string(current_opt_info_.global_ba_ ? 1 : 0),
         std::to_string(current_opt_info_.global_pgo_ ? 1 : 0),
         std::to_string(current_opt_info_.num_poses_),
         std::to_string(current_opt_info_.num_objects_),
         std::to_string(current_opt_info_.num_visual_features_),
         std::to_string(current_opt_info_.total_ceres_time_),
         std::to_string(current_opt_info_.linear_solver_time_),
         std::to_string(current_opt_info_.jacobian_time_),
         std::to_string(current_opt_info_.residual_time_),
         std::to_string(current_opt_info_.num_ceres_iterations_)},
        csv_file);
    current_opt_info_ = CurrentOptimizationInfo();
  }

  void writeOptInfoHeader() {
    if (!output_file_path_.empty()) {
      std::ofstream csv_file(output_file_path_, std::ios::trunc);
      file_io::writeCommaSeparatedStringsLineToFile({"max_frame_id",
                                                     "outliers_excluded?",
                                                     "local_ba?",
                                                     "global_ba?",
                                                     "global_pgo?",
                                                     "num_poses",
                                                     "num_objects",
                                                     "num_visual_features",
                                                     "total_ceres_time",
                                                     "linear_solver_time",
                                                     "jacobian_time",
                                                     "residual_time",
                                                     "num_ceres_iterations"},
                                                    csv_file);
    }
  }

 private:
  std::string output_file_path_;
  CurrentOptimizationInfo current_opt_info_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OPTIMIZATION_LOGGER_H
