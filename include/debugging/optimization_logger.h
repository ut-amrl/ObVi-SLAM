//
// Created by amanda on 2/28/23.
//

#ifndef UT_VSLAM_OPTIMIZATION_LOGGER_H
#define UT_VSLAM_OPTIMIZATION_LOGGER_H

#include <ceres/solver.h>
#include <file_io/file_access_utils.h>
#include <file_io/file_io_utils.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>

#include <string>
#include <unordered_set>

namespace vslam_types_refactor {

struct IterationSummaryInfo {
  int num_parameters_reduced_;
  std::vector<ceres::IterationSummary> iteration_summaries_;

  IterationSummaryInfo(
      const int &num_parameters_reduced,
      const std::vector<ceres::IterationSummary> &iteration_summaries)
      : num_parameters_reduced_(num_parameters_reduced),
        iteration_summaries_(iteration_summaries) {}
};

class IterationLogger {
 public:
  IterationLogger(const std::string &logging_directory,
                  const std::string &logging_type)
      : logging_directory_(logging_directory),
        logging_type_(logging_type),
        output_file_path_(
            file_io::ensureDirectoryPathEndsWithSlash(logging_directory) +
            "ceres_iterations_" + logging_type + file_io::kCsvExtension) {
    std::ofstream csv_file(output_file_path_, std::ios::trunc);
    file_io::writeCommaSeparatedStringsLineToFile({"optimization_id",
                                                   "iteration_num",
                                                   "cost",
                                                   "cost_change",
                                                   "step_norm",
                                                   "step_norm_per_param",
                                                   "is_successful"},
                                                  csv_file);
    csv_file.close();
  }

  void logIterations(const std::string &optimization_identifier,
                     const ceres::Solver::Summary &solver_summary) {
    iteration_info_.emplace_back(std::make_pair(
        optimization_identifier,
        IterationSummaryInfo(solver_summary.num_parameters_reduced,
                             solver_summary.iterations)));
  }

  void writeLatestData() {
    if (!iteration_info_.empty()) {
      std::ofstream csv_file(output_file_path_, std::ios::app);
      for (const std::pair<std::string, IterationSummaryInfo>
               &optimization_info : iteration_info_) {
        for (const ceres::IterationSummary &iteration_summary :
             optimization_info.second.iteration_summaries_) {
          file_io::writeCommaSeparatedStringsLineToFile(
              {optimization_info.first,
               std::to_string(iteration_summary.iteration),
               std::to_string(iteration_summary.cost),
               std::to_string(iteration_summary.cost_change),
               std::to_string(iteration_summary.step_norm),
               std::to_string(
                   (iteration_summary.step_norm /
                    optimization_info.second.num_parameters_reduced_)),
               std::to_string(iteration_summary.step_is_successful ? 1 : 0)},
              csv_file);
        }
      }
      csv_file.close();
      iteration_info_.clear();
    }
  }

 private:
  std::string logging_directory_;
  std::string logging_type_;
  std::string output_file_path_;

  std::vector<std::pair<std::string, IterationSummaryInfo>> iteration_info_;
};

class IterationLoggerFactory {
 protected:
  IterationLoggerFactory() : initialized_(false) {}

 public:
  inline const static std::string kPendingEstimatorOptimizationType =
      "pending_obj_est";
  inline const static std::string kVfAdjustOptimizationType = "vf_adjust";
  inline const static std::string kPrePgoTrackOptimizationType =
      "pre_pgo_track";
  inline const static std::string kPGOOptimizationType = "pgo";
  inline const static std::string kLBAPhase1OptimizationType = "lba_phase_1";
  inline const static std::string kLBAPhase2OptimizationType = "lba_phase_2";
  inline const static std::string kGBAPhase1OptimizationType = "gba_phase_1";
  inline const static std::string kGBAPhase2OptimizationType = "gba_phase_2";

  static IterationLoggerFactory &getInstance() {
    static IterationLoggerFactory factory_instance;
    return factory_instance;
  }

  static void setLoggingDirectory(const std::string &logging_directory) {
    IterationLoggerFactory &logger_factory =
        IterationLoggerFactory::getInstance();
    logger_factory.logging_directory_ = logging_directory;
    logger_factory.initialized_ = true;
  }

  std::shared_ptr<IterationLogger> getOrCreateLoggerOfType(
      const std::string &logger_type) {
    if (!initialized_) {
      LOG(ERROR) << "Not initialized with target directory, returning null ptr";
      return nullptr;
    }
    if (iteration_loggers_by_type_.find(logger_type) ==
        iteration_loggers_by_type_.end()) {
      iteration_loggers_by_type_[logger_type] =
          std::make_shared<IterationLogger>(logging_directory_, logger_type);
    }
    return iteration_loggers_by_type_.at(logger_type);
  }

  void writeAllIterationLoggerStates() {
    for (const auto &iteration_logger : iteration_loggers_by_type_) {
      if (iteration_logger.second != nullptr) {
        iteration_logger.second->writeLatestData();
      }
    }
  }

 private:
  bool initialized_;
  std::string logging_directory_;
  std::unordered_map<std::string, std::shared_ptr<IterationLogger>>
      iteration_loggers_by_type_;
};

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
  size_t attempt_num_;
};

class OptimizationLogger {
 public:
  OptimizationLogger(const std::string &output_file_path)
      : output_file_path_(output_file_path) {}

  void setOptimizationTypeParams(const FrameId &max_frame_id,
                                 const bool &global_ba,
                                 const bool &global_pgo,
                                 const bool &outliers_excluded,
                                 const size_t &attempt_num = 0) {
    current_opt_info_.max_frame_id_ = max_frame_id;
    current_opt_info_.global_ba_ = global_ba;
    current_opt_info_.global_pgo_ = global_pgo;
    current_opt_info_.local_ba_ = !global_ba;
    current_opt_info_.outliers_excluded_opt_ = outliers_excluded;
    current_opt_info_.attempt_num_ = attempt_num;
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

    std::string opt_type;
    if (current_opt_info_.global_pgo_) {
      opt_type = IterationLoggerFactory::kPGOOptimizationType;
    } else if (current_opt_info_.global_ba_) {
      if (current_opt_info_.outliers_excluded_opt_) {
        opt_type = IterationLoggerFactory::kGBAPhase2OptimizationType;
      } else {
        opt_type = IterationLoggerFactory::kGBAPhase1OptimizationType;
      }
    } else if (current_opt_info_.local_ba_) {
      if (current_opt_info_.outliers_excluded_opt_) {
        opt_type = IterationLoggerFactory::kLBAPhase2OptimizationType;
      } else {
        opt_type = IterationLoggerFactory::kLBAPhase1OptimizationType;
      }
    } else {
      LOG(WARNING) << "Couldn't find optimization type for current "
                      "configuration; not outputting iteration data";
      return;
    }
    std::string opt_identifier =
        std::to_string(current_opt_info_.max_frame_id_) + "_" +
        std::to_string(current_opt_info_.attempt_num_);

    std::shared_ptr<IterationLogger> iteration_logger =
        IterationLoggerFactory::getInstance().getOrCreateLoggerOfType(opt_type);
    if (iteration_logger != nullptr) {
      iteration_logger->logIterations(opt_identifier, ceres_solver_summary);
    }
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
    //    bool found_opt_type = false;
    //    std::string opt_type;
    //    if (current_opt_info_.global_pgo_) {
    //      found_opt_type = true;
    //      opt_type = IterationLoggerFactory::kPGOOptimizationType;
    //    } else if (current_opt_info_.global_ba_) {
    //      if (current_opt_info_.outliers_excluded_opt_) {
    //        found_opt_type = true;
    //        opt_type = IterationLoggerFactory::kGBAPhase2OptimizationType;
    //      } else {
    //        found_opt_type = true;
    //        opt_type = IterationLoggerFactory::kGBAPhase1OptimizationType;
    //      }
    //    } else if (current_opt_info_.local_ba_) {
    //      if (current_opt_info_.outliers_excluded_opt_) {
    //        found_opt_type = true;
    //        opt_type = IterationLoggerFactory::kLBAPhase2OptimizationType;
    //      } else {
    //        found_opt_type = true;
    //        opt_type = IterationLoggerFactory::kLBAPhase1OptimizationType;
    //      }
    //    } else {
    //      LOG(WARNING) << "Couldn't find optimization type for current "
    //                      "configuration; not outputting iteration data";
    //    }
    //    if (found_opt_type) {
    //      IterationLoggerFactory::getInstance()
    //          .getOrCreateLoggerOfType(opt_type)
    //          ->writeLatestData();
    //    }
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
