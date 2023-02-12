//
// Created by taijing on 2/11/22
//

#ifndef UT_VSLAM_CERES_CALLBACK_H
#define UT_VSLAM_CERES_CALLBACK_H

#include <ceres/iteration_callback.h>

#include <filesystem>
#include <map>

namespace fs = std::filesystem;
namespace vslam_types_refactor {

class CeresCallbackLogger {
 public:
  CeresCallbackLogger(std::string out_directory)
      : out_directory_(out_directory_) {
    if (!fs::is_directory(out_directory_) || !fs::exists(out_directory_)) {
      if (!fs::create_directory(out_directory_)) {
        LOG(ERROR) << "failed to create directory " << out_directory_;
      }
    }
    for (const auto &entry : fs::directory_iterator(out_directory_)) {
      fs::remove_all(entry.path());
    }
    fs::create_directory(out_directory_ / "trajectory")
  }

  void log(const size_t &frame_id, const size_t niter) {
    fs::path frame_output_directory = out_directory_ / "trajectory";
    if (!fs::exists(out_directory_)) {
      fs::create_directory(frame_output_directory);
    }
    frame_output_directory = frame_output_directory
  }

 private:
  fs::path out_directory_;
  void logTrajectory(
      const size_t &frame_id,
      const size_t niter,
      const std::map<FrameId, Pose3D<double>> &frame_ids_and_poses) {}
};

// TODO add an global BA checker
template <typename ProblemDataType>
class CeresCallback : public ceres::IterationCallback {
 public:
  explicit CeresCallback(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id)
      : input_problem_data_(input_problem_data),
        pose_graph_(pose_graph),
        min_frame_id_(min_frame_id),
        max_frame_id_(max_frame_id) {}

  ~CeresCallback() override = default;

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary &summary) override {
    std::map<FrameId, Pose3D<double>> frame_ids_and_poses;
    for (size_t frame_id = min_frame_id_; frame_id <= max_frame_id_;
         ++frame_id) {
      std::optional<RawPose3d<double>> raw_robot_pose =
          pose_graph_->getRobotPose(frame_id);
      if (!raw_robot_pose.has_value()) {
        LOG(WARNING) << "Couldn't find pose for frame " << frame_id
                     << " in ceres iteration call";
        continue;
      }
      frame_ids_and_poses[frame_id] = convertToPose3D(raw_robot_pose.value());
    }
    return ceres::SOLVER_CONTINUE;
  }

 private:
  ProblemDataType input_problem_data_;
  std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CERES_CALLBACK_H