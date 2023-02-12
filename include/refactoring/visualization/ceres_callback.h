//
// Created by taijing on 2/11/22
//

#ifndef UT_VSLAM_CERES_CALLBACK_H
#define UT_VSLAM_CERES_CALLBACK_H

#include <ceres/iteration_callback.h>
#include <refactoring/visualization/ros_visualization.h>
#include <unistd.h>
#include <visualization/matplotlibcpp.h>

#include <filesystem>
#include <map>

namespace fs = std::filesystem;
namespace plt = matplotlibcpp;
namespace vslam_types_refactor {

// TODO Fix hardcoding
class CeresCallbackLogger {
 public:
  CeresCallbackLogger() {}

  CeresCallbackLogger(std::string out_directory)
      : out_directory_(out_directory) {
    if (!fs::is_directory(out_directory_) || !fs::exists(out_directory_)) {
      if (!fs::create_directory(out_directory_)) {
        LOG(ERROR) << "failed to create directory " << out_directory_;
      }
    }
    out_traj_directory_ = out_directory_ / "trajectory";
    fs::create_directory(out_traj_directory_);
  }

  void log(const size_t &frame_id,
           const size_t niter,
           const std::map<FrameId, Pose3D<double>> &frame_ids_and_poses,
           const std::vector<std::pair<FrameId, FrameId>>
               &frame_id1s_and_frame_id2s) {
    fs::path out_frame_traj_directory_ =
        out_traj_directory_ / std::to_string(frame_id);
    if (!fs::exists(out_frame_traj_directory_)) {
      fs::create_directory(out_frame_traj_directory_);
    }
    fs::path out_frame_traj_path =
        out_frame_traj_directory_ / (std::to_string(niter) + ".csv");
    toCSV(out_frame_traj_path.string(), frame_ids_and_poses);
    fs::path out_frame_traj_viz_path =
        out_frame_traj_directory_ / (std::to_string(niter) + ".png");
    plotTrajectory2D(out_frame_traj_viz_path.string(),
                     frame_ids_and_poses,
                     frame_id1s_and_frame_id2s);
  }

 private:
  fs::path out_directory_;
  fs::path out_traj_directory_;

  void toCSV(const std::string &filename,
             const std::vector<Pose3D<double>> &trajectory,
             const std::string &delimiter = ",") {
    std::ofstream ofile;
    ofile.open(filename, std::ios::trunc);
    if (!ofile.is_open()) {
      LOG(ERROR) << "failed to open " << filename;
    }
    ofile << "x" << delimiter << "y" << delimiter << "z" << delimiter << "qx"
          << delimiter << "qy" << delimiter << "qz" << delimiter << "qw"
          << std::endl;
    for (const auto &pose : trajectory) {
      ofile << pose.transl_.x() << delimiter << pose.transl_.y() << delimiter
            << pose.transl_.z() << delimiter;
      Eigen::Quaterniond quat(pose.orientation_);
      ofile << quat.x() << delimiter << quat.y() << delimiter << quat.z()
            << delimiter << quat.w() << std::endl;
    }
    ofile.close();
  }

  void toCSV(const std::string &filename,
             const std::map<FrameId, Pose3D<double>> &frame_ids_and_poses,
             const std::string &delimiter = ",") {
    std::vector<Pose3D<double>> poses;
    for (const auto &frame_id_and_pose : frame_ids_and_poses) {
      poses.push_back(frame_id_and_pose.second);
    }
  }

  void plotTrajectory2D(
      const std::string &filepath,
      const std::map<FrameId, Pose3D<double>> &frame_ids_and_poses,
      const std::vector<std::pair<FrameId, FrameId>>
          &frame_id1s_and_frame_id2s) {
    const FrameId nframe_window = 20;
    plt::figure();
    plt::figure_size(640, 360);
    std::vector<std::pair<FrameId, FrameId>> start_frame_ids_and_end_frame_ids;
    for (const auto &frame_id1_and_frame_id2 : frame_id1s_and_frame_id2s) {
      const FrameId &frame_id1 = frame_id1_and_frame_id2.first;
      const FrameId &frame_id2 = frame_id1_and_frame_id2.second;
      const FrameId &min_frame_id = frame_ids_and_poses.begin()->first;
      const FrameId &max_frame_id = frame_ids_and_poses.rbegin()->first;
      FrameId start_frame_id, end_frame_id;
      start_frame_id = min_frame_id + nframe_window > frame_id1
                           ? min_frame_id
                           : frame_id1 - nframe_window;
      end_frame_id = frame_id2 + nframe_window > max_frame_id
                         ? max_frame_id
                         : frame_id2 + nframe_window;
      start_frame_ids_and_end_frame_ids.emplace_back(frame_id1, frame_id2);
    }
    std::sort(start_frame_ids_and_end_frame_ids.begin(),
              start_frame_ids_and_end_frame_ids.end());
    if (start_frame_ids_and_end_frame_ids.size() <= 1) {
      return;
    }
    std::vector<std::pair<FrameId, FrameId>>
        merged_start_frame_ids_and_end_frame_ids;
    std::pair<FrameId, FrameId> merged_start_frame_id_and_end_frame_id =
        start_frame_ids_and_end_frame_ids[0];
    for (size_t i = 1; i < start_frame_ids_and_end_frame_ids.size(); ++i) {
      // if there's overlapping between two consecutive frame id pairs
      if (merged_start_frame_id_and_end_frame_id.second >=
          start_frame_ids_and_end_frame_ids[i].first) {
        FrameId end_frame_id =
            merged_start_frame_id_and_end_frame_id.second >
                    start_frame_ids_and_end_frame_ids[i].second
                ? merged_start_frame_id_and_end_frame_id.second
                : start_frame_ids_and_end_frame_ids[i].second;
        merged_start_frame_id_and_end_frame_id = std::make_pair(
            merged_start_frame_id_and_end_frame_id.first, end_frame_id);
      } else {
        merged_start_frame_ids_and_end_frame_ids.push_back(
            merged_start_frame_id_and_end_frame_id);
        merged_start_frame_id_and_end_frame_id =
            start_frame_ids_and_end_frame_ids[i];
      }
    }
    merged_start_frame_ids_and_end_frame_ids.push_back(
        merged_start_frame_id_and_end_frame_id);
    for (const auto &start_frame_id_and_end_frame_id :
         merged_start_frame_ids_and_end_frame_ids) {
      const FrameId &start_frame_id = start_frame_id_and_end_frame_id.first;
      const FrameId &end_frame_id = start_frame_id_and_end_frame_id.second;
      std::vector<double> xs, ys;
      for (FrameId frame_id = start_frame_id; frame_id <= end_frame_id;
           ++frame_id) {
        xs.push_back(frame_ids_and_poses.at(frame_id).transl_.x());
        ys.push_back(frame_ids_and_poses.at(frame_id).transl_.y());
      }
      plt::plot(xs, ys);
    }
    for (const auto &frame_id1_and_frame_id2 : frame_id1s_and_frame_id2s) {
      const FrameId &frame_id1 = frame_id1_and_frame_id2.first;
      const FrameId &frame_id2 = frame_id1_and_frame_id2.second;
      plt::scatter(std::vector({frame_ids_and_poses.at(frame_id1).transl_.x(),
                                frame_ids_and_poses.at(frame_id2).transl_.x()}),
                   std::vector({frame_ids_and_poses.at(frame_id1).transl_.y(),
                                frame_ids_and_poses.at(frame_id2).transl_.y()}),
                   30.0);
    }
    std::string title = "Trajectory around ";
    for (const auto &frame_id1_and_frame_id2 : frame_id1s_and_frame_id2s) {
      title += ("Frame " + std::to_string(frame_id1_and_frame_id2.first) +
                " and Frame" + std::to_string(frame_id1_and_frame_id2.second) +
                ", ");
    }
    plt::title(title);
    plt::save(filepath);
  }

  void plotTrajectory2D(
      const std::string &filepath,
      const std::map<FrameId, Pose3D<double>> &frame_ids_and_poses,
      const FrameId &frame_id,
      const size_t &niter) {
    std::vector<double> xs, ys;
    for (const auto &frame_id_and_pose : frame_ids_and_poses) {
      xs.push_back(frame_id_and_pose.second.transl_.x());
      ys.push_back(frame_id_and_pose.second.transl_.y());
    }
    plt::figure();
    // TODO right now only assume all images have the same size
    plt::figure_size(640, 360);
    plt::plot(xs, ys);
    plt::legend();
    plt::title("Trajectory at Frame" + std::to_string(frame_id) +
               "and Iteration " + std::to_string(niter));
    plt::save(filepath);
  }
};

// TODO add an global BA checker
template <typename ProblemDataType>
class CeresCallback : public ceres::IterationCallback {
 public:
  explicit CeresCallback(
      const ProblemDataType &input_problem_data,
      const std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> &pose_graph,
      const FrameId &min_frame_id,
      const FrameId &max_frame_id,
      const std::shared_ptr<RosVisualization> &vis_manager)
      : input_problem_data_(input_problem_data),
        pose_graph_(pose_graph),
        min_frame_id_(min_frame_id),
        max_frame_id_(max_frame_id),
        vis_manager_(vis_manager),
        logger_(CeresCallbackLogger(
            "/robodata/taijing/object-slam/vslam/debug/ceres/")) {}

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
    std::vector<Pose3D<double>> trajectory;
    for (const auto &frame_id_and_pose : frame_ids_and_poses) {
      trajectory.push_back(frame_id_and_pose.second);
    }
    vis_manager_->visualizeTrajectory(trajectory, PlotType::ESTIMATED);
    std::vector<std::pair<FrameId, FrameId>> frame_id1s_and_frame_id2s;
    if (!isConsecutivePosesStable(frame_ids_and_poses,
                                  frame_id1s_and_frame_id2s)) {
      LOG(WARNING)
          << "Detect jumps in trajectory during optimization between frames";
      logger_.log(max_frame_id_,
                  summary.iteration,
                  frame_ids_and_poses,
                  frame_id1s_and_frame_id2s);
    }
    return ceres::SOLVER_CONTINUE;
  }

 private:
  ProblemDataType input_problem_data_;
  std::shared_ptr<ObjectAndReprojectionFeaturePoseGraph> pose_graph_;
  FrameId min_frame_id_;
  FrameId max_frame_id_;
  std::shared_ptr<RosVisualization> vis_manager_;
  CeresCallbackLogger logger_;

  const double kConsecutiveTranslTol = 0.75;
  const double kConsecutiveOrientTol = 3.14;

  bool isConsecutivePosesStable(
      const std::map<FrameId, Pose3D<double>> frame_ids_and_poses,
      std::vector<std::pair<FrameId, FrameId>> &frame_id1s_and_frame_id2s) {
    if (frame_ids_and_poses.size() <= 1) {
      return false;
    }
    FrameId min_frame_id = frame_ids_and_poses.begin()->first;
    FrameId max_frame_id = frame_ids_and_poses.rbegin()->first;
    for (FrameId frame_id = min_frame_id + 1; frame_id <= max_frame_id;
         ++frame_id) {
      Pose3D<double> relative_pose =
          getPose2RelativeToPose1(frame_ids_and_poses.at(frame_id),
                                  frame_ids_and_poses.at(frame_id - 1));
      if (relative_pose.transl_.norm() > kConsecutiveTranslTol ||
          std::fabs(relative_pose.orientation_.angle()) >
              kConsecutiveOrientTol) {
        frame_id1s_and_frame_id2s.emplace_back(frame_id - 1, frame_id);
      }
    }
    if (frame_id1s_and_frame_id2s.empty()) {
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CERES_CALLBACK_H