#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>

#include "slam_backend_solver.h"
#include "slam_solver_optimizer_params.h"
#include "visual_slam_ceres_visualization_callback.h"
#include "vslam_io.h"
#include "vslam_types.h"
#include "vslam_util.h"

DEFINE_string(dataset_path,
              "",
              "\nPath to folder containing the dataset. Structured as - \n"
              "vslam_setX/\n\tcalibration/camera_matrix.txt\n\tfeatures/"
              "features.txt\n\t0000x.txt\n\n");

DEFINE_string(output_path,
              "",
              "\nPath to folder where we want to write output trajectories to");

DEFINE_bool(
    save_poses,
    false,
    "\nIf true poses will be saved in Kitti format to the output_path file");

using std::cout;
using std::endl;

int main(int argc, char **argv) {
  cout << "start main" << endl;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty structured slam problem
  vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> prob;

  // Load structured slam problem and intrinsic calibration K
  cout << "start loading problem" << endl;
  vslam_io::LoadStructuredUTSLAMProblem(FLAGS_dataset_path, prob);
  cout << "finish loading problem" << endl;

  // Solve
  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::SLAMSolver solver(optimizer_params);

  // These are the poses that are going to be optimized
  std::vector<vslam_types::RobotPose> answer(prob.robot_poses);
  vslam_types::RobotPose init_pose_unadjusted = answer[0];
  std::vector<vslam_types::RobotPose> gt_robot_poses;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, gt_robot_poses);

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "gt.txt", gt_robot_poses);
  }

  std::vector<vslam_types::RobotPose> adjusted_to_zero_answer;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, adjusted_to_zero_answer);

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "start.txt",
                               adjusted_to_zero_answer);
  }

  Eigen::Vector3d feature_sigma_linear(0.0, 0.0, 0.0);
  for (size_t i = 0; i < prob.tracks.size(); i++) {
    prob.tracks[i].point = vslam_util::getPositionRelativeToPose(init_pose_unadjusted, prob.tracks[i].point);
  }
  if (1) {
    std::ofstream of_points;
    of_points.open(FLAGS_output_path + "adjusted_features.txt");
    for (const auto& it : prob.tracks) {
      of_points << it.first << " " << it.second.point[0] 
                            << " " << it.second.point[1] 
                            << " " << it.second.point[2] << endl;
    }
    of_points.close();
  }

  std::function<void(
      const vslam_solver::StructuredSlamProblemParams &,
      vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> &,
      ceres::Problem &,
      std::vector<vslam_types::SLAMNode> *)>
      structured_vision_constraint_adder =
          vslam_solver::AddStructuredVisionFactorsOffline;

  std::function<std::vector<vslam_types::VisionFeature>(
      const vslam_types::StructuredVisionFeatureTrack &)>
      feature_retriever =
          [](const vslam_types::StructuredVisionFeatureTrack &feature_track) {
            return feature_track.feature_track.track;
          };

  // Got the casting solution from
  // https://stackoverflow.com/questions/30393285/stdfunction-fails-to-distinguish-overloaded-functions
  typedef std::shared_ptr<vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>> (*funtype)(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *);
  funtype func = vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>::create;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *)>
      unbound_callback_creator = func;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      std::vector<vslam_types::SLAMNode> *)>
      callback_creator = std::bind(unbound_callback_creator,
                                   std::placeholders::_1,
                                   feature_retriever,
                                   gt_robot_poses,
                                   std::placeholders::_2);

  vslam_solver::StructuredSlamProblemParams problem_params;

/*
  for (size_t start_frame_idx = 0; 
       start_frame_idx + problem_params.n_interval_frames < prob.robot_poses.size(); 
       ++start_frame_idx) { // TODO FIXME
    prob.start_frame_id = start_frame_idx;
    solver.SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                    vslam_solver::StructuredSlamProblemParams>(
        structured_vision_constraint_adder,
        callback_creator,
        problem_params,
        prob,
        adjusted_to_zero_answer);
  }
*/
  const float MIN_TRANSLATION_OPT = 10;
  const float MIN_ROTATION_OPT = M_1_PI / 6; // 30 degree
  std::vector<vslam_types::RobotPose> adjusted_to_zero_answer;


  prob.output = FLAGS_output_path;
  int start_frame_idx = 0; 
  int end_frame_idx = start_frame_idx;
  while (end_frame_idx < prob.robot_poses.size()) {
    ++end_frame_idx;
    Eigen::Quaternionf q_start(adjusted_to_zero_answer[start_frame_idx].angle);
    Eigen::Quaternionf q_end(adjusted_to_zero_answer[end_frame_idx].angle);
    if ((adjusted_to_zero_answer[end_frame_idx].loc - adjusted_to_zero_answer[start_frame_idx].loc).norm() < MIN_TRANSLATION_OPT 
        && abs(q_start.angularDistance(q_end)) < MIN_ROTATION_OPT ) { continue; }
    prob.start_frame_id = start_frame_idx;
    prob.end_frame_id   = end_frame_idx;
    solver.SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                    vslam_solver::StructuredSlamProblemParams>(
        structured_vision_constraint_adder,
        callback_creator,
        problem_params,
        prob,
        adjusted_to_zero_answer);
    if (1) { // do it here bc of type incompatibility
      std::ofstream of_points;
      of_points.open(FLAGS_output_path + "features/" + std::to_string(prob.start_frame_id) + ".txt", std::ios::trunc);
      for (const auto& it : prob.tracks) {
        of_points << it.first << " " << it.second.point[0] 
                              << " " << it.second.point[1] 
                              << " " << it.second.point[2] << endl;
      }
      of_points.close();
    }
    ++start_frame_idx; // don't need to update end_frame_idx
    --end_frame_idx; // avoid huge jumps at frame end_frame_idx
  }

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "answer_batch_increment.txt",
                               adjusted_to_zero_answer);
  }

  return 0;
}