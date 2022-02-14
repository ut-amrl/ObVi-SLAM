#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <iostream>

#include "slam_backend_solver.h"
#include "slam_solver_optimizer_params.h"
#include "visual_slam_ceres_visualization_callback.h"
#include "vslam_io.h"
#include "vslam_types.h"
#include "vslam_util.h"

using namespace std;
using namespace ros;
using namespace vslam_types;
using namespace vslam_util;

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "ut_vslam_online");
  ros::NodeHandle n;

  // Make empty structured slam problem
  vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> prob;

  // load Intrinsics and Extrinsics into slam_problem_ptr

  // subscribers - load values into slam_problem_ptr
  // Need: RobotPose, vector<VisionFeature> 
  // don't call AdjustTrajectoryToStartAtZero(); call it when saving poses instead

  // estimated 3D Feature positions 
  // TODO mean or the first one?
  // store in slam_problem_ptr

  cout << "start loading problem" << endl;
  vslam_io::LoadStructuredUTSLAMProblem(FLAGS_dataset_path, prob);
  cout << "finish loading problem" << endl;

  // init solver
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

  std::function<void(
      const vslam_solver::StructuredSlamProblemParams &,
      vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> &,
      ceres::Problem &,
      std::vector<vslam_types::SLAMNode> *)>
      structured_vision_constraint_adder =
          vslam_solver::AddStructuredVisionFactors;

  std::function<std::vector<vslam_types::VisionFeature>(
      const vslam_types::StructuredVisionFeatureTrack &)>
      feature_retriever =
          [](const vslam_types::StructuredVisionFeatureTrack &feature_track) {
            return feature_track.feature_track.track;
          };

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

  for (size_t start_frame_idx = 0; 
       start_frame_idx + problem_params.n_interval_frames < prob.robot_poses.size(); 
       start_frame_idx += problem_params.n_interval_frames) {
    prob.start_frame_id = start_frame_idx;
    solver.SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                    vslam_solver::StructuredSlamProblemParams>(
        structured_vision_constraint_adder,
        callback_creator,
        problem_params,
        prob,
        adjusted_to_zero_answer);
  }

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "answer.txt",
                               adjusted_to_zero_answer);
  }


/*
  ros::Rate loop_rate(20.0);
  while (slam_problem_ptr && ros::ok()) {
    ros::spinOnce();
    // solve slam problem
    
    loop_rate.sleep();
  }
*/
  return 0;
}