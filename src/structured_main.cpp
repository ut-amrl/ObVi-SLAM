#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>

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
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty structured slam problem
  vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> prob;
  // Make empty camera calibration matrix
  Eigen::Matrix3f K;
  // Load structured slam problem and intrinsic calibration K
  vslam_io::LoadStructuredUTSLAMProblem(FLAGS_dataset_path, prob, K);

  // Make intrinsics and unit camera extrinsics
  vslam_types::CameraIntrinsics intrinsics{K};
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse()};  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  // Solve
  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::SLAMSolver solver(optimizer_params);

  // These are the poses that are going to be optimized
  std::vector<vslam_types::RobotPose> answer(prob.robot_poses);
  vslam_types::RobotPose init_pose_unadjusted = answer[0];
  std::vector<vslam_types::RobotPose> gt_robot_poses;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, gt_robot_poses);

  Eigen::Matrix<double, 3, 1> sigma_linear(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 1> sigma_rotation(0.0, 0.0, 0.0);
  vslam_util::CorruptRobotPoses(sigma_linear, sigma_rotation, answer);
  std::vector<vslam_types::RobotPose> adjusted_to_zero_answer;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, adjusted_to_zero_answer);

  for (int i = 0; i < prob.tracks.size(); i++) {
    prob.tracks[i].point = vslam_util::getPositionRelativeToPose(
        init_pose_unadjusted, prob.tracks[i].point);
  }

  // TODO corrupt feature positions too

  std::function<void(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
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

  // Got the casting solution from
  // https://stackoverflow.com/questions/30393285/stdfunction-fails-to-distinguish-overloaded-functions
  typedef std::shared_ptr<vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>> (*funtype)(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *);
  funtype func = vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::StructuredVisionFeatureTrack>::create;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::StructuredVisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *)>
      unbound_callback_creator = func;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<
          vslam_types::StructuredVisionFeatureTrack> &,
      std::vector<vslam_types::SLAMNode> *)>
      callback_creator = std::bind(unbound_callback_creator,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   feature_retriever,
                                   gt_robot_poses,
                                   std::placeholders::_4);

  vslam_solver::StructuredSlamProblemParams problem_params;

  solver.SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                   vslam_solver::StructuredSlamProblemParams>(
      intrinsics,
      extrinsics,
      structured_vision_constraint_adder,
      callback_creator,
      problem_params,
      prob,
      adjusted_to_zero_answer);

  // Print poses to terminal for display
  for (const auto &pose : adjusted_to_zero_answer) {
    cout << pose << endl;
  }
  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path, adjusted_to_zero_answer);
  }

  return 0;
}