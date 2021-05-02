#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>

#include "slam_backend_solver.h"
#include "slam_solver_optimizer_params.h"
#include "structureless_ceres_visualization_callback.h"
#include "vslam_io.h"
#include "vslam_types.h"
#include "vslam_util.h"

DEFINE_string(
    data_path,
    "",
    "Path to folder containing the files with frames, poses, and labeled "
    "keypoints for each image");

DEFINE_string(calibration_path,
              "",
              "Path to the file containing the calibration (K) for the camera");

using std::cout;
using std::endl;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty unstructured slam problem
  vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> prob;
  // Load unstructured slam problem
  vslam_io::LoadStructurelessUTSLAMProblem(FLAGS_data_path, prob);
  // Make empty camera calibration matrix
  Eigen::Matrix3f K;
  // Load camera calibration matrix
  vslam_io::LoadCameraCalibration(FLAGS_calibration_path, K);

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
  Eigen::Matrix<double, 3, 1> sigma_linear(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 1> sigma_rotation(0.0, 0.0, 0.0);
  vslam_util::CorruptRobotPoses(sigma_linear, sigma_rotation, answer);

  std::function<void(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_solver::StructurelessSlamProblemParams &,
      vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      ceres::Problem &,
      std::vector<vslam_types::SLAMNode> *)>
      structureless_vision_constraint_adder =
          vslam_solver::AddStructurelessVisionFactors;

  // TODO if more args are needed for the visualization, then we should bind
  // them here so the solver only needs to pass the intrinsics, extrinsics, slam
  // problem, and slam nodes.
  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      std::vector<vslam_types::SLAMNode> *)>
      callback_creator =
          vslam_viz::StructurelessCeresVisualizationCallback::create;

  vslam_solver::StructurelessSlamProblemParams problem_params;

  solver.SolveSLAM<vslam_types::VisionFeatureTrack>(
      intrinsics,
      extrinsics,
      structureless_vision_constraint_adder,
      callback_creator,
      problem_params,
      prob,
      answer);

  // Print poses to terminal for display
  for (const auto &pose : answer) {
    cout << pose << endl;
  }

  return 0;
}