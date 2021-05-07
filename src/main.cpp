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

  // Make empty unstructured slam problem
  vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> prob;
  // Make empty camera calibration matrix
  Eigen::Matrix3f K;
  // Load unstructured slam problem and intrinsic calibration K
  vslam_io::LoadStructurelessUTSLAMProblem(FLAGS_dataset_path, prob, K);

  // Make intrinsics and unit camera extrinsics
  vslam_types::CameraIntrinsics intrinsics{K};
  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  vslam_types::CameraExtrinsics extrinsics{
      Eigen::Vector3f(0, 0, 0),
      Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5)
          .inverse()};  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  // Note that for the microsoft data set the extrinsics are identity. This also
  // needs to be change/paramaterized for the pose viewer
  // vslam_types::CameraExtrinsics extrinsics{
  //   Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0).inverse()};

  // Solve
  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::SLAMSolver solver(optimizer_params);

  // These are the poses that are going to be optimized
  std::vector<vslam_types::RobotPose> answer(prob.robot_poses);
  std::vector<vslam_types::RobotPose> gt_robot_poses;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, gt_robot_poses);

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "gt.txt", gt_robot_poses);
  }

  // A1 - rotation error due to rotation -- A1 -translation error due to
  // translation
  Eigen::Matrix<double, 2, 1> odom_alphas(0.25, 0.25);
  vslam_util::CorruptRobotPoses(odom_alphas, answer);
  std::vector<vslam_types::RobotPose> adjusted_to_zero_answer;
  vslam_util::AdjustTrajectoryToStartAtZero(answer, adjusted_to_zero_answer);

  // Uncomment to add odom factors from synthetic data
  //  prob.odom_factors =
  //      vslam_util::getOdomFactorsFromInitPosesAndNoise(answer, odom_alphas);

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "start.txt",
                               adjusted_to_zero_answer);
  }

  std::function<void(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_solver::StructurelessSlamProblemParams &,
      vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      ceres::Problem &,
      std::vector<vslam_types::SLAMNode> *)>
      structureless_vision_constraint_adder =
          vslam_solver::AddStructurelessVisionFactors;

  std::function<std::vector<vslam_types::VisionFeature>(
      const vslam_types::VisionFeatureTrack &)>
      feature_retriever =
          [](const vslam_types::VisionFeatureTrack &feature_track) {
            return feature_track.track;
          };

  // Got the casting solution from
  // https://stackoverflow.com/questions/30393285/stdfunction-fails-to-distinguish-overloaded-functions
  typedef std::shared_ptr<vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::VisionFeatureTrack>> (*funtype)(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::VisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *);
  funtype func = vslam_viz::VisualSlamCeresVisualizationCallback<
      vslam_types::VisionFeatureTrack>::create;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      const std::function<std::vector<vslam_types::VisionFeature>(
          const vslam_types::VisionFeatureTrack &)> &,
      const std::vector<vslam_types::RobotPose> &,
      std::vector<vslam_types::SLAMNode> *)>
      unbound_callback_creator = func;

  std::function<std::shared_ptr<ceres::IterationCallback>(
      const vslam_types::CameraIntrinsics &,
      const vslam_types::CameraExtrinsics &,
      const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
      std::vector<vslam_types::SLAMNode> *)>
      callback_creator = std::bind(unbound_callback_creator,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   std::placeholders::_3,
                                   feature_retriever,
                                   gt_robot_poses,
                                   std::placeholders::_4);

  vslam_solver::StructurelessSlamProblemParams problem_params;

  solver.SolveSLAM<vslam_types::VisionFeatureTrack>(
      intrinsics,
      extrinsics,
      structureless_vision_constraint_adder,
      callback_creator,
      problem_params,
      prob,
      adjusted_to_zero_answer);

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "answer.txt",
                               adjusted_to_zero_answer);
  }

  return 0;
}