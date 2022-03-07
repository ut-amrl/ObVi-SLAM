#include <type_traits>
#include <ceres/ceres.h>
#include <pairwise_2d_feature_cost_functor.h>
#include <reprojection_cost_functor.h>
#include <slam_backend_solver.h>
#include <iostream>
#include <fstream>

using namespace std; // TODO: delete me

namespace vslam_solver {

void RobotPosesToSLAMNodes(
    const std::vector<vslam_types::RobotPose> &robot_poses,
    std::vector<vslam_types::SLAMNode> &nodes) {
  for (const vslam_types::RobotPose &robot_pose : robot_poses) {
    nodes.emplace_back(FromRobotPose(robot_pose));
  }
}

void SLAMNodesToRobotPoses(const std::vector<vslam_types::SLAMNode> &slam_nodes,
                           std::vector<vslam_types::RobotPose> &updated_poses) {
  updated_poses.clear();
  for (const vslam_types::SLAMNode &node : slam_nodes) {
    updated_poses.emplace_back(FromSLAMNode(node));
  }
}

vslam_types::SLAMNode FromRobotPose(const vslam_types::RobotPose &robot_pose) {
  return vslam_types::SLAMNode(
      robot_pose.frame_idx, robot_pose.loc, robot_pose.angle);
}

vslam_types::RobotPose FromSLAMNode(const vslam_types::SLAMNode &slam_node) {
  Eigen::Vector3f rotation_axis(
      slam_node.pose[3], slam_node.pose[4], slam_node.pose[5]);

  Eigen::AngleAxisf rotation_aa = vslam_types::VectorToAxisAngle(rotation_axis);

  Eigen::Vector3f transl(
      slam_node.pose[0], slam_node.pose[1], slam_node.pose[2]);
  return vslam_types::RobotPose(slam_node.node_idx, transl, rotation_aa);
}

// TODO make all class methods lower case ?
template <typename FeatureTrackType, typename ProblemParams>
bool SLAMSolver::SolveSLAM(
    const std::function<void(const ProblemParams &,
                             vslam_types::UTSLAMProblem<FeatureTrackType> &,
                             ceres::Problem &,
                             std::vector<vslam_types::SLAMNode> *)>
        vision_constraint_adder,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTSLAMProblem<FeatureTrackType> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const ProblemParams &problem_params,
    vslam_types::UTSLAMProblem<FeatureTrackType> &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses) {
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  // TODO configure options

  options.max_num_iterations = solver_optimization_params_.max_iterations;
  options.minimizer_type = solver_optimization_params_.minimizer_type;
  options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.max_num_consecutive_invalid_steps = solver_optimization_params_.max_num_consecutive_invalid_steps;

  std::vector<vslam_types::SLAMNode> slam_nodes;
  RobotPosesToSLAMNodes(updated_robot_poses, slam_nodes);

  vision_constraint_adder(problem_params, slam_problem, problem, &slam_nodes);

  // Set the first pose constant
  // problem.SetParameterBlockConstant(slam_nodes[0].pose);
  problem.SetParameterBlockConstant(slam_nodes[slam_problem.start_frame_id].pose); 

  std::shared_ptr<ceres::IterationCallback> viz_callback =
      callback_creator(slam_problem, &slam_nodes);

  if (viz_callback != nullptr) {
    options.callbacks.emplace_back(viz_callback.get());
    options.update_state_every_iteration = true;
  }

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  SLAMNodesToRobotPoses(slam_nodes, updated_robot_poses);
  if (1) { // TODO: delete me
    std::ofstream of_success;
    // dump out if optimization is successful or not
    of_success.open(slam_problem.output + "success/" + std::to_string(slam_problem.start_frame_id) + ".txt", ios::trunc);
    if (summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS) {
      of_success << "succeed" << std::endl;
    } else {
      of_success << "failed" << std::endl;
      of_success << summary.FullReport() << std::endl;
    }
    of_success.close();
    // dump out start and end frames of this opimization
    std::ofstream of_frame;
    of_frame.open(slam_problem.output + "frame/" + std::to_string(slam_problem.start_frame_id) + ".txt", ios::trunc);
    of_frame << slam_problem.start_frame_id << " " << slam_problem.end_frame_id << std::endl;
    of_frame.close();
    // dump out trajectory after this optimization
    vslam_util::SaveKITTIPoses(slam_problem.output + "traj/" + std::to_string(slam_problem.start_frame_id) + ".txt",
                               updated_robot_poses);
  }

  return (summary.termination_type == ceres::CONVERGENCE ||
          summary.termination_type == ceres::USER_SUCCESS);
}

void AddStructurelessVisionFactors(
    const StructurelessSlamProblemParams &solver_optimization_params,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes) {
  std::vector<vslam_types::SLAMNode> &solution = *updated_solved_nodes;

  for (const auto &feature_track_by_id : slam_problem.tracks) {
    for (int i = 0; i < feature_track_by_id.second.track.size() - 1; i++) {
      vslam_types::VisionFeature f1 = feature_track_by_id.second.track[i];
      vslam_types::CameraId f1_primary_camera_id = f1.primary_camera_id;
      vslam_types::CameraIntrinsics feature_1_primary_cam_intrinsics =
          slam_problem.camera_instrinsics_by_camera.at(f1_primary_camera_id);
      vslam_types::CameraExtrinsics feature_1_primary_cam_extrinsics =
          slam_problem.camera_extrinsics_by_camera.at(f1_primary_camera_id);
      Eigen::Vector2f feature_1_primary_pixel =
          f1.pixel_by_camera_id.at(f1_primary_camera_id);

      for (const auto &pixel_with_camera_id : f1.pixel_by_camera_id) {
        vslam_types::CameraId other_feat_camera_id = pixel_with_camera_id.first;
        if (other_feat_camera_id == f1_primary_camera_id) {
          continue;
        }
        double *pose_block = solution[f1.frame_idx].pose;

        ceres_problem.AddResidualBlock(
            Pairwise2dFeatureCostFunctor::create(
                feature_1_primary_cam_intrinsics,
                feature_1_primary_cam_extrinsics,
                slam_problem.camera_instrinsics_by_camera.at(
                    other_feat_camera_id),
                slam_problem.camera_extrinsics_by_camera.at(
                    other_feat_camera_id),
                f1.pixel_by_camera_id.at(f1_primary_camera_id),
                f1.pixel_by_camera_id.at(other_feat_camera_id),
                solver_optimization_params.epipolar_error_std_dev),
            new ceres::HuberLoss(1.0),
            pose_block,
            pose_block);
      }

      for (int j = i + 1; j < feature_track_by_id.second.track.size(); j++) {
        vslam_types::VisionFeature f2 = feature_track_by_id.second.track[j];

        double *initial_pose_block = solution[f1.frame_idx].pose;
        double *curr_pose_block = solution[f2.frame_idx].pose;

        ceres_problem.AddResidualBlock(
            Pairwise2dFeatureCostFunctor::create(
                feature_1_primary_cam_intrinsics,
                feature_1_primary_cam_extrinsics,
                slam_problem.camera_instrinsics_by_camera.at(
                    f2.primary_camera_id),
                slam_problem.camera_extrinsics_by_camera.at(
                    f2.primary_camera_id),
                f1.pixel_by_camera_id.at(f1.primary_camera_id),
                f2.pixel_by_camera_id.at(f2.primary_camera_id),
                solver_optimization_params.epipolar_error_std_dev),
            new ceres::HuberLoss(1.0),
            initial_pose_block,
            curr_pose_block);
      }
    }
  }
}

// TODO: FIXME
void AddStructuredVisionFactorsOnline(
    const StructuredSlamProblemParams &solver_optimization_params,
    vslam_types::UTSLAMProblemOnline<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes) {
  std::cout << "AddStructuredVisionFactors online" << std::endl;
  std::vector<vslam_types::SLAMNode> &solution = *updated_solved_nodes;

  for (auto &feature_track_by_id : slam_problem.tracks) {
    double *feature_position_block = feature_track_by_id.second.point.data();
    for (const vslam_types::VisionFeature &feature :
         feature_track_by_id.second.feature_track.track) {
      double *pose_block = solution[feature.frame_idx].pose;
      if (feature.frame_idx <  slam_problem.start_frame_id || 
          feature.frame_idx >= slam_problem.start_frame_id + solver_optimization_params.n_interval_frames) {
        continue;
      }
      for (const auto &camera_id_and_pixel : feature.pixel_by_camera_id) {
        ceres_problem.AddResidualBlock(
            ReprojectionCostFunctor::create(
                slam_problem.camera_instrinsics_by_camera.at(
                    camera_id_and_pixel.first),
                slam_problem.camera_extrinsics_by_camera.at(
                    camera_id_and_pixel.first),
                camera_id_and_pixel.second,
                solver_optimization_params.reprojection_error_std_dev),
            new ceres::HuberLoss(1.0),
            pose_block,
            feature_position_block);
      }
    }
  }
}

// TODO: FIXME
void AddStructuredVisionFactorsOffline(
    const StructuredSlamProblemParams &solver_optimization_params,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    ceres::Problem &ceres_problem,
    std::vector<vslam_types::SLAMNode> *updated_solved_nodes) {
  std::vector<vslam_types::SLAMNode> &solution = *updated_solved_nodes;
  std::cout << "AddStructuredVisionFactors offline" << std::endl;
  for (auto &feature_track_by_id : slam_problem.tracks) {
    double *feature_position_block = feature_track_by_id.second.point.data();
    for (const vslam_types::VisionFeature &feature :
         feature_track_by_id.second.feature_track.track) {
      double *pose_block = solution[feature.frame_idx].pose;
      if (feature.frame_idx <  slam_problem.start_frame_id || 
          feature.frame_idx >= slam_problem.end_frame_id) { continue; }
      for (const auto &camera_id_and_pixel : feature.pixel_by_camera_id) {
        ceres_problem.AddResidualBlock(
            ReprojectionCostFunctor::create(
                slam_problem.camera_instrinsics_by_camera.at(
                    camera_id_and_pixel.first),
                slam_problem.camera_extrinsics_by_camera.at(
                    camera_id_and_pixel.first),
                camera_id_and_pixel.second,
                solver_optimization_params.reprojection_error_std_dev),
            new ceres::HuberLoss(1.0),
            pose_block,
            feature_position_block);
      }
    }
  }
}

/*
void AddStructuredVisionFactorsFrameTrack(const StructuredSlamProblemParams &solver_optimization_params,
                                          vslam_types::UTSLAMProblemOnline<vslam_types::StructuredFrameTrack> &slam_problem,
                                          ceres::Problem &ceres_problem) {
  std::vector<vslam_types::SLAMNode> &slam_nodes = slam_problem.slam_nodes;
  std::unordered_map<vslam_types::FeatureId, Eigen::Vector3d> &points = slam_problem.points;
  std::vector<vslam_types::StructuredFrameTrack> &tracks = slam_problem.tracks;

  // iterate through all poses in the current window
  for (vslam_types::FrameId frame_id = slam_problem.start_frame_id; 
       frame_id < slam_problem.start_frame_id + solver_optimization_params.n_interval_frames;
       ++frame_id) {
    double *pose_block = slam_nodes[frame_id].pose;
    // iterate through all features in the current pose
    for (const vslam_types::VisionFeature &feature : tracks[frame_id].track) {
      double *point_block = points[feature.feature_idx].data();
      // iterate through all <camera, feature> where camera capture that feature
      for (const auto &camera_id_and_pixel : feature.pixel_by_camera_id) {
        ceres_problem.AddResidualBlock(
            ReprojectionCostFunctor::create(
                slam_problem.camera_intrinsics_by_camera[camera_id_and_pixel.first],
                slam_problem.camera_extrinsics_by_camera[camera_id_and_pixel.first],
                camera_id_and_pixel.second,
                solver_optimization_params.reprojection_error_std_dev),
            new ceres::HuberLoss(1.0),
            pose_block,
            point_block);
      }
    }
  }
}
*/
template bool SLAMSolver::SolveSLAM<vslam_types::VisionFeatureTrack,
                                    StructurelessSlamProblemParams>(
    const std::function<
        void(const StructurelessSlamProblemParams &,
             vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
             ceres::Problem &,
             std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const StructurelessSlamProblemParams &problem_params,
    vslam_types::UTSLAMProblem<vslam_types::VisionFeatureTrack> &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses);

template bool SLAMSolver::SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                                    StructuredSlamProblemParams>(
    const std::function<void(
        const StructuredSlamProblemParams &,
        vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> &,
        ceres::Problem &,
        std::vector<vslam_types::SLAMNode> *)> vision_constraint_adder,
    const std::function<std::shared_ptr<ceres::IterationCallback>(
        const vslam_types::UTSLAMProblem<
            vslam_types::StructuredVisionFeatureTrack> &,
        std::vector<vslam_types::SLAMNode> *)> callback_creator,
    const StructuredSlamProblemParams &problem_params,
    vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>
        &slam_problem,
    std::vector<vslam_types::RobotPose> &updated_robot_poses);
}  // namespace vslam_solver