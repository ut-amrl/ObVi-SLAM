#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <experimental/filesystem> 

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
namespace fs = std::experimental::filesystem;

// This is a dummy structure to simulate what we want from ros
struct ObservationTrack {
  vslam_types::RobotPose velocity; // transformation from prev pose to curr pose
  std::unordered_map<vslam_types::CameraId, std::unordered_map<vslam_types::FeatureId, Eigen::Vector2f>> measurements_by_camera;
  std::unordered_map<vslam_types::CameraId, std::unordered_map<vslam_types::FeatureId, float>> depths_by_camera;
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  // Make empty structured slam problem
  vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> prob;

  // Set up optimization params
  vslam_solver::SLAMSolverOptimizerParams optimizer_params;
  vslam_solver::SLAMSolver solver(optimizer_params);

  // Load Camera Extrinsics and Instrinsics
  vslam_io::LoadCameraCalibrationData(FLAGS_dataset_path, prob.camera_instrinsics_by_camera, prob.camera_extrinsics_by_camera);

  /**
   * Temporary Solution for sliding window: 
   * Load everything as an ObservationTrack into Memory at once
   * Need to take in subscribe to rostopic later
   */
  for (const auto& entry : fs::directory_iterator(fs::path(FLAGS_dataset_path))) {
    const auto file_extension = entry.path().extension().string();
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return false;
    }

    // Start loading data files
    std::string line;

    // Read frame ID from 1st line
    // Skip it; not the frame ID we want
    std::getline(data_file_stream, line);

    // Read frame/robot pose from 2nd line
    std::getline(data_file_stream, line);
    std::stringstream ss_pose(line);
    float x, y, z, qx, qy, qz, qw;
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f loc(x, y, z);
    Eigen::Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    Eigen::AngleAxisf angle(angle_q);
    vslam_types::RobotPose pose(0, loc, angle); // Don't want frame ID in this case

  }

  // init starting position from zero
  // NOTE: frame_id starts at 0 translation and 0 rotation
  vslam_types::RobotPose start_pose(0, Eigen::Vector3f(0, 0, 0), Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0)));

  // Use while instead of for, as we want "while" ultimately 
  // while () {
    // 
  // }

# if 0
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

  const float MIN_TRANSLATION_OPT = 1.0;
  const float MIN_ROTATION_OPT = M_1_PI / 36; // 5 degree
  size_t start_frame_idx = 0;
  size_t end_frame_idx = start_frame_idx;
  prob.valid_frame_ids.insert(0);
  for (end_frame_idx = start_frame_idx; end_frame_idx < adjusted_to_zero_answer.size(); ++end_frame_idx) {
    Eigen::Quaternionf q_start(adjusted_to_zero_answer[start_frame_idx].angle);
    Eigen::Quaternionf q_end(adjusted_to_zero_answer[end_frame_idx].angle);
    if ((adjusted_to_zero_answer[end_frame_idx].loc - adjusted_to_zero_answer[start_frame_idx].loc).norm() < MIN_TRANSLATION_OPT 
        && abs(q_start.angularDistance(q_end)) < MIN_ROTATION_OPT ) { continue; }
    prob.valid_frame_ids.insert(end_frame_idx);
    start_frame_idx = end_frame_idx;
  }

  if (1) {
    std::ofstream of_frame_ids;
    of_frame_ids.open(FLAGS_output_path + "valid_frame_ids.txt", std::ios::trunc);
    for (const auto& valid_frame_id : prob.valid_frame_ids) {
      of_frame_ids << valid_frame_id << endl;
    }
    of_frame_ids.close();
  }

  size_t n_interval_frames = 0;
  vslam_types::FrameId second_valid_frame_id = 0;
  prob.output = FLAGS_output_path;
  prob.start_frame_id = 0; 
  prob.end_frame_id = prob.start_frame_id;
  while (prob.start_frame_id + problem_params.n_interval_frames < prob.robot_poses.size() &&
         prob.end_frame_id < prob.robot_poses.size() ) {
    ++prob.end_frame_id;
    if (n_interval_frames < problem_params.n_interval_frames) {
      // if prob.end_frame_id is a valid frame id
      if (prob.valid_frame_ids.find(prob.end_frame_id) != prob.valid_frame_ids.end()) { 
        // store second_valid_frame_id to update prob.start_frame_id
        if (n_interval_frames == 0) { second_valid_frame_id = prob.end_frame_id; }
        ++n_interval_frames; 
      }
      continue;
    }
    n_interval_frames = 0;
    cout << "start frame id: " << prob.start_frame_id << ", end frame id: " << prob.end_frame_id << endl;;
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
    prob.start_frame_id = second_valid_frame_id;
    prob.end_frame_id   = second_valid_frame_id; // FIXME: redundant computation
  }

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "answer_batch_increment.txt",
                               adjusted_to_zero_answer);
  }
# endif
  return 0;
}