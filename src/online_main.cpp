#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <experimental/filesystem> 
#include <map>
#include <string>

#include "slam_backend_solver.h"
#include "slam_solver_optimizer_params.h"
#include "visual_slam_ceres_visualization_callback.h"
#include "vslam_io.h"
#include "vslam_types.h"
#include "vslam_util.h"
#include "vslam_unproject.h"

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

// TODO hardcode; move me to some params file
const vslam_types::CameraId primary_camera_id = 1;

// This is a dummy structure to simulate what we want from ros
struct ObservationTrack {
  vslam_types::RobotPose velocity; // transformation from prev pose to curr pose
  vslam_types::RobotPose robot_pose;
  // using map instead of unordered_map to ensure no collision
  std::map<vslam_types::CameraId, std::map<vslam_types::FeatureId, Eigen::Vector2f>> measurements_by_camera;
  std::map<vslam_types::CameraId, std::map<vslam_types::FeatureId, float>> depths_by_camera;
};

void LoadMeasurements(const std::string& dataset_path,
                      std::map<vslam_types::FrameId, ObservationTrack>& observationTracks) {
  /**
   * Load all observations as an ObservationTrack's measurements_by_camera into Memory at once
   * Need to take in subscribe to rostopic later
   */
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path))) {
    const auto file_extension = entry.path().extension().string();
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      exit(1);
    }

    // Start loading measurement files
    std::string line;

    // Read frame ID from 1st line
    // NOTE not the actual frame ID we want
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    vslam_types::FrameId frame_id;
    ss_id >> frame_id;
    // Read frame/robot pose from 2nd line
    // Skip it; we only want relative pose from "velocities" direcotry
    std::getline(data_file_stream, line);

    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      vslam_types::CameraId camera_id;
      vslam_types::FeatureId feature_id;
      float x, y;
      std::vector<float> xs, ys;
      std::vector<vslam_types::CameraId> camera_ids;
      // parse one line
      ss_feature >> feature_id;
      ss_feature >> camera_id;
      while ( !ss_feature.eof() ) {
        ss_feature >> x >> y;
        camera_ids.emplace_back(camera_id);
        xs.emplace_back(x);
        ys.emplace_back(y);
        ss_feature >> camera_id;
      }
      // load each line to measurements_by_camera
      if (observationTracks.find(frame_id) == observationTracks.end()) {
        observationTracks[frame_id] = ObservationTrack();
      }
      for (size_t camera_idx = 0; camera_idx < camera_ids.size(); ++camera_idx) {
        camera_id = camera_ids[camera_idx];
        x = xs[camera_idx];
        y = ys[camera_idx];
        observationTracks[frame_id].measurements_by_camera[camera_id][feature_id] = Eigen::Vector2f(x, y);
      }
    }
  }
}

void LoadDepths(const std::string& dataset_path,
                std::map<vslam_types::FrameId, ObservationTrack>& observationTracks) {
  /**
   * Load all observations as an ObservationTrack's depths_by_camera into Memory at once
   * NOTE current depth files only have left camera depth; they don't have camera ID either
   * Need to take in subscribe to rostopic later
   */
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path + "depths/"))) {
    const auto file_extension = entry.path().extension().string();
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      exit(1);
    }

    // Start loading depth files
    std::string line;
    // Read frame ID from 1st line
    // NOTE not the actual frame ID we want
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    vslam_types::FrameId frame_id;
    ss_id >> frame_id;
    // Read frame/robot pose from 2nd line
    // Skip it; we only want relative pose from "velocities" direcotry
    std::getline(data_file_stream, line);

    auto& depths_by_camera = observationTracks[frame_id].depths_by_camera;
    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_depth(line);
      vslam_types::FeatureId feature_id;
      float depth;
      ss_depth >> feature_id >> depth;
      depths_by_camera[primary_camera_id][feature_id] = depth;
    }
  }
}

void LoadVelocities(const std::string& dataset_path,
                    std::map<vslam_types::FrameId, ObservationTrack>& observationTracks) {
  for (const auto& entry : fs::directory_iterator(fs::path(dataset_path + "velocities/"))) {
    const auto file_extension = entry.path().extension().string();
    if (!fs::is_regular_file(entry) || file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      exit(1);
    }

    // Start loading depth files
    std::string line;
    // Read frame ID from 1st line
    // NOTE not the actual frame ID we want
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    vslam_types::FrameId frame_id;
    ss_id >> frame_id;
    std::getline(data_file_stream, line);
    std::stringstream ss_velocity(line);
    float x, y, z, qx, qy, qz, qw;
    ss_velocity >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Vector3f translation(x, y, z);
    Eigen::AngleAxisf rotation_a(Eigen::Quaternionf(qw, qx, qy, qz));
    observationTracks[frame_id].velocity = vslam_types::RobotPose(frame_id, translation, rotation_a);
  }
}

// also defined in unproject_main
// TODO move me to some helper file
vslam_types::RobotPose getCurrentRobotPose(const vslam_types::RobotPose& prev_robot_pose, 
                                           const vslam_types::RobotPose& velocity) {
    Eigen::Affine3f velocity_matrix = velocity.RobotToWorldTF();
    Eigen::Affine3f prev_pose_matrix = prev_robot_pose.RobotToWorldTF();
    velocity_matrix = velocity_matrix.inverse();
    prev_pose_matrix = prev_pose_matrix.inverse();
    Eigen::Affine3f curr_pose_matrix = velocity_matrix * prev_pose_matrix;
    curr_pose_matrix = curr_pose_matrix.inverse();
    vslam_types::RobotPose pose = vslam_types::RobotPose(prev_robot_pose.frame_idx+1, 
                     curr_pose_matrix.translation(),
                     Eigen::AngleAxisf(curr_pose_matrix.rotation()) );
    return pose;
}

// TODO use generic types instead
void LoadObservationTrack(const vslam_types::RobotPose& robot_pose, 
                          const std::map<vslam_types::CameraId, std::map<vslam_types::FeatureId, Eigen::Vector2f>>& measurements_by_camera,
                          const std::map<vslam_types::FeatureId, float>& depths_by_feature_id,
                          const CameraIntrinsics& intrinsics,
                          const CameraExtrinsics& extrinsics,
                          std::unordered_map<vslam_types::FeatureId, vslam_types::StructuredVisionFeatureTrack>& tracks_by_feature_id) {
  vslam_types::FeatureId feature_id;
  Eigen::Vector2f measurement;
  float depth;
  vslam_types::FrameId frame_id = robot_pose.frame_idx;

  // handle primary camera
  for (const auto& measurement_by_feature_id : measurements_by_camera.at(primary_camera_id)) {
    feature_id = measurement_by_feature_id.first;
    measurement = measurement_by_feature_id.second;
    depth = depths_by_feature_id.at(feature_id);
    // unproject point by the primary camera
    Eigen::Vector3d point = vslam_unproject::Unproject(measurement, intrinsics, extrinsics, robot_pose, depth).cast<double>();
    // set up structuredVisionFeatureTrack
    vslam_types::StructuredVisionFeatureTrack structuredVisionFeatureTrack;
    structuredVisionFeatureTrack.point = point;
    structuredVisionFeatureTrack.feature_track.feature_idx = feature_id;
    std::unordered_map<CameraId, Eigen::Vector2f> pixel_by_camera_id;
    pixel_by_camera_id[primary_camera_id] = measurement;
    structuredVisionFeatureTrack.feature_track.track.emplace_back(feature_id, frame_id, pixel_by_camera_id, primary_camera_id);
    // assign #feature_id-th structuredVisionFeatureTrack
    tracks_by_feature_id[feature_id] = structuredVisionFeatureTrack;
  }

  // handle non-primary camera: update pixel_by_camera_id
  for (const auto& measurement_by_camera : measurements_by_camera) {
    vslam_types::CameraId camera_id = measurement_by_camera.first;
    if (camera_id == primary_camera_id) { continue; }
    for (const auto& measurement_by_feature_id : measurements_by_camera.at(camera_id)) {
      feature_id = measurement_by_feature_id.first;
      measurement = measurement_by_feature_id.second;
      depth = depths_by_feature_id.at(feature_id);
      // iterate through track in VisionFeatureTrack to find VisionFeature associated with current frmae ID
      for (auto& vision_feature : tracks_by_feature_id[feature_id].feature_track.track) {
        if (vision_feature.frame_idx != frame_id) { continue; }
        vision_feature.pixel_by_camera_id[camera_id] = measurement;
      }
    }
  }
}

// TODO use generic types instead
void CleanFeatureTrackInProb(const vslam_solver::StructuredSlamProblemParams& prob_params,
                             vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack>& prob) {
  vslam_types::FeatureId feature_id;
  vslam_types::FrameId start_frame_id = prob.start_frame_idx;
  // use unordered_set bc even if there're duplicates in rare cases, we don't care
  std::unordered_set<vslam_types::FeatureId> feature_ids_to_delete;
  for (auto& track_by_feature_id : prob.tracks) {
    feature_id = track_by_feature_id.first;
    auto& vision_features = track_by_feature_id.second.feature_track.track;
    // delete tracks related to old frame IDs
    for (auto iter = vision_features.begin(); iter != vision_features.end(); ) {
      if (iter->frame_idx < start_frame_id) {
        iter = vision_features.erase(iter);
      } else {
        ++iter;
      }
    }
    // after deletion, check if there's no vision feature associated with some feature ID
    if (vision_features.size() == 0) {
      feature_ids_to_delete.emplace(feature_id);
    }
  }
  // delete all feature_ids whose associated feature track is empty
  for (auto iter = prob.tracks.begin(); iter != prob.tracks.end(); ) {
    if (feature_ids_to_delete.find(iter->first) != feature_ids_to_delete.end()) {
      iter = prob.tracks.erase(iter);
    } else {
      ++iter;
    }
  }
}

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

  std::map<vslam_types::FrameId, ObservationTrack> observationTracks;
  
  LoadMeasurements(FLAGS_dataset_path, observationTracks);
  LoadDepths(FLAGS_dataset_path, observationTracks);
  LoadVelocities(FLAGS_dataset_path, observationTracks);
  
  // use a hardcoded solution for now
  // TODO: fix this in ORB_SLAM
  // vslam_types::RobotPose first_pose = vslam_types::RobotPose(1, 
  //                                     Eigen::Vector3f(-0.00654, -0.00277, 0.965), 
  //                                     Eigen::AngleAxisf(Eigen::Quaternionf(1, 0.00226, 0.000729, -0.00036)));
  vslam_types::RobotPose first_pose = vslam_types::RobotPose(1, 
                                      Eigen::Vector3f(0, 0, 0), 
                                      Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0)));
  /**
   * set up solveSLAM function parameters
   */

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

  std::function<std::vector<vslam_types::VisionFeature>(
      const vslam_types::StructuredVisionFeatureTrack &)>
      feature_retriever =
          [](const vslam_types::StructuredVisionFeatureTrack &feature_track) {
            return feature_track.feature_track.track;
          };

  std::function<void(
      const vslam_solver::StructuredSlamProblemParams &,
      vslam_types::UTSLAMProblem<vslam_types::StructuredVisionFeatureTrack> &,
      ceres::Problem &,
      std::vector<vslam_types::SLAMNode> *)>
      structured_vision_constraint_adder =
          vslam_solver::AddStructuredVisionFactorsOnline;

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
      callback_creator;

  // TODO move them to some params header file
  const float MIN_TRANSLATION_OPT = 1.0;
  const float MIN_ROTATION_OPT = M_1_PI / 18; // 10 degree
  // use t to index through observationTracks
  size_t t = 1;
  const size_t ntimes = observationTracks.size();
  // Use while instead of for, as we want "while" ultimately 
  prob.start_frame_idx = 0; // this id starts from 0

  vslam_solver::StructuredSlamProblemParams problem_params;
  vslam_types::RobotPose prev_added_robot_pose; // TODO move me to some header file/struct
  vslam_types::FrameId next_frame_idx;
  std::vector<vslam_types::RobotPose> answer;
  bool is_last_iter_optimized = false;
  cout << "start loop" << endl;
  while (t < ntimes) {
    next_frame_idx =  prob.robot_poses.size();
    if (t == 1) {
      observationTracks[t].robot_pose = first_pose;
      prev_added_robot_pose = observationTracks[t].robot_pose;
      // add first pose to robot_pose
      prob.robot_poses.emplace_back(next_frame_idx, 
                                    observationTracks[t].robot_pose.loc, 
                                    observationTracks[t].robot_pose.angle);
    } else {
      prev_added_robot_pose = prob.robot_poses[next_frame_idx-1];
      vslam_types::RobotPose prev_robot_pose;
      /**
       * If in the last iteration, we have solved SLAM. Then, prev_robot_pose is 
       * different from what's stored in observationTracks. We should use the last 
       * robot pose stored in prob.robot_poses
       */
      if (is_last_iter_optimized) {
        prev_robot_pose = prob.robot_poses[next_frame_idx-1];
      } else {
        prev_robot_pose = observationTracks[t-1].robot_pose;
      }
      observationTracks[t].robot_pose = getCurrentRobotPose(prev_robot_pose, 
                                                            observationTracks[t].velocity);
    }
    
    float dist_traveled = (observationTracks[t].robot_pose.loc - prev_added_robot_pose.loc).norm();
    Eigen::Quaternionf q_prev(prev_added_robot_pose.angle);
    Eigen::Quaternionf q_curr(observationTracks[t].robot_pose.angle);
    float angle_rotated = abs(q_curr.angularDistance(q_prev));

    if (dist_traveled < MIN_TRANSLATION_OPT && angle_rotated < MIN_ROTATION_OPT) {
      if ( t == 1 ) {
        // estimate landmarks
        LoadObservationTrack(prob.robot_poses[next_frame_idx], 
                             observationTracks[t].measurements_by_camera, 
                             observationTracks[t].depths_by_camera[primary_camera_id],
                             prob.camera_instrinsics_by_camera[primary_camera_id], 
                             prob.camera_extrinsics_by_camera[primary_camera_id], prob.tracks);
      }
      is_last_iter_optimized = false;
      ++t;
      continue;
    }
    // add current pose to robot_pose
    prob.robot_poses.emplace_back(next_frame_idx, 
                                  observationTracks[t].robot_pose.loc, 
                                  observationTracks[t].robot_pose.angle);

    // if add current pose to robot_pose, estimate landmarks
    LoadObservationTrack(prob.robot_poses[next_frame_idx], 
                         observationTracks[t].measurements_by_camera, 
                         observationTracks[t].depths_by_camera[primary_camera_id],
                         prob.camera_instrinsics_by_camera[primary_camera_id], 
                         prob.camera_extrinsics_by_camera[primary_camera_id], prob.tracks);

    // check if we need SLAM optimization
    vslam_types::FrameId current_end_frame_idx = prob.robot_poses.size();
    if (current_end_frame_idx >= prob.start_frame_idx + problem_params.n_interval_frames) {
      // solve SLAM
      cout << "solving frame " << prob.start_frame_idx << " to " << prob.start_frame_idx + problem_params.n_interval_frames;
      cout << "; total times: " << ntimes << endl;
      solver.SolveSLAM<vslam_types::StructuredVisionFeatureTrack,
                    vslam_solver::StructuredSlamProblemParams>(
        structured_vision_constraint_adder,
        callback_creator,
        problem_params,
        prob,
        prob.robot_poses);
      is_last_iter_optimized = true;
      ++prob.start_frame_idx;
    }
    ++t;
    CleanFeatureTrackInProb(problem_params, prob);
  }

  if (FLAGS_save_poses) {
    vslam_util::SaveKITTIPoses(FLAGS_output_path + "answer.txt",
                               prob.robot_poses);
  }
  return 0;
}