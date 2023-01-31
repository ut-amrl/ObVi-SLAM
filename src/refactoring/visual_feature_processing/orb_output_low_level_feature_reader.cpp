//
// Created by amanda on 10/10/22.
//

#include <file_io/features_ests_with_id_io.h>
#include <glog/logging.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>

#include <experimental/filesystem>

namespace vslam_types_refactor {

bool OrbOutputLowLevelFeatureReader::getLowLevelFeatures(
    std::unordered_map<FeatureId, StructuredVisionFeatureTrack>
        &feature_tracks) {
  if (!loaded_) {
    if (!loadData()) {
      return false;
    }
  }
  feature_tracks = feature_tracks_;
  return true;
}

bool OrbOutputLowLevelFeatureReader::loadData() {
  FeatureFileContents feature_file_contents;
  if (!readFeatureFileContentsFromDirectory(orb_data_directory_name_,
                                            feature_file_contents)) {
    LOG(ERROR) << "Failed to load initial feature positions";
    return false;
  }
  std::unordered_map<FrameId, FeatureObservationsForFrame>
      single_frame_feature_observations;
  if (!readSingleFileFrameContentsFromDirectory(
          orb_data_directory_name_, single_frame_feature_observations)) {
    LOG(ERROR) << "Failed to load feature observations";
    return false;
  }
  LOG(INFO) << "Done reading single frame contents from directory";

  std::unordered_map<
      FeatureId,
      std::unordered_map<FrameId,
                         std::unordered_map<CameraId, PixelCoord<double>>>>
      feature_obs;
  std::unordered_map<FeatureId, uint64_t> feature_observation_count;
  for (const auto &frame_data : single_frame_feature_observations) {
    FrameId frame = frame_data.first;
    for (const auto &feat_id_to_feat_obs : frame_data.second.features_) {
      // Warning: This assumes that there's only one source of data for each
      // frame (will overwrite if somehow encounters data for a single frame in
      // 2 places); Data is stored by frame now, so this is fine, but we may
      // want to consider merging instead of setting equal to
      feature_obs[feat_id_to_feat_obs.first][frame] =
          feat_id_to_feat_obs.second;
      feature_observation_count[feat_id_to_feat_obs.first] =
          feature_observation_count[feat_id_to_feat_obs.first] + 1;
    }
  }
  LOG(INFO) << "Num features before cleaning " << feature_obs.size();
  for (const auto &feat_and_num_obs : feature_observation_count) {
    if (feat_and_num_obs.second == 1) {
      feature_obs.erase(feat_and_num_obs.first);
    }
  }
  LOG(INFO) << "Num features after cleaning " << feature_obs.size();

  for (const auto &initial_feature_estimates :
       feature_file_contents.feature_initial_position_estimates_) {
    FeatureId feat_id = initial_feature_estimates.first;
    Position3d<double> initial_est_for_feat = initial_feature_estimates.second;
    if (feature_obs.find(feat_id) == feature_obs.end()) {
      continue;
    }
    std::unordered_map<FrameId,
                       std::unordered_map<CameraId, PixelCoord<double>>>
        obs_for_feat = feature_obs.at(feat_id);

    bool has_obs = false;
    std::unordered_map<FrameId, VisionFeature> feat_structs_for_feat;
    for (const auto &obs_at_frame_for_feat : obs_for_feat) {
      FrameId frame = obs_at_frame_for_feat.first;
      std::unordered_map<CameraId, PixelCoord<double>>
          pixel_by_camera_id_for_frame = obs_at_frame_for_feat.second;
      if (pixel_by_camera_id_for_frame.empty()) {
        LOG(WARNING) << "Observation for feature " << feat_id << " at frame "
                     << obs_at_frame_for_feat.first
                     << " was present, but empty; ignoring";
        continue;
      }
      has_obs = true;
      CameraId primary_cam;
      for (const auto &cam_feat_entry : pixel_by_camera_id_for_frame) {
        // Default to the first camera id in the iterator
        primary_cam = cam_feat_entry.first;
        break;
      }

      // See if any of the cameras in the precedence list are in the observation
      // and if so, use the first one aka highest precedence for the primary
      // camera
      for (const CameraId &cam_in_precedence_list : camera_precedence_order_) {
        if (pixel_by_camera_id_for_frame.find(cam_in_precedence_list) !=
            pixel_by_camera_id_for_frame.end()) {
          primary_cam = cam_in_precedence_list;
          break;
        }
      }

      feat_structs_for_feat.insert_or_assign(
          frame,
          VisionFeature(frame, pixel_by_camera_id_for_frame, primary_cam));
    }

    if (!has_obs) {
      continue;
    }

    feature_tracks_[feat_id] = StructuredVisionFeatureTrack(
        initial_est_for_feat,
        VisionFeatureTrack(feat_id, feat_structs_for_feat));
  }
  // TODO maybe warn if there are observations but no initial feature estimates
  loaded_ = true;
  return true;
}

bool OrbOutputLowLevelFeatureReader::readSingleFileFrameContentsFromDirectory(
    const std::string &directory_name,
    std::unordered_map<FrameId, FeatureObservationsForFrame>
        &single_frame_feature_observations) {
  for (const auto &entry : std::experimental::filesystem::directory_iterator(
           std::experimental::filesystem::path(directory_name))) {
    const auto file_extension = entry.path().extension().string();
    if (!std::experimental::filesystem::is_regular_file(entry) ||
        file_extension != ".txt") {
      continue;
    }
    std::ifstream data_file_stream(entry.path());
    if (data_file_stream.fail()) {
      LOG(FATAL) << "Failed to load: " << entry.path()
                 << " are you sure this a valid data file? ";
      return false;
    }

    // Start loading measurement files
    std::string line;

    // Read frame ID from 1st line
    // NOTE not the actual frame ID we want
    std::getline(data_file_stream, line);
    std::stringstream ss_id(line);
    FrameId frame_id;
    ss_id >> frame_id;
    // Read frame/robot pose from 2nd line
    // Skip it; we only want relative pose from "velocities" direcotry
    std::getline(data_file_stream, line);

    while (std::getline(data_file_stream, line)) {
      std::stringstream ss_feature(line);
      CameraId camera_id;
      FeatureId feature_id;
      float x, y;
      std::vector<float> xs, ys;
      std::vector<CameraId> camera_ids;
      // parse one line
      ss_feature >> feature_id;
      ss_feature >> camera_id;
      while (!ss_feature.eof()) {
        ss_feature >> x >> y;
        camera_ids.emplace_back(camera_id);
        xs.emplace_back(x);
        ys.emplace_back(y);
        ss_feature >> camera_id;
      }
      // load each line to measurements_by_camera
      if (single_frame_feature_observations.find(frame_id) ==
          single_frame_feature_observations.end()) {
        single_frame_feature_observations[frame_id] =
            FeatureObservationsForFrame();
      }
      for (size_t camera_idx = 0; camera_idx < camera_ids.size();
           camera_idx++) {
        camera_id = camera_ids[camera_idx];
        x = xs[camera_idx];
        y = ys[camera_idx];
        single_frame_feature_observations[frame_id].frame_id_ = frame_id;
        single_frame_feature_observations[frame_id]
            .features_[feature_id][camera_id] = Eigen::Vector2d(x, y);
      }
    }
  }
  return true;
}

bool OrbOutputLowLevelFeatureReader::readFeatureFileContentsFromDirectory(
    const std::string &directory_name,
    FeatureFileContents &feature_file_contents) {
  std::string features_file_path = directory_name;
  std::string last_char;
  last_char.push_back(features_file_path.back());
  if (last_char != "/") {
    features_file_path += "/";
  }
  features_file_path += kFeaturesFileLocation;

  // Load features
  std::ifstream feature_file_stream;
  feature_file_stream.open(features_file_path);
  if (feature_file_stream.fail()) {
    LOG(FATAL) << " Failed to open 3D feature file.";
    return false;
  }

  std::vector<file_io::FeatureEstWithId> raw_features;
  file_io::readFeatureEstsWithIdFromFile(features_file_path, raw_features);

  for (const file_io::FeatureEstWithId &raw_feat : raw_features) {
    feature_file_contents
        .feature_initial_position_estimates_[raw_feat.feature_id] =
        Eigen::Vector3d(raw_feat.x, raw_feat.y, raw_feat.z);
  }

  return true;
}

}  // namespace vslam_types_refactor