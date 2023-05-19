//
// Created by amanda on 1/27/23.
//

#include <base_lib/basic_utils.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/image_processing/debugging_image_utils.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace vtr = vslam_types_refactor;

DEFINE_string(jacobian_residual_info_file,
              "",
              "File containing jacobian parameter and residual info");
DEFINE_string(problem_feats_matlab_file,
              "",
              "File containing columns identified in matlab as very low value "
              "on diagonal of Hessian");
DEFINE_string(problem_ellipsoids_matlab_file,
              "",
              "File containing columns for ellipsoids identified in matlab as "
              "very low value "
              "on diagonal of Hessian");
DEFINE_string(images_out_dir, "", "Directory to which to output images");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(low_level_feats_dir,
              "",
              "Directory that contains low level features");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(visual_feature_results_file,
              "",
              "Name of file that contains feature estimates");

void drawFeatureInfoOnImage(
    const vtr::FrameId &frame_id,
    const vtr::CameraId &cam_id,
    const vtr::PixelCoord<double> &pixel_coord,
    const std::pair<double, double> &img_height_and_width,
    cv_bridge::CvImagePtr &img) {
  vtr::optionallyDisplayLeftCornerTextOnImage(
      std::to_string(frame_id), img_height_and_width, img);

  vtr::optionallyDisplayRightCornerTextOnImage(
      std::to_string(cam_id), img_height_and_width, img);

  std_msgs::ColorRGBA pixel_color;
  pixel_color.a = 1;
  pixel_color.r = 1;
  pixel_color.g = 0.5;

  vtr::drawTinyCircleOnImage(
      pixel_coord, vtr::brightenColor(pixel_color, 0.3), img);
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_images_out_dir.empty()) {
    LOG(INFO) << "Need images output directory";
    exit(1);
  }
  if (FLAGS_problem_feats_matlab_file.empty()) {
    LOG(INFO) << "Need file of problem features";
    exit(1);
  }
  if (FLAGS_jacobian_residual_info_file.empty()) {
    LOG(INFO) << "Need jacobian info file";
    exit(1);
  }
  if (FLAGS_rosbag_file.empty()) {
    LOG(INFO) << "Rosbag file was empty";
    exit(1);
  }
  if (FLAGS_low_level_feats_dir.empty()) {
    LOG(INFO) << "Low level features directory was empty";
    exit(1);
  }
  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(INFO) << "Nodes by timestamp file was empty";
    exit(1);
  }

  std::vector<vtr::GenericFactorInfo> generic_factor_infos;
  std::vector<vtr::ParameterBlockInfo> parameter_block_infos;

  vtr::SerializableVector<vtr::ParameterBlockInfo,
                          vtr::SerializableParameterBlockInfo>
      serializable_param_info;
  vtr::SerializableVector<vtr::GenericFactorInfo,
                          vtr::SerializableGenericFactorInfo>
      serializable_factors;

  LOG(INFO) << "Reading jacobian param/residual info from "
            << FLAGS_jacobian_residual_info_file;
  cv::FileStorage jacobian_info_fs(FLAGS_jacobian_residual_info_file,
                                   cv::FileStorage::READ);
  jacobian_info_fs["jacobian_param_block_info"] >> serializable_param_info;
  jacobian_info_fs["jacobian_residual_info"] >> serializable_factors;
  jacobian_info_fs.release();

  generic_factor_infos = serializable_factors.getEntry();
  parameter_block_infos = serializable_param_info.getEntry();

  std::vector<int> problem_columns;

  LOG(INFO) << "Reading problem features from "
            << FLAGS_problem_feats_matlab_file;
  std::ifstream file_obj(FLAGS_problem_feats_matlab_file);
  std::string line;
  while (std::getline(file_obj, line)) {
    int matlab_problem_col = std::stoi(line);
    problem_columns.emplace_back(matlab_problem_col -
                                 1);  // Because MATLAB is 1-indexed
  }

  // Assumes all problem entries are features and that features occur first in
  // the param info;
  std::unordered_set<vtr::FeatureId> problem_features;
  for (const int &problem_column : problem_columns) {
    int problem_feat_index = problem_column / 3;
    vtr::ParameterBlockInfo param_block =
        parameter_block_infos[problem_feat_index];
    if (!param_block.feature_id_.has_value()) {
      LOG(INFO) << "Problem column (MATLAB): " << problem_column + 1;
      LOG(INFO)
          << "No feature id for the param block with problem feature index "
          << problem_feat_index
          << ". Assumptions about param "
             "block ordering mustve been wrong";
      exit(1);
    }
    problem_features.insert(param_block.feature_id_.value());
  }

  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;
  std::unordered_map<
      vtr::FeatureId,
      std::unordered_map<
          vtr::FrameId,
          std::unordered_map<vtr::CameraId, vtr::PixelCoord<double>>>>
      low_level_features_map;
  vtr::LimitTrajectoryEvaluationParams limit_traj_params;
  limit_traj_params.should_limit_trajectory_evaluation_ = false;
  vtr::OrbOutputLowLevelFeatureReader orb_feat_reader(
      FLAGS_low_level_feats_dir, {}, limit_traj_params);
  vtr::FrameId max_frame_id = 0;
  orb_feat_reader.getLowLevelFeatures(visual_features);
  for (const auto &feature_track : visual_features) {
    vtr::FeatureId feat_id = feature_track.first;
    for (const auto &feat_obs_by_frame :
         feature_track.second.feature_track.feature_observations_) {
      vtr::FrameId frame_id = feat_obs_by_frame.first;
      //      if (frame_id > 25) {
      //        continue;
      //      }
      for (const auto &feat_obs_for_cam :
           feat_obs_by_frame.second.pixel_by_camera_id) {
        vtr::CameraId cam_id = feat_obs_for_cam.first;
        low_level_features_map[feat_id][frame_id][cam_id] =
            feat_obs_for_cam.second;
        max_frame_id = std::max(max_frame_id, frame_id);
      }
    }
  }

  std::unordered_map<std::string, vtr::CameraId> camera_topic_to_camera_id = {
      {"/zed/zed_node/left/image_rect_color/compressed", 1},
      {"/zed/zed_node/left/image_rect_color", 1},
      {"/zed/zed_node/right/image_rect_color/compressed", 2},
      {"/zed/zed_node/right/image_rect_color", 2},
      {"/zed2i/zed_node/left/image_rect_color/compressed", 1},
      {"/zed2i/zed_node/left/image_rect_color", 1},
      {"/zed2i/zed_node/right/image_rect_color/compressed", 2},
      {"/zed2i/zed_node/right/image_rect_color", 2}};

  std::unordered_map<vslam_types_refactor::FrameId,
                     std::unordered_map<vslam_types_refactor::CameraId,
                                        sensor_msgs::Image::ConstPtr>>
      images_for_rosbag =
          image_utils::getImagesFromRosbag(FLAGS_rosbag_file,
                                           FLAGS_nodes_by_timestamp_file,
                                           camera_topic_to_camera_id);

  for (const vtr::FeatureId &feat_id : problem_features) {
    std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId, vtr::PixelCoord<double>>>
        observations_for_feat = low_level_features_map[feat_id];
    LOG(INFO) << "Feat " << feat_id << " observed in "
              << low_level_features_map.size() << " frames";

    std::vector<cv::Mat> images_for_feat;
    for (vtr::FrameId frame = 0; frame <= max_frame_id; frame++) {
      if (observations_for_feat.find(frame) == observations_for_feat.end()) {
        continue;
      }
      for (const auto &cam_and_feat : observations_for_feat.at(frame)) {
        if (images_for_rosbag.find(frame) == images_for_rosbag.end()) {
          LOG(WARNING) << "Couldn't find image for frame " << frame;
          continue;
        }
        if (images_for_rosbag.at(frame).find(cam_and_feat.first) ==
            images_for_rosbag.at(frame).end()) {
          LOG(WARNING) << "Couldn't find image for frame " << frame
                       << " and cam " << cam_and_feat.first
                       << " even though there was a feature";
          continue;
        }
        cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::Image::ConstPtr image =
            images_for_rosbag.at(frame).at(cam_and_feat.first);
        try {
          cv_ptr =
              cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
          LOG(ERROR) << "cv_bridge exception: " << e.what();
          exit(1);
        }
        std::pair<double, double> img_height_and_width =
            std::make_pair(image->height, image->width);
        drawFeatureInfoOnImage(frame,
                               cam_and_feat.first,
                               cam_and_feat.second,
                               img_height_and_width,
                               cv_ptr);
        images_for_feat.emplace_back(cv_ptr->image);
      }
    }
    LOG(INFO) << "Writing images for feat " << feat_id << " to "
              << FLAGS_images_out_dir;
    if (!images_for_feat.empty()) {
      cv::Mat image_mosaic = vtr::generateMosaic(images_for_feat, 4);
      cv::imwrite(
          file_io::ensureDirectoryPathEndsWithSlash(FLAGS_images_out_dir) +
              "feat_" + std::to_string(feat_id) + vtr::kPngExtension,
          image_mosaic);
    } else {
      LOG(INFO) << "Images for feat was empty?";
    }
  }

  if (!FLAGS_visual_feature_results_file.empty()) {
    cv::FileStorage visual_feature_fs(FLAGS_visual_feature_results_file,
                                      cv::FileStorage::READ);
    LOG(INFO) << "Reading feature estimates from "
              << FLAGS_visual_feature_results_file;
    vtr::SerializableVisualFeatureResults ser_vis_feat_results;

    visual_feature_fs["visual_feats"] >> ser_vis_feat_results;
    visual_feature_fs.release();

    std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        visual_feature_positions =
            ser_vis_feat_results.getEntry().visual_feature_positions_;
    if (visual_feature_positions.empty()) {
      LOG(ERROR) << "Feature results was empty? ";
      exit(1);
    }

    std::vector<vtr::FeatureId> sorted_ids;
    for (const vtr::FeatureId &feat : problem_features) {
      sorted_ids.push_back(feat);
    }
    std::sort(sorted_ids.begin(), sorted_ids.end());

    for (const vtr::FeatureId &feat_id : sorted_ids) {
      if (visual_feature_positions.find(feat_id) ==
          visual_feature_positions.end()) {
        LOG(WARNING) << "No visual feature estimate found for " << feat_id;
      } else {
        vtr::Position3d<double> position = visual_feature_positions[feat_id];
        LOG(INFO) << "Feat " << feat_id << ": " << position.x() << ", "
                  << position.y() << ", " << position.z();
      }
    }
  }
}