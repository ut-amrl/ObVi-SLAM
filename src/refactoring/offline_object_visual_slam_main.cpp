#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <file_io/cv_file_storage/long_term_object_map_file_storage_io.h>
#include <file_io/cv_file_storage/output_problem_data_file_storage_io.h>
#include <file_io/cv_file_storage/roshan_bounding_box_front_end_file_storage_io.h>
#include <file_io/cv_file_storage/vslam_basic_types_file_storage_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/visual_feature_frontend/viusal_feature_front_end.h>
#include <refactoring/bounding_box_frontend/bounding_box_front_end_creation_utils.h>
#include <refactoring/bounding_box_frontend/bounding_box_retriever.h>
#include <refactoring/bounding_box_frontend/feature_based_bounding_box_front_end.h>
#include <refactoring/bounding_box_frontend/roshan_bounding_box_front_end.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <refactoring/long_term_map/long_term_map_factor_creator.h>
#include <refactoring/long_term_map/long_term_object_map_extraction.h>
#include <refactoring/offline/offline_problem_data.h>
#include <refactoring/offline/offline_problem_runner.h>
#include <refactoring/offline/pose_graph_frame_data_adder.h>
#include <refactoring/optimization/residual_creator.h>
#include <refactoring/output_problem_data.h>
#include <refactoring/output_problem_data_extraction.h>
#include <refactoring/visual_feature_processing/orb_output_low_level_feature_reader.h>
#include <refactoring/visualization/ros_visualization.h>
#include <refactoring/visualization/save_to_file_visualizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <filesystem>

#include <visualization/matplotlibcpp.h>
namespace plt = matplotlibcpp;

namespace vtr = vslam_types_refactor;

typedef vtr::IndependentEllipsoidsLongTermObjectMap<
    //    std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
    util::EmptyStruct>
    MainLtm;
typedef std::shared_ptr<MainLtm> MainLtmPtr;
typedef vtr::UnassociatedBoundingBoxOfflineProblemData<
    vtr::StructuredVisionFeatureTrack,
    sensor_msgs::Image::ConstPtr,
    MainLtm>
    MainProbData;
typedef vtr::ObjectAndReprojectionFeaturePoseGraph MainPg;
typedef std::shared_ptr<MainPg> MainPgPtr;
typedef std::pair<vslam_types_refactor::FactorType,
                  vslam_types_refactor::FeatureFactorId>
    MainFactorInfo;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(intrinsics_file, "", "File with camera intrinsics");
DEFINE_string(extrinsics_file, "", "File with camera extrinsics");
DEFINE_string(bounding_boxes_by_node_id_file,
              "",
              "File with bounding box observations by node id");
DEFINE_string(poses_by_node_id_file,
              "",
              "File with initial robot pose estimates");
DEFINE_string(nodes_by_timestamp_file,
              "",
              "File containing the timestamp-node mapping");
DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(
    long_term_map_input,
    "",
    "File name that stores the long-term map to load. If empty, will start "
    "from scratch.");
DEFINE_string(long_term_map_output,
              "",
              "File name to output the long-term map to.");
DEFINE_double(min_confidence, 0.2, "Minimum confidence");
DEFINE_string(low_level_feats_dir,
              "",
              "Directory that contains low level features");
DEFINE_string(vslam_debugger_directory, 
              "/robodata/taijing/object-slam/vslam/debug/1669743059/", 
              "Output root directory for debugger");
DEFINE_string(bb_associations_out_file,
              "",
              "File to write ellipsoid results and associated bounding boxes "
              "to. Skipped if this param is not set");
DEFINE_string(ltm_opt_jacobian_info_directory,
              "",
              "Directory to write jacobian info from the LTM optimization for");
DEFINE_string(visual_feature_results_file, "", "Visual feature results");
DEFINE_string(debug_images_output_directory,
              "",
              "Directory to output debug images to. If not specified, no debug "
              "images saved");

namespace fs = std::filesystem;

enum DebugTypeEnum {
  INITIALIZED,
  BEFORE_OPTIM,
  AFTER_OPTIM,
  ORBSLAM,
  OBSERVED,
  ALL,
  NONE
};

std::unordered_map<DebugTypeEnum, std::string> debugLabels = {
  {DebugTypeEnum::INITIALIZED, "initialization"},
  {DebugTypeEnum::BEFORE_OPTIM, "before optimization"},
  {DebugTypeEnum::AFTER_OPTIM,  "after optimization"},
  {DebugTypeEnum::ORBSLAM,    "from ORBSLAM"},
  {DebugTypeEnum::OBSERVED,   "observations"},
  {DebugTypeEnum::ALL,        "all"},
  {DebugTypeEnum::NONE,       ""}
};

void ToCSV(const std::string& filename, 
    const std::vector<vtr::Pose3D<double>>& trajectory,
    const std::string& delimiter = ",") {
  std::ofstream ofile;
  ofile.open(filename, std::ios::trunc);
  if (!ofile.is_open()) {
      LOG(ERROR) << "failed to open " << filename;
  }
  ofile << "x" << delimiter << "y" << delimiter << "z" << delimiter 
        << "qx" << delimiter << "qy" << delimiter 
        << "qz"  << delimiter << "qw" << std::endl;
  for (const auto& pose : trajectory) {
    ofile << pose.transl_.x() << delimiter 
          << pose.transl_.y() << delimiter
          << pose.transl_.z() << delimiter;
    Eigen::Quaterniond quat(pose.orientation_);
    ofile << quat.x() << delimiter 
          << quat.y() << delimiter
          << quat.z() << delimiter
          << quat.w() << std::endl;
  }
  ofile.close();
}

void ToCSV(const std::string& filename,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features,
    const std::string& delimiter = ",") {
  std::ofstream ofile;
  ofile.open(filename, std::ios::trunc);
  if (!ofile.is_open()) {
      LOG(ERROR) << "failed to open " << filename;
  }
  ofile << "feature_id" << delimiter << "x" << delimiter 
        << "y" << delimiter << "z" << std::endl;
  for (const auto& feature : features) {
    ofile << feature.first << delimiter
          << feature.second[0] << delimiter 
          << feature.second[1] << delimiter
          << feature.second[2] << std::endl;;
  }
  ofile.close();
}

template <typename datatype>
void LogToCSV(
    const std::unordered_map<DebugTypeEnum, fs::path>& save_directories,
    const std::unordered_map<DebugTypeEnum, datatype>& types_and_datas,
    const vtr::FrameId& frame_id) {
  for (const auto& type_and_data : types_and_datas) {
    const auto& type = type_and_data.first;
    const auto& data = type_and_data.second;
    fs::path savepath = save_directories.at(type) / (std::to_string(frame_id)+".csv");
    ToCSV(savepath, data);
  }
}

void PutImageText(const std::string& text, cv::Mat image) {
  cv::putText(image, 
              text, 
              cv::Point(10, 50), 
              cv::FONT_HERSHEY_SIMPLEX, 
              1.0, // font scale
              CV_RGB(255, 255, 255), 
              2.0); // line thickness
}

// assume each visualization has the same size
template <typename IdType>
cv::Mat SummarizeVisualization(
    const std::unordered_map<IdType, cv::Mat> &ids_and_visualizations, 
    const int &nimages_each_row = -1) {
  
  if (ids_and_visualizations.empty()) { 
    LOG(FATAL) << "Doesn't support this control path! "
               << "Please check ids_and_visualizations is non-empty before calling SummarizeVisualization()";
  }
  std::vector<IdType> ids;
  for (const auto& id_and_visualization : ids_and_visualizations) {
    ids.push_back(id_and_visualization.first);
  }
  std::sort(ids.begin(), ids.end());

  size_t summary_height, summary_width;
  const size_t& height = ids_and_visualizations.begin()->second.rows;
  const size_t& width = ids_and_visualizations.begin()->second.cols;
  const size_t& nvisualization = ids_and_visualizations.size();
  const size_t& nimages_per_row = (nimages_each_row == -1) ? nvisualization : (size_t) nimages_each_row;
  const size_t& nimages_per_col = (nvisualization % nimages_per_row == 0) ? 
      nvisualization / nimages_per_row : nvisualization / nimages_per_row + 1;
  summary_height = nimages_per_col * height;
  summary_width  = nimages_per_row * width;
  cv::Mat summary = cv::Mat::zeros(summary_height, summary_width, CV_8UC3);
  std::vector<std::pair<int, int>> coordinates;
  for (size_t i = 0; i < nvisualization; ++i) {
      coordinates.emplace_back( (i/nimages_per_row)*height, (i%nimages_per_row)*width );
  }

  for (size_t i = 0; i < ids.size(); ++i) {
    const auto& id = ids[i];
    const cv::Mat& visualization = ids_and_visualizations.at(id);
    const int& row = coordinates[i].first;
    const int& col = coordinates[i].second;
    visualization.copyTo(summary(cv::Range(row, row+height), cv::Range(col, col+width)));
  }
  return summary;
}

template <typename IdType>
cv::Mat SummarizeVisualization(
    const std::unordered_map<IdType, cv_bridge::CvImagePtr> &ids_and_visualizations, 
    const int &nimages_each_row = -1) {
  std::unordered_map<IdType, cv::Mat> new_ids_and_visualizations;
  for (const auto& id_and_visualization : ids_and_visualizations) {
    const IdType& id = id_and_visualization.first;
    const cv_bridge::CvImagePtr& imgptr = id_and_visualization.second;
    new_ids_and_visualizations[id] = imgptr->image;
  }
  cv::Mat ret = SummarizeVisualization(new_ids_and_visualizations, nimages_each_row);
  return ret;
}

void SetupOutputDirectory(const std::string& output_dir) {
  if (!fs::is_directory(output_dir) || !fs::exists(output_dir)) {
    if (!fs::create_directory(output_dir)) {
      LOG(FATAL) << "failed to create directory " << output_dir;
    }
  }
  for (const auto& entry : fs::directory_iterator(output_dir)) {
      fs::remove_all(entry.path());
  }
}

class VSLAMFrameDebugger {
public:
  VSLAMFrameDebugger() {}

  VSLAMFrameDebugger(const vtr::FrameId& frame_id,
      const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>& extrinsics,
      const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>& intrinsics,
      const std::unordered_map<vtr::CameraId, std::pair<double, double>>& img_heights_and_widths,
      const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>& images,
      const std::unordered_map<vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>& features2d,
      const std::unordered_map<vtr::CameraId, fs::path>& output_image_directories,
      const std::unordered_map<vtr::CameraId, fs::path>& output_residual_directories,
      const std::unordered_map<DebugTypeEnum, fs::path>& output_pcl_directories,
      const fs::path& output_pose_root_directory,
      const std::unordered_map<DebugTypeEnum, fs::path>& output_pose_directories,
      const fs::path& output_summary_directory,
      const std::unordered_map<vtr::CameraId, fs::path>& output_opt_window_directories_,
      const std::unordered_map<DebugTypeEnum, std_msgs::ColorRGBA>& deubg_types_and_ros_colors,
      const unsigned int& encoding) 
        : frame_id_(frame_id), 
          extrinsics_(extrinsics),
          intrinsics_(intrinsics),
          img_heights_and_widths_(img_heights_and_widths),
          images_(images),
          features2d_(features2d),
          output_image_directories_(output_image_directories),
          output_residual_directories_(output_residual_directories),
          output_pcl_directories_(output_pcl_directories),
          output_pose_root_directory_(output_pose_root_directory),
          output_pose_directories_(output_pose_directories),
          output_summary_directory_(output_summary_directory),
          output_opt_window_directories_(output_opt_window_directories_),
          deubg_types_and_ros_colors_(deubg_types_and_ros_colors),
          encoding_(encoding) {}
  
  bool updateFeatures3dForDebuggingOptWindow(
      const vtr::Pose3D<double>& pose,
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features3d,
      const DebugTypeEnum& debug_type,
      const size_t& local_BA_window) {
    
    std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr> cam_ids_and_visualizations;

    for (const auto& cam_id_and_feature2d : features2d_) {
      const vtr::CameraId& cam_id = cam_id_and_feature2d.first;
      const vtr::CameraExtrinsics<double>& extrinsics_for_cam = extrinsics_.at(cam_id);
      const vtr::CameraIntrinsicsMat<double>& intrinsics_for_cam = intrinsics_.at(cam_id);

      const sensor_msgs::Image::ConstPtr& img_for_cam = images_.at(cam_id);
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(img_for_cam, vtr::kImageEncoding);
      } catch (cv_bridge::Exception &e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
        exit(1);
      }

      for (const auto& feat_id_and_feature : cam_id_and_feature2d.second) {
        const auto& feat_id = feat_id_and_feature.first;
        const auto& observed_feat = feat_id_and_feature.second;
        if (features3d.find(feat_id) == features3d.end()) { continue; }
        vtr::Position3d<double> feature3d = features3d.at(feat_id);
        vtr::PixelCoord<double> projected_pixel = 
          vtr::getProjectedPixelCoord(feature3d,
                                      pose,
                                      extrinsics_for_cam,
                                      intrinsics_for_cam);

        vtr::RosVisualization::drawTinyCircleOnImage(
            observed_feat, deubg_types_and_ros_colors_[DebugTypeEnum::OBSERVED], cv_ptr);
        vtr::RosVisualization::drawTinyCircleOnImage(
            projected_pixel, deubg_types_and_ros_colors_[debug_type], cv_ptr);
        vtr::RosVisualization::drawLineOnImage(
            observed_feat, projected_pixel, deubg_types_and_ros_colors_[debug_type], cv_ptr);
      }
      cam_ids_and_visualizations[cam_id] = cv_ptr;
    }
    for (const auto& cam_id_and_visualization : cam_ids_and_visualizations) {
      const vtr::CameraId& cam_id = cam_id_and_visualization.first;
      const auto& visualization = cam_id_and_visualization.second;
      output_images_within_opt_window_[cam_id].push_back(visualization);
    }

    if (output_images_within_opt_window_.empty() 
            || output_images_within_opt_window_.begin()->second.size() < local_BA_window) { return false; }
    std::unordered_map<vtr::CameraId, cv::Mat> cam_ids_and_summaries;
    for (const auto& cam_id_and_visualizations : output_images_within_opt_window_) {
      const vtr::CameraId& cam_id = cam_id_and_visualizations.first;
      const auto& visualizations = cam_id_and_visualizations.second;
        std::unordered_map<size_t, cv_bridge::CvImagePtr> opt_ids_and_visualizations;
      for (size_t i = 0; i < visualizations.size(); ++i) {
        opt_ids_and_visualizations[i] = visualizations[i];
      }
      cam_ids_and_summaries[cam_id] = SummarizeVisualization(opt_ids_and_visualizations, 5);
    }
    for (const auto& cam_id_and_summary : cam_ids_and_summaries) {
      const auto& cam_id = cam_id_and_summary.first;
      const auto& summary = cam_id_and_summary.second;
      fs::path savepath 
          = output_opt_window_directories_[cam_id] / (std::to_string(frame_id_)+".png");
      cv::imwrite(savepath, summary);
    }
    return true;
  }

  void debug(const vtr::Pose3D<double>& pose,
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features3d,
      const std::vector<vtr::Pose3D<double>>& trajectory_vec,
      const DebugTypeEnum& debug_type) {
    getLowLevelFeaturesLatestImages(features3d, pose, debug_type);
    debugPoses(trajectory_vec, debug_type);
    debugFeaturePointcloud(features3d, debug_type);
  }

  void debugFeaturePointcloud(
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features3d,
      const DebugTypeEnum& debug_type) {
    std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> new_features3d;
    for (const auto& cam_id_and_observations : features2d_) {
      for (const auto& feat_id_and_feature : cam_id_and_observations.second) {
        const auto& feat_id = feat_id_and_feature.first;
        if (features3d.find(feat_id) != features3d.end()) {
          new_features3d[feat_id] = features3d.at(feat_id);
        }
      }
    }
    output_pointclouds_[debug_type] = new_features3d;
  }

  void debugPoses(
      const std::vector<vtr::Pose3D<double>>& trajectory_vec,
      const DebugTypeEnum& debug_type) {
    output_trajectories_[debug_type] = trajectory_vec;
  }

  void logOutputs() {
    if (!output_images_.empty()) {
      for (const auto& cam_id_and_output_images : output_images_) {
        const auto& cam_id = cam_id_and_output_images.first;
        const auto& output_images = cam_id_and_output_images.second;
        if ( output_images.empty() ) { return; }
        const int nimage_per_row = 2;
        cv::Mat summary = SummarizeVisualization(output_images, nimage_per_row);
        std::string image_path = 
            output_image_directories_.at(cam_id) / (std::to_string(frame_id_) + ".png");
        cv::imwrite(image_path, summary);
      }
    }

    if (!output_residuals_.empty()) {
      LogToCSV(output_pcl_directories_, output_pointclouds_, frame_id_);
    }

    if (!output_trajectories_.empty()) {
      LogToCSV(output_pose_directories_, output_trajectories_, frame_id_);
      plotTrajectory2D();
    }
    
    if (output_pointclouds_.empty()) {
      LogToCSV(output_pcl_directories_, output_pointclouds_, frame_id_);
    }
  }

  // TODO fix hardcoding
  void summarizeByFrameId() {
    std::unordered_map<vtr::CameraId, cv::Mat> cam_ids_and_visualizations;
    for (const auto& cam_id_and_output_directory : output_image_directories_) {
      const auto& cam_id = cam_id_and_output_directory.first;
      const auto& reprojection_path 
          = output_image_directories_.at(cam_id) / (std::to_string(frame_id_) + ".png");
      const auto& residual_path     
          = output_residual_directories_.at(cam_id) / (std::to_string(frame_id_) + ".png");
      const auto& trajectory_path   
          = output_pose_root_directory_ / (std::to_string(frame_id_) + ".png");
      cv::Mat reprojection = cv::imread(reprojection_path);
      cv::Mat residual = cv::imread(residual_path);
      cv::Mat trajectory = cv::imread(trajectory_path);
      if (reprojection.data == NULL || residual.data == NULL || trajectory.data == NULL) {
        LOG(FATAL) << "failed to read image data!";
      }
      if (reprojection.cols != 2 * residual.cols || residual.cols != trajectory.cols) {
        // TODO use more informative error message
        LOG(FATAL) << "Doesn't support these image sizes";
      }
      size_t summary_height, summary_width;
      summary_height = reprojection.rows + residual.rows;
      summary_width  = reprojection.cols;
      cv::Mat summary = cv::Mat::zeros(summary_height, summary_width, CV_8UC3);
      reprojection.copyTo(summary(cv::Range(0, reprojection.rows), cv::Range(0, reprojection.cols)));
      residual.copyTo(summary(cv::Range(reprojection.rows, summary_height), cv::Range(0, residual.cols)));
      trajectory.copyTo(summary(cv::Range(reprojection.rows, summary_height), cv::Range(residual.cols, summary_width)));
      cam_ids_and_visualizations[cam_id] = summary;
    }

    if (cam_ids_and_visualizations.empty()) { return; }
    cv::Mat summaries = SummarizeVisualization(cam_ids_and_visualizations);
    fs::path savepath = 
      output_summary_directory_ / (std::to_string(frame_id_) + ".png");
    cv::imwrite(savepath, summaries);
  }

  std::tuple<std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr>,
             std::unordered_map<vtr::CameraId, std::vector<double>>> 
  getLowLevelFeaturesLatestImages(
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& feat_ids_and_features3d,
      const vtr::Pose3D<double>& pose,
      const DebugTypeEnum& debug_type,
      const bool save = true) {
    std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr> ret_images;
    std::unordered_map<vtr::CameraId, std::vector<double>> ret_residuals;

    for (const auto& cam_id_and_observations : features2d_) {
      std::vector<double> residuals;
      const vtr::CameraId& cam_id = cam_id_and_observations.first;
      const vtr::CameraExtrinsics<double>& extrinsics_for_cam = extrinsics_.at(cam_id);
      const vtr::CameraIntrinsicsMat<double>& intrinsics_for_cam = intrinsics_.at(cam_id);
      std::optional<sensor_msgs::Image::ConstPtr> img_for_cam = std::nullopt;
      if (images_.find(cam_id) != images_.end()) {
        img_for_cam = images_.at(cam_id);
      }

      cv_bridge::CvImagePtr cv_ptr;
      if (img_for_cam.has_value()) {
        try {
          cv_ptr = cv_bridge::toCvCopy(img_for_cam.value(), vtr::kImageEncoding);
        } catch (cv_bridge::Exception &e) {
          LOG(ERROR) << "cv_bridge exception: " << e.what();
          exit(1);
        }
      }
      for (const auto &feat_id_and_feature2d : cam_id_and_observations.second) {
        const auto& feat_id = feat_id_and_feature2d.first;
        const auto& feature2d_obs = feat_id_and_feature2d.second;
        if (feat_ids_and_features3d.find(feat_id) ==
          feat_ids_and_features3d.end()) { continue; }
        vtr::PixelCoord<double> projected_pixels = 
        vtr::getProjectedPixelCoord(feat_ids_and_features3d.at(feat_id),
                                    pose,
                                    extrinsics_for_cam,
                                    intrinsics_for_cam);
        vtr::RosVisualization::drawTinyCircleOnImage(
            feature2d_obs, deubg_types_and_ros_colors_[DebugTypeEnum::OBSERVED], cv_ptr);
        vtr::RosVisualization::drawTinyCircleOnImage(
            projected_pixels, deubg_types_and_ros_colors_[debug_type], cv_ptr);
        vtr::RosVisualization::drawLineOnImage(
            feature2d_obs, projected_pixels, deubg_types_and_ros_colors_[debug_type], cv_ptr);
        double residual = getResidual(feature2d_obs, projected_pixels);
        output_residuals_[cam_id][debug_type].push_back(residual);
        ret_residuals[cam_id].push_back(residual);
      }
      PutImageText(debugLabels[debug_type] + " " + std::to_string(frame_id_), 
                   cv_ptr->image);
      if (save) {
        if (output_images_.find(cam_id) == output_images_.end()) {
          output_images_[cam_id] 
                = std::unordered_map<DebugTypeEnum, cv_bridge::CvImagePtr>();
        }
        output_images_[cam_id][debug_type] = cv_ptr;
      }
      ret_images[cam_id] = cv_ptr;
    }
    return std::make_tuple(ret_images, ret_residuals);
  }

  void getImages(std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr> &images) { 
    for (const auto &cam_id_and_img_msg : images_) {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(cam_id_and_img_msg.second, 
                                     vtr::kImageEncoding);
      } catch (cv_bridge::Exception &e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
        exit(1);
      }
      images[cam_id_and_img_msg.first] = cv_ptr;
    }
  }

  void getFeatures2d(std::unordered_map<vtr::CameraId,
      std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
          &features2d) { features2d = features2d_; }

private:
  // Debug at each frame
  vtr::FrameId frame_id_;
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>> extrinsics_;
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>> intrinsics_;
  std::unordered_map<vtr::CameraId, std::pair<double, double>> img_heights_and_widths_;
  std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr> images_;
  std::unordered_map<vtr::CameraId,
      std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>> features2d_;

  std::unordered_map<vtr::CameraId, 
      std::unordered_map<DebugTypeEnum, cv_bridge::CvImagePtr>> output_images_;
  std::unordered_map<vtr::CameraId, 
      std::unordered_map<DebugTypeEnum, std::vector<double>>> output_residuals_;
  std::unordered_map<DebugTypeEnum, 
      std::vector<vtr::Pose3D<double>>> output_trajectories_;
  std::unordered_map<DebugTypeEnum, 
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>> output_pointclouds_;

  fs::path root_directory_;
  std::unordered_map<vtr::CameraId, fs::path> output_image_directories_;
  std::unordered_map<vtr::CameraId, fs::path> output_residual_directories_;
  fs::path output_pose_root_directory_;
  std::unordered_map<DebugTypeEnum, fs::path> output_pcl_directories_;
  std::unordered_map<DebugTypeEnum, fs::path> output_pose_directories_;
  fs::path output_summary_directory_;
  std::unordered_map<vtr::CameraId, fs::path> output_opt_window_directories_;

  // Debug Optimization Window; Duplicate stored values for fast look-ups
  std::unordered_map<vtr::CameraId, std::vector<cv_bridge::CvImagePtr>> output_images_within_opt_window_;
  
  std::unordered_map<DebugTypeEnum, std_msgs::ColorRGBA> deubg_types_and_ros_colors_;
  unsigned int encoding_;

  double getResidual(const vtr::PixelCoord<double>& pixel1, 
      const vtr::PixelCoord<double>& pixel2) {
    return (pixel1-pixel2).norm();
  }

  void plotTrajectory2D() {
    fs::path savepath 
      = output_pose_root_directory_ / (std::to_string(frame_id_)+".png");
    std::unordered_map<DebugTypeEnum, std::vector<double>> types_and_xs;
    std::unordered_map<DebugTypeEnum, std::vector<double>> types_and_ys;
    for (const auto& type_and_traj : output_trajectories_) {
      const auto& type = type_and_traj.first;
      for (const auto& pose : type_and_traj.second) {
        types_and_xs[type].push_back(pose.transl_.x());
        types_and_ys[type].push_back(pose.transl_.y());
      }
    }
    plt::figure();
    // TODO right now only assume all images have the same size
    plt::figure_size(img_heights_and_widths_.at(1).second, img_heights_and_widths_.at(1).first);
    for (const auto& type_and_traj : output_trajectories_) {
      const auto& type = type_and_traj.first;
      const auto& xs = types_and_xs.at(type);
      const auto& ys = types_and_ys.at(type);
      plt::named_plot(debugLabels[type], xs, ys);
    }
    plt::legend();
    plt::title("Trajectory at Frame" + std::to_string(frame_id_));
    plt::save(savepath.string());
  }

  void plotResidualsHistogram() {
    long bins = 15;
    double alpha = 0.5;
    for (const auto& cam_id_and_output_directory : output_residual_directories_) {
      const auto& cam_id = cam_id_and_output_directory.first;
      const auto& output_residual_directory = cam_id_and_output_directory.second;
      fs::path savepath 
          = output_residual_directory / (std::to_string(frame_id_)+".png");
      const auto& output_residuals = output_residuals_.at(cam_id);
      plt::figure();
      plt::figure_size(img_heights_and_widths_.at(1).second, img_heights_and_widths_.at(1).first);
      // TODO fix this configurations;
      std::unordered_map<DebugTypeEnum, std::string> colors 
          = {{DebugTypeEnum::INITIALIZED, "y"},
             {DebugTypeEnum::BEFORE_OPTIM, "r"},
             {DebugTypeEnum::AFTER_OPTIM, "b"}};
      for (const auto& type_and_residuals : output_residuals) {
        const auto& type = type_and_residuals.first;
        plt::named_hist(debugLabels.at(type), output_residuals.at(type), bins, colors.at(type), alpha);
      }
      plt::legend();
      plt::title("Reprojection Residual Histogram at Frame " + std::to_string(frame_id_));
      plt::save(savepath.string());
    }
  }

};

class VSLAMDebugger {
public:
  VSLAMDebugger(const std::string& root_directory,
                const std::vector<vtr::CameraId>& cam_ids) :
                root_directory_(root_directory) {
    SetupOutputDirectory(root_directory_.string());
    
    fs::path output_image_directory = root_directory_ / "images";
    SetupOutputDirectory(output_image_directory.string());
    for (const auto& cam_id : cam_ids) {
      output_image_directories_[cam_id] = output_image_directory / std::to_string(cam_id);
      SetupOutputDirectory(output_image_directories_[cam_id]);
    }
    fs::path output_residual_directory = root_directory_ / "residuals";
    SetupOutputDirectory(output_residual_directory);
    for (const auto& cam_id : cam_ids) {
      output_residual_directories_[cam_id] = output_residual_directory / std::to_string(cam_id);
      SetupOutputDirectory(output_residual_directories_[cam_id]);
    }

    fs::path output_pcl_root_directory = root_directory_ / "pointclouds";
    SetupOutputDirectory(output_pcl_root_directory.string());
    output_pcl_directories_[DebugTypeEnum::INITIALIZED] = output_pcl_root_directory   / "init";
    output_pcl_directories_[DebugTypeEnum::BEFORE_OPTIM]  = output_pcl_root_directory / "before_optim";
    output_pcl_directories_[DebugTypeEnum::AFTER_OPTIM]  = output_pcl_root_directory / "after_optim";
    for (const auto& output_pcl_directory : output_pcl_directories_) {
      SetupOutputDirectory(output_pcl_directory.second.string());
    }

    output_pose_root_directory_ = root_directory_ / "poses";
    SetupOutputDirectory(output_pose_root_directory_.string());
    output_pose_directories_[DebugTypeEnum::INITIALIZED] = output_pose_root_directory_ / "init";
    output_pose_directories_[DebugTypeEnum::BEFORE_OPTIM]  = output_pose_root_directory_ / "before_optim";
    output_pose_directories_[DebugTypeEnum::AFTER_OPTIM]  = output_pose_root_directory_ / "after_optim";
    for (const auto& output_pose_directory : output_pose_directories_) {
      SetupOutputDirectory(output_pose_directory.second.string());
    }

    output_summary_directory_ = root_directory_ / "summaries";
    SetupOutputDirectory(output_summary_directory_);

    fs::path output_opt_window_root_directory = root_directory_ / "optimizations";
    SetupOutputDirectory(output_opt_window_root_directory.string());
    for (const auto& cam_id : cam_ids) {
      output_opt_window_directories_[cam_id] = output_opt_window_root_directory / std::to_string(cam_id);
      SetupOutputDirectory(output_opt_window_directories_[cam_id]);
    }

    fs::path output_feature_flow_root_directory = root_directory_ / "featureflow";
    SetupOutputDirectory(output_feature_flow_root_directory.string());
    for (const auto& cam_id : cam_ids) {
      output_feature_flow_directories_[cam_id] = output_feature_flow_root_directory / std::to_string(cam_id);
      SetupOutputDirectory(output_feature_flow_directories_[cam_id]);
    }

    fs::path output_imgdata_root_directory = root_directory_ / "imgdata";
    SetupOutputDirectory(output_imgdata_root_directory.string());
    for (const auto& cam_id : cam_ids) {
      output_imgdata_directories_[cam_id] = output_imgdata_root_directory / std::to_string(cam_id);
      SetupOutputDirectory(output_imgdata_directories_[cam_id]);
    }

    float alpha = .8;
    std_msgs::ColorRGBA obs_color;
    std_msgs::ColorRGBA init_color;
    std_msgs::ColorRGBA before_optim_color;
    std_msgs::ColorRGBA after_optim_color;
    obs_color.a = init_color.a = before_optim_color.a = after_optim_color.a = alpha;

    obs_color.g = 1;  // green
    init_color.r = 1; // yellow
    init_color.g = 1;
    before_optim_color.r = 1; // red
    after_optim_color.b = 1; // blue
    
    deubg_types_and_ros_colors_[DebugTypeEnum::OBSERVED] = obs_color;
    deubg_types_and_ros_colors_[DebugTypeEnum::INITIALIZED] = init_color;
    deubg_types_and_ros_colors_[DebugTypeEnum::BEFORE_OPTIM] = before_optim_color;
    deubg_types_and_ros_colors_[DebugTypeEnum::AFTER_OPTIM] = after_optim_color;

    if (vtr::kImageEncoding == sensor_msgs::image_encodings::MONO8) {
      encoding_ = CV_8UC1;
    } else if (vtr::kImageEncoding == sensor_msgs::image_encodings::BGR8) {
      encoding_ = CV_8UC3;
    } else {
      LOG(FATAL) << "doesn't support encoding " << encoding_;
      exit(1);
    }
  }

  void setupFrameDebuggerByFrameId(const vtr::FrameId& frame_id, 
      const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>& extrinsics,
      const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>& intrinsics,
      const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>& images,
      const std::unordered_map<vtr::CameraId, std::pair<double, double>>& img_heights_and_widths,
      const std::unordered_map<vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>& features2d) {

    frame_ids_and_debuggers_[frame_id] 
        = VSLAMFrameDebugger(frame_id, 
                             extrinsics,
                             intrinsics,
                             img_heights_and_widths,
                             images,
                             features2d,
                             output_image_directories_, 
                             output_residual_directories_,
                             output_pcl_directories_,
                             output_pose_root_directory_, 
                             output_pose_directories_, 
                             output_summary_directory_,
                             output_opt_window_directories_,
                             deubg_types_and_ros_colors_,
                             encoding_);
  }

  void debugOptimizationRunByFrameId(
      const vtr::FrameId& start_frame_id,
      const vtr::FrameId& end_frame_id,
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features3d,
      const std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>& poses,
      const DebugTypeEnum& debug_type) {
    
    const double kConsecutiveTranslChangeTol = 1.5;
    
    std::vector<vtr::FrameId> frame_ids;
    if (end_frame_id - start_frame_id <= 1) { return; }
    for (vtr::FrameId frame_id = start_frame_id+1; frame_id <= end_frame_id; ++frame_id) {
      if ((poses.at(frame_id).transl_-poses.at(frame_id-1).transl_).norm() 
                > kConsecutiveTranslChangeTol) {
        vtr::FrameId left_frame_id = 
            frame_id < start_frame_id + 2 ? start_frame_id : frame_id - 2;
        vtr::FrameId right_frame_id = 
            frame_id > end_frame_id - 2 ? end_frame_id : frame_id + 2;
        for (vtr::FrameId f_id = left_frame_id; f_id <= right_frame_id; ++f_id) {
          frame_ids.push_back(f_id);
        }
      }
    }
    
    // TODO Technically, we only need map instead of unordered_map
    std::map<vtr::CameraId, 
        std::unordered_map<vtr::FrameId, cv_bridge::CvImagePtr>> cam_ids_and_images;
    for (const vtr::FrameId &frame_id : frame_ids) {
      auto ret_images_and_residuals = frame_ids_and_debuggers_[frame_id]
          .getLowLevelFeaturesLatestImages(
              features3d, poses.at(frame_id), debug_type, false);
      std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr> cam_ids_and_visualizations 
          = std::get<0>(ret_images_and_residuals);
      std::unordered_map<vtr::CameraId, std::vector<double>> cam_ids_and_residuals
          = std::get<1>(ret_images_and_residuals);
      // TODO filter by residual
      for (const auto &cam_id_and_visualization : cam_ids_and_visualizations) {
        const vtr::CameraId &cam_id = cam_id_and_visualization.first;
        std::sort(cam_ids_and_residuals[cam_id].begin(), 
                  cam_ids_and_residuals[cam_id].end());
        if (cam_ids_and_images.find(cam_id) == cam_ids_and_images.end()) {
          cam_ids_and_images[cam_id] = std::unordered_map<vtr::FrameId, cv_bridge::CvImagePtr>();
        }
        cam_ids_and_images[cam_id][frame_id] = cam_id_and_visualization.second;
      }
    }
    for (const auto &cam_id_and_images : cam_ids_and_images) {
      const vtr::CameraId &cam_id = cam_id_and_images.first;
      cv::Mat summary = SummarizeVisualization(cam_id_and_images.second, 5);
      std::string optim_path = 
            output_opt_window_directories_.at(cam_id) / 
            (std::to_string(end_frame_id) + "_" + debugLabels.at(debug_type) + ".png");
      cv::imwrite(optim_path, summary);
    }
  }

  void debugFeatureFlowsByFrameId(const vtr::FrameId& end_frame_id,
                                  const vtr::FrameId& n_frame = 10) {
    vtr::FrameId start_frame_id 
        = end_frame_id < (n_frame+1) ? 1 : end_frame_id - n_frame;
    std::unordered_map<vtr::CameraId, cv_bridge::CvImagePtr> images;
    frame_ids_and_debuggers_[end_frame_id].getImages(images);
    for (const auto &cam_id_and_img : images) {
      fs::path savepath 
          = output_imgdata_directories_[cam_id_and_img.first] / (std::to_string(end_frame_id)+".png");
      cv::imwrite(savepath, cam_id_and_img.second->image);
    }
    std::unordered_map<vtr::CameraId,
          std::unordered_map<vtr::FeatureId, 
                             std::vector<vtr::PixelCoord<double>>>> features2d_vecs;
    for (vtr::FrameId frame_id = start_frame_id; frame_id <= end_frame_id; ++frame_id) {
      std::unordered_map<vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>> features2d;
      frame_ids_and_debuggers_[frame_id].getFeatures2d(features2d);
      for (const auto &feature2d : features2d) {
        const auto &cam_id = feature2d.first;
        for (const auto &feat_id_and_pixel : feature2d.second) {
          const auto &feat_id = feat_id_and_pixel.first;
          const auto &obs_pixel = feat_id_and_pixel.second;
          features2d_vecs[cam_id][feat_id].push_back(obs_pixel);
        }
      }
    }
    for (const auto &cam_id_and_img : images) {
      const auto &cam_id = cam_id_and_img.first;
      const auto &image = cam_id_and_img.second->image;
      for (const auto feat_id_and_flow : features2d_vecs.at(cam_id)) {
        const auto &flow = feat_id_and_flow.second;
        if (flow.size() <= 1) { continue; }
        for (size_t i = 1; i < flow.size(); ++i) {
          cv::line(image, 
                   cv::Point((int)flow[i-1][0], (int)flow[i-1][1]), 
                   cv::Point((int)flow[i][0],   (int)flow[i][1]), 
                   cv::Scalar(0, 255, 0),
                   1,
                   cv::LINE_AA);
        }
      }
      fs::path savepath = 
          output_feature_flow_directories_[cam_id] / (std::to_string(end_frame_id)+".png");
      cv::imwrite(savepath, image);
    }
  }

  void debugByFrameId(const vtr::FrameId& frame_id,
      const vtr::Pose3D<double>& pose,
      const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>& features3d,
      const std::vector<vtr::Pose3D<double>>& trajectory_vec,
      const DebugTypeEnum& debug_type) {
    if (frame_ids_and_debuggers_.find(frame_id) == frame_ids_and_debuggers_.end()) {
      LOG(INFO) << "Please call setupFrameDebuggerByFrameId before calling debugByFrameId";
      exit(1);
    }
    frame_ids_and_debuggers_[frame_id].debug(
        pose,
        features3d, 
        trajectory_vec,
        debug_type);
  }

  void logOutputsByFrameId(const vtr::FrameId& frame_id) {
    if (frame_ids_and_debuggers_.find(frame_id) == frame_ids_and_debuggers_.end()) {
      LOG(INFO) << "Please call setupFrameDebuggerByFrameId before calling debugByFrameId";
      exit(1);
    }
    frame_ids_and_debuggers_[frame_id].logOutputs();
  }

private:
  std::unordered_map<vtr::FrameId, VSLAMFrameDebugger> frame_ids_and_debuggers_;

  fs::path root_directory_;
  std::unordered_map<vtr::CameraId, 
      std::unordered_map<DebugTypeEnum, cv_bridge::CvImagePtr>> output_images_;
  std::unordered_map<vtr::CameraId, 
      std::unordered_map<DebugTypeEnum, std::vector<double>>> output_residuals_;
  std::vector<vtr::Pose3D<double>> init_trajectory_;
  std::vector<vtr::Pose3D<double>> est_trajectory_;

  std::unordered_map<vtr::CameraId, fs::path> output_image_directories_;
  std::unordered_map<vtr::CameraId, fs::path> output_residual_directories_;
  fs::path output_pose_root_directory_;
  std::unordered_map<DebugTypeEnum, fs::path> output_pcl_directories_;
  std::unordered_map<DebugTypeEnum, fs::path> output_pose_directories_;
  fs::path output_summary_directory_;
  std::unordered_map<vtr::CameraId, fs::path> output_opt_window_directories_;
  std::unordered_map<vtr::CameraId, fs::path> output_feature_flow_directories_;
  std::unordered_map<vtr::CameraId, fs::path> output_imgdata_directories_;

  std::unordered_map<DebugTypeEnum, std_msgs::ColorRGBA> deubg_types_and_ros_colors_;
  unsigned int encoding_;
};

// TODO read from user specified input
VSLAMDebugger debugger(FLAGS_vslam_debugger_directory, {1,2});

std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
readCameraIntrinsicsByCameraFromFile(const std::string &file_name) {
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_by_cam_id;
  file_io::readCameraIntrinsicsWithIdsFromFile(file_name,
                                               camera_intrinsics_by_cam_id);
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      cam_intrinsics_map;
  for (const file_io::CameraIntrinsicsWithId &intrinsics_for_cam :
       camera_intrinsics_by_cam_id) {
    vtr::CameraIntrinsicsMat<double> intrinsics_mat;
    intrinsics_mat << intrinsics_for_cam.mat_00, intrinsics_for_cam.mat_01,
        intrinsics_for_cam.mat_02, intrinsics_for_cam.mat_10,
        intrinsics_for_cam.mat_11, intrinsics_for_cam.mat_12,
        intrinsics_for_cam.mat_20, intrinsics_for_cam.mat_21,
        intrinsics_for_cam.mat_22;
    cam_intrinsics_map[intrinsics_for_cam.camera_id] = intrinsics_mat;
    // TODO do we need to include the width/height anywhere?
  }
  return cam_intrinsics_map;
}

std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
readCameraExtrinsicsByCameraFromFile(const std::string &file_name) {
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      extrinsics_map;

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  // TODO Verify that this is correct (and eventually just update the file to
  // have this)
  //  Eigen::Quaterniond extrinsics_orientation_switch_to_cam =
  //      Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)
  //          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_by_cam_id;
  file_io::readCameraExtrinsicsWithIdsFromFile(file_name,
                                               camera_extrinsics_by_cam_id);
  for (const file_io::CameraExtrinsicsWithId &extrinsics_for_cam :
       camera_extrinsics_by_cam_id) {
    vtr::Position3d<double> extrinsics_pos(extrinsics_for_cam.transl_x,
                                           extrinsics_for_cam.transl_y,
                                           extrinsics_for_cam.transl_z);

    //    vtr::Orientation3D<double> extrinsics_orient(
    //        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
    //                           extrinsics_for_cam.quat_x,
    //                           extrinsics_for_cam.quat_y,
    //                           extrinsics_for_cam.quat_z) *
    //        extrinsics_orientation_switch_to_cam);
    vtr::Orientation3D<double> extrinsics_orient(
        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
                           extrinsics_for_cam.quat_x,
                           extrinsics_for_cam.quat_y,
                           extrinsics_for_cam.quat_z));
    vtr::CameraExtrinsics<double> extrinsics_obj(extrinsics_pos,
                                                 extrinsics_orient);

    extrinsics_map[extrinsics_for_cam.camera_id] = extrinsics_obj;
  }
  return extrinsics_map;
}

std::unordered_map<
    vtr::FrameId,
    std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
readBoundingBoxesFromFile(const std::string &file_name) {
  std::vector<file_io::BoundingBoxWithNodeId> bounding_boxes_by_node_id;
  file_io::readBoundingBoxesWithNodeIdFromFile(file_name,
                                               bounding_boxes_by_node_id);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bb_map;
  for (const file_io::BoundingBoxWithNodeId &raw_bb :
       bounding_boxes_by_node_id) {
    if (raw_bb.detection_confidence >= FLAGS_min_confidence) {
      vtr::RawBoundingBox bb;
      bb.pixel_corner_locations_ = std::make_pair(
          vtr::PixelCoord<double>(raw_bb.min_pixel_x, raw_bb.min_pixel_y),
          vtr::PixelCoord<double>(raw_bb.max_pixel_x, raw_bb.max_pixel_y));
      bb.semantic_class_ = raw_bb.semantic_class;
      bb.detection_confidence_ = raw_bb.detection_confidence;
      bb_map[raw_bb.node_id][raw_bb.camera_id].emplace_back(bb);
    }
  }
  return bb_map;
}

std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> readRobotPosesFromFile(
    const std::string &file_name) {
  std::vector<std::pair<uint64_t, pose::Pose3d>> robot_poses_by_node_id;
  file_io::readPose3dsAndNodeIdFromFile(file_name, robot_poses_by_node_id);
  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses_by_node_num;
  for (const std::pair<uint64, pose::Pose3d> &pose_3d_with_frame :
       robot_poses_by_node_id) {
    vtr::Pose3D<double> pose(
        pose_3d_with_frame.second.first,
        vtr::Orientation3D<double>(pose_3d_with_frame.second.second));
    robot_poses_by_node_num[pose_3d_with_frame.first] = pose;
  }
  return robot_poses_by_node_num;
}

std::vector<std::shared_ptr<ceres::IterationCallback>>
dummyCeresCallbackCreator(const MainProbData &input_problem_data,
                          const MainPgPtr &pose_graph,
                          const vtr::FrameId &min_frame_optimized,
                          const vtr::FrameId &max_frame_optimized) {
  // TODO replace with actual code later
  return {};
}

bool checkFactorRefresh(const MainFactorInfo &factor,
                        const MainPgPtr &,
                        const util::EmptyStruct &) {
  return false;
}

void createPoseGraph(
    const MainProbData &input_problem_data,
    const std::function<bool(
        util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>>
            &)> &long_term_map_factor_provider,
    MainPgPtr &pose_graph) {
  std::unordered_map<vtr::ObjectId,
                     std::pair<std::string, vtr::RawEllipsoid<double>>>
      ltm_objects;
  vtr::EllipsoidResults ellipsoids_in_map;
  if (input_problem_data.getLongTermObjectMap() != nullptr) {
    input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
        ellipsoids_in_map);
  }
  LOG(INFO) << "Ellipsoids size " << ellipsoids_in_map.ellipsoids_.size();
  for (const auto &ellipsoid_entry : ellipsoids_in_map.ellipsoids_) {
    ltm_objects[ellipsoid_entry.first] = std::make_pair(
        ellipsoid_entry.second.first,
        vtr::convertToRawEllipsoid(ellipsoid_entry.second.second));
  }
  LOG(INFO) << "Ltm objects size " << ltm_objects.size();
  LOG(INFO) << "Creating pose graph ";
  pose_graph =
      std::make_shared<MainPg>(input_problem_data.getObjDimMeanAndCovByClass(),
                               input_problem_data.getCameraExtrinsicsByCamera(),
                               input_problem_data.getCameraIntrinsicsByCamera(),
                               ltm_objects,
                               long_term_map_factor_provider);
}

void publishLowLevelFeaturesLatestImages(
    const std::shared_ptr<vtr::RosVisualization> &vis_manager,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &intrinsics,
    const std::unordered_map<vtr::CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>
        &images,
    const vtr::FrameId &latest_frame_num,
    const vtr::Pose3D<double> &pose_at_frame,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &feature_ests,
    const std::unordered_map<
        vtr::CameraId,
        std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
        &observed_feats_for_frame,
    const vtr::PlotType &plot_type) {
  for (const auto &obs_feat_and_cam : observed_feats_for_frame) {
    vtr::CameraId cam_id = obs_feat_and_cam.first;
    if (img_heights_and_widths.find(cam_id) == img_heights_and_widths.end()) {
      LOG(ERROR) << "Didn't have height and width for camera with observed "
                    "features (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    std::pair<double, double> img_height_and_width =
        img_heights_and_widths.at(cam_id);
    if (extrinsics.find(cam_id) == extrinsics.end()) {
      LOG(ERROR) << "Didn't have extrinsics for camera with observed features"
                    " (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    vtr::CameraExtrinsics<double> extrinsics_for_cam = extrinsics.at(cam_id);

    if (intrinsics.find(cam_id) == intrinsics.end()) {
      LOG(ERROR) << "Didn't have intrinsics for camera with observed features"
                    " (id="
                 << cam_id
                 << "). Skipping visualization of"
                    " observations";
      continue;
    }
    vtr::CameraIntrinsicsMat<double> intrinsics_for_cam = intrinsics.at(cam_id);

    std::optional<sensor_msgs::Image::ConstPtr> img_for_cam = std::nullopt;
    if (images.find(cam_id) != images.end()) {
      img_for_cam = images.at(cam_id);
    }

    std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>
        projected_pixels;
    for (const auto &feat_est : feature_ests) {
      if (obs_feat_and_cam.second.find(feat_est.first) ==
          obs_feat_and_cam.second.end()) {
        // Feature wasn't observed, don't want to visualize it
        continue;
      }

      projected_pixels[feat_est.first] =
          vtr::getProjectedPixelCoord(feat_est.second,
                                      pose_at_frame,
                                      extrinsics_for_cam,
                                      intrinsics_for_cam);
    }

    vis_manager->publishLatestImageWithReprojectionResiduals(
        latest_frame_num,
        cam_id,
        intrinsics_for_cam,
        plot_type,
        obs_feat_and_cam.second,
        projected_pixels,
        img_for_cam,
        img_height_and_width,
        true);
  }
}

void visualizationStub(
    const std::shared_ptr<vtr::RosVisualization> &vis_manager,
    const vtr::SaveToFileVisualizer &save_to_file_visualizer,
    const std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
        &extrinsics,
    const std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
        &intrinsics,
    const std::unordered_map<vtr::CameraId, std::pair<double, double>>
        &img_heights_and_widths,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
        &images,
    const std::shared_ptr<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId,
                           std::vector<std::pair<vtr::BbCornerPair<double>,
                                                 std::optional<double>>>>>>
        &all_observed_corner_locations_with_uncertainty,
    const std::shared_ptr<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<
                vtr::ObjectId,
                std::pair<vtr::BbCornerPair<double>, std::optional<double>>>>>>
        &observed_corner_locations,
    const std::shared_ptr<std::vector<std::unordered_map<
        vtr::FrameId,
        std::unordered_map<vtr::CameraId,
                           std::pair<vtr::BbCornerPair<double>, double>>>>>
        &bounding_boxes_for_pending_object,
    const std::shared_ptr<std::vector<
        std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
        &pending_objects,
    const std::unordered_map<
        vtr::FrameId,
        std::unordered_map<
            vtr::CameraId,
            std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>>
        &observed_features,
    const std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
        &initial_feat_positions,
    const MainProbData &input_problem_data,
    std::shared_ptr<
        vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
        &pose_graph,
    const vtr::FrameId &min_frame_optimized,
    const vtr::FrameId &max_frame_optimized,
    const vtr::VisualizationTypeEnum &visualization_stage,
    const double &near_edge_threshold,
    const size_t &pending_obj_min_obs_threshold) {
  switch (visualization_stage) {
    case vtr::BEFORE_ANY_OPTIMIZATION:
      vis_manager->publishTransformsForEachCamera(
          input_problem_data.getMaxFrameId(),
          input_problem_data.getRobotPoseEstimates(),
          input_problem_data.getCameraExtrinsicsByCamera());

      sleep(3);
      break;
    case vtr::BEFORE_EACH_OPTIMIZATION: {
      debugger.setupFrameDebuggerByFrameId(
          max_frame_optimized,
          extrinsics, 
          intrinsics, 
          images.at(max_frame_optimized), 
          img_heights_and_widths,
          observed_features.at(max_frame_optimized));

      std::unordered_map<vtr::FrameId, vtr::RawPose3d<double>>
          optimized_robot_pose_estimates;
      pose_graph->getRobotPoseEstimates(optimized_robot_pose_estimates);
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          optimized_trajectory;
      for (const auto &frame_raw_pose : optimized_robot_pose_estimates) {
        optimized_trajectory[frame_raw_pose.first] =
            vtr::convertToPose3D(frame_raw_pose.second);
      }
      const vtr::Pose3D<double>& pose = optimized_trajectory.at(max_frame_optimized);
      
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_before_optim;
      pose_graph->getVisualFeatureEstimates(feature_before_optim);

      std::vector<vtr::Pose3D<double>> before_optim_trajectory_vec;
      for (vtr::FrameId frame_id = 0; frame_id <= max_frame_optimized;
           frame_id++) {
        before_optim_trajectory_vec.emplace_back(optimized_trajectory.at(frame_id));
      }

      debugger.debugByFrameId(
          max_frame_optimized, 
          pose, 
          feature_before_optim, 
          before_optim_trajectory_vec,
          DebugTypeEnum::BEFORE_OPTIM);
      debugger.debugOptimizationRunByFrameId(
        min_frame_optimized,
        max_frame_optimized,
        feature_before_optim,
        optimized_trajectory,
        DebugTypeEnum::BEFORE_OPTIM);
      break;
    }
    case vtr::AFTER_EACH_OPTIMIZATION: {
      std::unordered_map<vtr::FrameId, vtr::RawPose3d<double>>
          optimized_robot_pose_estimates;
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          initial_robot_pose_estimates =
              input_problem_data.getRobotPoseEstimates();
      pose_graph->getRobotPoseEstimates(optimized_robot_pose_estimates);
      std::unordered_map<vtr::FrameId, vtr::Pose3D<double>>
          optimized_trajectory;
      for (const auto &frame_raw_pose : optimized_robot_pose_estimates) {
        optimized_trajectory[frame_raw_pose.first] =
            vtr::convertToPose3D(frame_raw_pose.second);
      }

      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::RawEllipsoid<double>>>
          object_estimates;
      pose_graph->getObjectEstimates(object_estimates);
      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::EllipsoidState<double>>>
          optimized_ellipsoid_estimates_with_classes;
      std::unordered_map<vtr::ObjectId, vtr::EllipsoidState<double>>
          optimized_ellipsoid_estimates;
      for (const auto &obj_and_raw_est : object_estimates) {
        vtr::EllipsoidState<double> ellipsoid_state =
            vtr::convertToEllipsoidState(obj_and_raw_est.second.second);
        optimized_ellipsoid_estimates_with_classes[obj_and_raw_est.first] =
            std::make_pair(obj_and_raw_est.second.first, ellipsoid_state);
        optimized_ellipsoid_estimates[obj_and_raw_est.first] = ellipsoid_state;
      }
      std_msgs::ColorRGBA optimized_ellipsoid_color;
      optimized_ellipsoid_color.a = 0.5;
      //      optimized_ellipsoid_color.r = 1.0;
      optimized_ellipsoid_color.g = 1;
      vis_manager->visualizeEllipsoids(
          optimized_ellipsoid_estimates_with_classes, vtr::ESTIMATED);
      std::unordered_map<vtr::ObjectId,
                         std::pair<std::string, vtr::EllipsoidState<double>>>
          ltm_ellipsoids;
      if (input_problem_data.getLongTermObjectMap() != nullptr) {
        vtr::EllipsoidResults ellipsoids_in_map;
        input_problem_data.getLongTermObjectMap()->getEllipsoidResults(
            ellipsoids_in_map);
        for (const auto &ltm_ellipsoid : ellipsoids_in_map.ellipsoids_) {
          ltm_ellipsoids[ltm_ellipsoid.first] = ltm_ellipsoid.second;
        }
      }
      vis_manager->visualizeEllipsoids(ltm_ellipsoids, vtr::INITIAL, false);

      std::vector<size_t> num_obs_per_pending_obj;
      for (const auto &pending_obj_obs : *bounding_boxes_for_pending_object) {
        size_t num_obs_for_pending_obj = 0;
        for (const auto &obs_for_frame : pending_obj_obs) {
          for (const auto &obs_for_cam : obs_for_frame.second) {
            num_obs_for_pending_obj++;
          }
        }
        num_obs_per_pending_obj.emplace_back(num_obs_for_pending_obj);
      }
      vis_manager->visualizePendingEllipsoids(pending_objects,
                                              num_obs_per_pending_obj,
                                              pending_obj_min_obs_threshold);

      vis_manager->visualizeCameraObservations(
          max_frame_optimized,
          initial_robot_pose_estimates,
          optimized_trajectory,
          std::nullopt,
          std::nullopt,  // TODO should we extract the initial ellipsoid
                         // estimates from the front end?
          optimized_ellipsoid_estimates,
          std::nullopt,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images,
          *observed_corner_locations,
          {},
          false,
          near_edge_threshold);

      vis_manager->publishDetectedBoundingBoxesWithUncertainty(
          max_frame_optimized,
          *all_observed_corner_locations_with_uncertainty,
          images,
          intrinsics,
          img_heights_and_widths,
          vtr::PlotType::ESTIMATED);

      std::vector<vtr::Pose3D<double>> est_trajectory_vec;
      std::vector<vtr::Pose3D<double>> init_trajectory_vec;
      for (vtr::FrameId frame_id = 0; frame_id <= max_frame_optimized;
           frame_id++) {
        est_trajectory_vec.emplace_back(optimized_trajectory.at(frame_id));
        init_trajectory_vec.emplace_back(
            initial_robot_pose_estimates.at(frame_id));
      }
      vis_manager->visualizeTrajectory(init_trajectory_vec,
                                       vtr::PlotType::INITIAL);
      vis_manager->visualizeTrajectory(est_trajectory_vec,
                                       vtr::PlotType::ESTIMATED);
      vis_manager->publishTfForLatestPose(est_trajectory_vec.back(),
                                          vtr::PlotType::ESTIMATED);

      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>> feature_ests;
      std::unordered_map<
          vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>
          observed_feats_for_frame = observed_features.at(max_frame_optimized);
      pose_graph->getVisualFeatureEstimates(feature_ests);
      for (const auto &feature_est : feature_ests) {
        vtr::Position3d<double> initial_pos =
            initial_feat_positions.at(feature_est.first);
      }
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
          curr_frame_initial_feature_ests;
      for (const auto &cam_and_feats : observed_feats_for_frame) {
        for (const auto &feats_for_cam : cam_and_feats.second) {
          if (initial_feat_positions.find(feats_for_cam.first) !=
              initial_feat_positions.end()) {
            curr_frame_initial_feature_ests[feats_for_cam.first] =
                initial_feat_positions.at(feats_for_cam.first);
          }
        }
      }
      vis_manager->visualizeFeatureEstimates(curr_frame_initial_feature_ests,
                                             vtr::PlotType::INITIAL);
      std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
          curr_frame_est_feature_ests;
      for (const auto &cam_and_feats : observed_feats_for_frame) {
        for (const auto &feats_for_cam : cam_and_feats.second) {
          if (feature_ests.find(feats_for_cam.first) != feature_ests.end()) {
            curr_frame_est_feature_ests[feats_for_cam.first] =
                feature_ests.at(feats_for_cam.first);
          }
        }
      }
      vis_manager->visualizeFeatureEstimates(curr_frame_est_feature_ests,
                                             vtr::PlotType::ESTIMATED);
      publishLowLevelFeaturesLatestImages(
          vis_manager,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images.at(max_frame_optimized),
          max_frame_optimized,
          optimized_trajectory.at(max_frame_optimized),
          feature_ests,
          observed_feats_for_frame,
          vtr::PlotType::ESTIMATED);

      publishLowLevelFeaturesLatestImages(
          vis_manager,
          extrinsics,
          intrinsics,
          img_heights_and_widths,
          images.at(max_frame_optimized),
          max_frame_optimized,
          input_problem_data.getRobotPoseEstimates().at(max_frame_optimized),
          initial_feat_positions,
          observed_feats_for_frame,
          vtr::PlotType::INITIAL);
      
      debugger.debugByFrameId(
          max_frame_optimized, 
          input_problem_data.getRobotPoseEstimates().at(max_frame_optimized),
          initial_feat_positions, 
          init_trajectory_vec,
          DebugTypeEnum::INITIALIZED);
      debugger.debugByFrameId(
          max_frame_optimized, 
          optimized_trajectory.at(max_frame_optimized), 
          feature_ests, 
          est_trajectory_vec,
          DebugTypeEnum::AFTER_OPTIM);
      debugger.logOutputsByFrameId(max_frame_optimized);
      debugger.debugOptimizationRunByFrameId(
        min_frame_optimized,
        max_frame_optimized,
        feature_ests,
        optimized_trajectory,
        DebugTypeEnum::AFTER_OPTIM);
      debugger.debugFeatureFlowsByFrameId(max_frame_optimized);

      save_to_file_visualizer.boundingBoxFrontEndVisualization(
          images,
          bounding_boxes_for_pending_object,
          extrinsics,
          intrinsics,
          all_observed_corner_locations_with_uncertainty,
          observed_corner_locations,
          observed_features,
          min_frame_optimized,
          max_frame_optimized);

      break;
    }
    case vtr::AFTER_ALL_OPTIMIZATION:
      break;
    default:
      break;
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal
  FLAGS_colorlogtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  if (FLAGS_extrinsics_file.empty()) {
    LOG(ERROR) << "No extrinsics file provided";
    exit(1);
  }

  if (FLAGS_intrinsics_file.empty()) {
    LOG(ERROR) << "No intrinsics file provided";
    exit(1);
  }

  if (FLAGS_poses_by_node_id_file.empty()) {
    LOG(ERROR) << "No robot poses file provided";
    exit(1);
  }

  if (FLAGS_long_term_map_output.empty()) {
    LOG(ERROR) << "No long-term map output file provided";
    exit(1);
  }

  if (FLAGS_rosbag_file.empty()) {
    LOG(WARNING) << "No rosbag file provided";
  }

  if (FLAGS_nodes_by_timestamp_file.empty()) {
    LOG(WARNING) << "No nodes by timestamp file";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "ellipsoid_estimator_real_data");
  ros::NodeHandle node_handle;

  // Hard-coded values -----------------------------------------------------
  std::unordered_map<std::string,
                     std::pair<vtr::ObjectDim<double>, vtr::ObjectDim<double>>>
      shape_mean_and_std_devs_by_semantic_class;
  Eigen::Vector3d chair_mean(0.62, 0.62, 0.975);
  Eigen::Vector3d chair_std_dev(0.05, 0.05, 0.05);
  std::string chair_class = "chair";
  shape_mean_and_std_devs_by_semantic_class[chair_class] =
      std::make_pair(chair_mean, chair_std_dev);

  Eigen::Vector3d cone_mean(0.29, 0.29, 0.48);
  Eigen::Vector3d cone_std_dev(0.001, 0.001, 0.01);
  std::string cone_class = "roadblock";
  shape_mean_and_std_devs_by_semantic_class[cone_class] =
      std::make_pair(cone_mean, cone_std_dev);

  Eigen::Vector3d tree_mean(.40, .40, 2);
  Eigen::Vector3d tree_cov(0.2, 0.2, 3);
  std::string tree_class = "treetrunk";
  shape_mean_and_std_devs_by_semantic_class[tree_class] =
      std::make_pair(tree_mean, tree_cov);

  Eigen::Vector3d lamppost_mean(.3, .3, 4);
  Eigen::Vector3d lamppost_cov(0.15, 0.15, 3);
  std::string lamppost_class = "lamppost";
  shape_mean_and_std_devs_by_semantic_class[lamppost_class] =
      std::make_pair(lamppost_mean, lamppost_cov);

  Eigen::Vector3d bench_mean(1, 2.5, 1.5);
  Eigen::Vector3d bench_cov(1.5, 2, 1.5);
  std::string bench_class = "bench";
  shape_mean_and_std_devs_by_semantic_class[bench_class] =
      std::make_pair(bench_mean, bench_cov);

  Eigen::Vector3d trashcan_mean(1, 1, 1.5);
  Eigen::Vector3d trashcan_cov(1, 1, 1.5);
  std::string trashcan_class = "trashcan";
  shape_mean_and_std_devs_by_semantic_class[trashcan_class] =
      std::make_pair(trashcan_mean, trashcan_cov);

  pose_graph_optimization::OptimizationSolverParams solver_params;  // TODO
  solver_params.max_num_iterations_ = 500; // Tuning
  pose_graph_optimization::ObjectVisualPoseGraphResidualParams
      residual_params;
  residual_params.visual_residual_params_.reprojection_error_huber_loss_param_ = .1; // Tuning

  vtr::RoshanBbAssociationParams roshan_associator_params;  // TODO tune these
  roshan_associator_params.saturation_histogram_bins_ = 50;
  roshan_associator_params.hue_histogram_bins_ = 60;
  //  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 2.0; //
  //  inside
  roshan_associator_params.max_distance_for_associated_ellipsoids_ = 3.5;
  //  roshan_associator_params.min_observations_ = 40;
  //  roshan_associator_params.min_observations_ = 10;
  roshan_associator_params.min_observations_ = 40;
  roshan_associator_params.discard_candidate_after_num_frames_ = 40;
  roshan_associator_params.min_bb_confidence_ = 0.3;
  roshan_associator_params.required_min_conf_for_initialization = 0.5;

  Eigen::Vector4d bounding_box_std_devs;  // TODO maybe use different values
  bounding_box_std_devs(0) = 30;
  bounding_box_std_devs(1) = 30;
  bounding_box_std_devs(2) = 30;
  bounding_box_std_devs(3) = 30;
  vtr::Covariance<double, 4> bounding_box_covariance =
      vtr::createDiagCovFromStdDevs(bounding_box_std_devs);
  double near_edge_threshold = 20;
  double image_boundary_variance = pow(200.0, 2.0);  // TODO?

  // TODO read this from file
  std::unordered_map<std::string, vtr::CameraId> camera_topic_to_camera_id = {
      {"/zed/zed_node/left/image_rect_color/compressed", 1},
      {"/zed/zed_node/left/image_rect_color", 1},
      {"/zed/zed_node/right/image_rect_color/compressed", 2},
      {"/zed/zed_node/right/image_rect_color", 2}};

  // Post-processing of the hard-coded values ---------------------------
  std::unordered_map<
      std::string,
      std::pair<vtr::ObjectDim<double>, vtr::Covariance<double, 3>>>
      mean_and_cov_by_semantic_class;
  for (const auto &shape_mean_and_std_dev_for_class :
       shape_mean_and_std_devs_by_semantic_class) {
    mean_and_cov_by_semantic_class[shape_mean_and_std_dev_for_class.first] =
        std::make_pair(shape_mean_and_std_dev_for_class.second.first,
                       vtr::createDiagCovFromStdDevs(
                           shape_mean_and_std_dev_for_class.second.second));
  }

  // Read necessary data in from file -----------------------------------------
  std::unordered_map<vtr::CameraId, vtr::CameraIntrinsicsMat<double>>
      camera_intrinsics_by_camera =
          readCameraIntrinsicsByCameraFromFile(FLAGS_intrinsics_file);
  std::unordered_map<vtr::CameraId, vtr::CameraExtrinsics<double>>
      camera_extrinsics_by_camera =
          readCameraExtrinsicsByCameraFromFile(FLAGS_extrinsics_file);
  std::unordered_map<vtr::FeatureId, vtr::StructuredVisionFeatureTrack>
      visual_features;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      init_bounding_boxes;
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>>
      bounding_boxes;
  for (const auto &frame_id_bb_entry : init_bounding_boxes) {
    for (const auto &camera_bb_entry : frame_id_bb_entry.second) {
      for (const vtr::RawBoundingBox &bb : camera_bb_entry.second) {
        if ((bb.pixel_corner_locations_.second.x() < 1280) &&
            (bb.pixel_corner_locations_.second.y() < 720)) {
          bounding_boxes[frame_id_bb_entry.first][camera_bb_entry.first]
              .emplace_back(bb);
        }
      }
    }
  }
  std::shared_ptr<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::vector<std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>>
      all_observed_corner_locations_with_uncertainty =
          std::make_shared<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::vector<std::pair<vtr::BbCornerPair<double>,
                                        std::optional<double>>>>>>();
  for (const auto &bounding_boxes_for_frame : bounding_boxes) {
    for (const auto &bounding_boxes_for_frame_and_cam :
         bounding_boxes_for_frame.second) {
      std::vector<std::pair<vtr::BbCornerPair<double>, std::optional<double>>>
          observed_corners_for_frame_and_cam;
      for (const vtr::RawBoundingBox &bb :
           bounding_boxes_for_frame_and_cam.second) {
        observed_corners_for_frame_and_cam.emplace_back(std::make_pair(
            bb.pixel_corner_locations_, bb.detection_confidence_));
      }
      (*all_observed_corner_locations_with_uncertainty)
          [bounding_boxes_for_frame.first]
          [bounding_boxes_for_frame_and_cam.first] =
              observed_corners_for_frame_and_cam;
    }
  }

  std::unordered_map<vtr::FrameId, vtr::Pose3D<double>> robot_poses =
      readRobotPosesFromFile(FLAGS_poses_by_node_id_file);
  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId, sensor_msgs::Image::ConstPtr>>
      images = image_utils::getImagesFromRosbag(FLAGS_rosbag_file,
                                                FLAGS_nodes_by_timestamp_file,
                                                camera_topic_to_camera_id);

  std::unordered_map<vtr::CameraId, std::pair<double, double>>
      img_heights_and_widths;
  for (const auto &frame_and_imgs : images) {
    for (const auto &cam_and_img : frame_and_imgs.second) {
      if (img_heights_and_widths.find(cam_and_img.first) ==
          img_heights_and_widths.end()) {
        sensor_msgs::Image::ConstPtr img = cam_and_img.second;
        img_heights_and_widths[cam_and_img.first] =
            std::make_pair(img->height, img->width);
      }
    }
  }

  MainLtmPtr long_term_map;
  if (!FLAGS_long_term_map_input.empty()) {
    cv::FileStorage ltm_in_fs(FLAGS_long_term_map_input, cv::FileStorage::READ);
    vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
        //        std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>,
        //        vtr::SerializableMap<vtr::ObjectId,
        //                             vtr::SerializableObjectId,
        //                             vtr::RoshanAggregateBbInfo,
        //                             vtr::SerializableRoshanAggregateBbInfo>>
        util::EmptyStruct,
        vtr::SerializableEmptyStruct>
        serializable_ltm;
    ltm_in_fs["long_term_map"] >> serializable_ltm;
    ltm_in_fs.release();
    MainLtm ltm_from_serializable = serializable_ltm.getEntry();
    vtr::EllipsoidResults ellipsoid_results_ltm_v3;
    ltm_from_serializable.getEllipsoidResults(ellipsoid_results_ltm_v3);
    LOG(INFO) << "Second check results size "
              << ellipsoid_results_ltm_v3.ellipsoids_.size();
    long_term_map = std::make_shared<MainLtm>(ltm_from_serializable);
  }

  std::unordered_map<
      vtr::FrameId,
      std::unordered_map<
          vtr::CameraId,
          std::unordered_map<vtr::FeatureId, vtr::PixelCoord<double>>>>
      low_level_features_map;
  std::unordered_map<vtr::FeatureId, vtr::Position3d<double>>
      initial_feat_positions;

  if (!FLAGS_low_level_feats_dir.empty()) {
    LOG(INFO) << "Reading low level features";
    vtr::OrbOutputLowLevelFeatureReader orb_feat_reader(
        FLAGS_low_level_feats_dir, {});
    orb_feat_reader.getLowLevelFeatures(visual_features);
    for (const auto &feature_track : visual_features) {
      vtr::FeatureId feat_id = feature_track.first;
      initial_feat_positions[feat_id] = feature_track.second.feature_pos_;
      for (const auto &feat_obs_by_frame :
           feature_track.second.feature_track.feature_observations_) {
        vtr::FrameId frame_id = feat_obs_by_frame.first;
        for (const auto &feat_obs_for_cam :
             feat_obs_by_frame.second.pixel_by_camera_id) {
          vtr::CameraId cam_id = feat_obs_for_cam.first;
          low_level_features_map[frame_id][cam_id][feat_id] =
              feat_obs_for_cam.second;
        }
      }
    }
  }

  MainProbData input_problem_data(camera_intrinsics_by_camera,
                                  camera_extrinsics_by_camera,
                                  visual_features,
                                  robot_poses,
                                  mean_and_cov_by_semantic_class,
                                  bounding_boxes,
                                  long_term_map,
                                  images);

  // Configurable params ------------------------------------------------------
  vtr::FeatureBasedBbAssociationParams association_params;  // TODO populate
  association_params.discard_candidate_after_num_frames_ = 40;
  association_params.feature_validity_window_ = 20;

  vtr::SaveToFileVisualizerConfig save_to_file_visualizer_config;
  save_to_file_visualizer_config.bb_assoc_visualizer_config_
      .bounding_box_inflation_size_ =
      association_params.bounding_box_inflation_size_;
  save_to_file_visualizer_config.bb_assoc_visualizer_config_
      .feature_validity_window_ = association_params.feature_validity_window_;

  // Connect up functions needed for the optimizer --------------------------
  std::shared_ptr<vtr::RosVisualization> vis_manager =
      std::make_shared<vtr::RosVisualization>(node_handle);
  vtr::SaveToFileVisualizer save_to_file_visualizer(
      FLAGS_debug_images_output_directory, save_to_file_visualizer_config);
  //    vtr::RawEllipsoid<double> ellipsoid;
  //    ellipsoid << -0.164291, 0.41215, -0.0594742, 79.9495, 209.015, 248.223,
  //        0.432929, 0.450756, 2.05777;
  //    vtr::RawPose3d<double> robot_pose;
  //    robot_pose << 0.135177, -0.000860353, 0.0109102, 0.00145096,
  //    -0.000676748,
  //        -0.00533544;
  //    vtr::EllipsoidState<double> publishable_ellipsoid =
  //        vtr::convertToEllipsoidState(ellipsoid);
  //    vtr::Pose3D<double> publishable_robot_pose =
  //    vtr::convertToPose3D(robot_pose);
  //    vis_manager->visualizeEllipsoids({{1,std::make_pair("chair",
  //    publishable_ellipsoid)}},
  //                                     vtr::PlotType::INITIAL, false);
  //    for (const auto &extrinsics_entry : camera_extrinsics_by_camera) {
  //      LOG(INFO) << "Publishing transforms for camera " <<
  //      extrinsics_entry.first; vis_manager->publishTransformsForEachCamera(
  //          0, {{0, publishable_robot_pose}}, camera_extrinsics_by_camera,
  //          "init_");
  //      LOG(INFO) << "Publishing empty image for camera " <<
  //      extrinsics_entry.first;
  //      vis_manager->publishLatestImageWithReprojectionResiduals(
  //          0,
  //          extrinsics_entry.first,
  //          camera_intrinsics_by_camera.at(extrinsics_entry.first),
  //          vtr::PlotType::INITIAL,
  //          {},
  //          {},
  //          std::nullopt,
  //          img_heights_and_widths.at(extrinsics_entry.first),
  //          true);
  //      vis_manager->visualizeFrustum(publishable_robot_pose,
  //                            camera_intrinsics_by_camera.at(extrinsics_entry.first),
  //                            extrinsics_entry.second,
  //                            img_heights_and_widths.at(extrinsics_entry.first),
  //                            vtr::PlotType::INITIAL);
  //      LOG(INFO) << "Done with camera " << extrinsics_entry.first;
  //    }
  //    ros::Duration(2).sleep();
  //    exit(1);

  vtr::IndependentEllipsoidsLongTermObjectMapFactorCreator<
      util::EmptyStruct,
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_factor_creator(long_term_map);

  std::function<bool()> continue_opt_checker = []() { return ros::ok(); };

  // Tuning
  std::function<vtr::FrameId(const vtr::FrameId &)> window_provider_func =
      [](const vtr::FrameId &max_frame) -> vtr::FrameId {
    // For now, we'll just optimize the whole trajectory (so return 0 so we
    // start the optimization with node 0
//    return 0;
        if ((max_frame % 30) == 0) {
          return 0;
        }
        if (max_frame < 50) {
          return 0;
        }
        return max_frame - 50;
  };

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, const util::EmptyStruct &)>
      refresh_residual_checker = checkFactorRefresh;

  std::function<bool(
      const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)>
      cached_info_creator = [](const MainFactorInfo &factor_info,
                               const MainPgPtr &pose_graph,
                               util::EmptyStruct &cached_info) {
        return true;  // TODO maybe fill in with real info some day
      };
  std::function<bool(
      const MainFactorInfo &,
      const MainPgPtr &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const std::function<bool(
          const MainFactorInfo &, const MainPgPtr &, util::EmptyStruct &)> &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      long_term_map_residual_creator_func =
          [&](const MainFactorInfo &factor_info,
              const MainPgPtr &pose_graph,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &residual_params,
              const std::function<bool(const MainFactorInfo &,
                                       const MainPgPtr &,
                                       util::EmptyStruct &)> &cached_inf_create,
              ceres::Problem *problem,
              ceres::ResidualBlockId &res_id,
              util::EmptyStruct &cached_inf) {
            return ltm_factor_creator.createResidual(factor_info,
                                                     pose_graph,
                                                     residual_params,
                                                     cached_inf_create,
                                                     problem,
                                                     res_id,
                                                     cached_inf);
          };
  std::function<bool(
      const MainFactorInfo &,
      const pose_graph_optimization::ObjectVisualPoseGraphResidualParams &,
      const MainPgPtr &,
      ceres::Problem *,
      ceres::ResidualBlockId &,
      util::EmptyStruct &)>
      residual_creator =
          [&](const MainFactorInfo &factor_id,
              const pose_graph_optimization::ObjectVisualPoseGraphResidualParams
                  &solver_residual_params,
              const MainPgPtr &pose_graph,
              ceres::Problem *problem,
              ceres::ResidualBlockId &residual_id,
              util::EmptyStruct &cached_info) {
            return vtr::createResidual(factor_id,
                                       pose_graph,
                                       solver_residual_params,
                                       cached_info_creator,
                                       long_term_map_residual_creator_func,
                                       problem,
                                       residual_id,
                                       cached_info);
          };

  std::function<bool(util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>> &)>
      long_term_map_factor_provider =
          [&](util::BoostHashMap<MainFactorInfo, std::unordered_set<vtr::ObjectId>> &factor_data) {
            return ltm_factor_creator.getFactorsToInclude(factor_data);
          };
  std::function<void(const MainProbData &, MainPgPtr &)> pose_graph_creator =
      std::bind(createPoseGraph,
                std::placeholders::_1,
                long_term_map_factor_provider,
                std::placeholders::_2);
  // Tuning
  std::function<double(const MainProbData &,
                       const MainPgPtr &,
                       const vtr::FrameId &,
                       const vtr::FeatureId &,
                       const vtr::CameraId &)>
      reprojection_error_provider = [](const MainProbData &input_problem,
                                       const MainPgPtr &pose_graph,
                                       const vtr::FrameId &frame_id,
                                       const vtr::FeatureId &feature_id,
                                       const vtr::CameraId &camera_id) {
        // TODO replace with thought out function once we add in the visual
        // part
        return 2;  // Probably need to do something more sophisticated here --
                   // ORB-SLAM has a more advanced thing that I haven't looked
                   // into
      };

  double min_visual_feature_parallax_pixel_requirement = 5.0;
  double min_visual_feature_parallax_robot_transl_requirement = 0.1;
  double min_visual_feature_parallax_robot_orient_requirement = 0.05;
  vtr::VisualFeatureFrontend visual_feature_fronted(
      reprojection_error_provider, 
      min_visual_feature_parallax_pixel_requirement, 
      min_visual_feature_parallax_robot_transl_requirement,
      min_visual_feature_parallax_robot_orient_requirement);
  std::function<void(const MainProbData &,
                       const MainPgPtr &,
                       const vtr::FrameId &,
                       const vtr::FrameId &)>
      visual_feature_frame_data_adder = [&](const MainProbData &input_problem,
                                            const MainPgPtr &pose_graph,
                                            const vtr::FrameId &min_frame_id,
                                            const vtr::FrameId &max_frame_id) {
        visual_feature_fronted.addVisualFeatureObservations(
            input_problem_data, pose_graph, min_frame_id, max_frame_id);
      };

  //  std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>
  //      long_term_map_front_end_data;
  //  if (long_term_map != nullptr) {
  //    long_term_map->getFrontEndObjMapData(long_term_map_front_end_data);
  //  }
  std::shared_ptr<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::unordered_map<vtr::ObjectId,
                                            std::pair<vtr::BbCornerPair<double>,
                                                      std::optional<double>>>>>>
      associated_observed_corner_locations =
          std::make_shared<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::unordered_map<vtr::ObjectId,
                                     std::pair<vtr::BbCornerPair<double>,
                                               std::optional<double>>>>>>();

  std::shared_ptr<std::vector<std::unordered_map<
      vtr::FrameId,
      std::unordered_map<vtr::CameraId,
                         std::pair<vtr::BbCornerPair<double>, double>>>>>
      bounding_boxes_for_pending_object =
          std::make_shared<std::vector<std::unordered_map<
              vtr::FrameId,
              std::unordered_map<
                  vtr::CameraId,
                  std::pair<vtr::BbCornerPair<double>, double>>>>>();
  std::shared_ptr<std::vector<
      std::pair<std::string, std::optional<vtr::EllipsoidState<double>>>>>
      pending_objects = std::make_shared<
          std::vector<std::pair<std::string,
                                std::optional<vtr::EllipsoidState<double>>>>>();

  vtr::BoundingBoxCovGenParams cov_gen_params;
  //    std::function<std::pair<bool,
  //    std::optional<sensor_msgs::Image::ConstPtr>>(
  //        const vtr::FrameId &, const vtr::CameraId &, const MainProbData &)>
  //        bb_context_retriever = [](const vtr::FrameId &frame_id,
  //                                  const vtr::CameraId &camera_id,
  //                                  const MainProbData &problem_data) {
  //      //            LOG(INFO) << "Getting image for frame " << frame_id
  //      //                      << " and camera " << camera_id;
  //      std::optional<sensor_msgs::Image::ConstPtr> image =
  //          problem_data.getImageForFrameAndCamera(frame_id, camera_id);
  //      return std::make_pair(image.has_value(), image);
  //    };
  //    vtr::RoshanBbFrontEndCreator<vtr::ReprojectionErrorFactor>
  //        roshan_associator_creator = vtr::generateRoshanBbCreator(
  //            associated_observed_corner_locations,
  //            all_observed_corner_locations_with_uncertainty,
  //            img_heights_and_widths,
  //            cov_gen_params,
  //            long_term_map_front_end_data);
  //    std::function<std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
  //        vtr::ReprojectionErrorFactor,
  //        vtr::RoshanAggregateBbInfo,
  //        std::optional<sensor_msgs::Image::ConstPtr>,
  //        vtr::RoshanImageSummaryInfo,
  //        vtr::RoshanBbInfo,
  //        std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>>(
  //        const MainPgPtr &, const MainProbData &)>
  //        bb_associator_retriever =
  //        [&](const MainPgPtr &pg, const MainProbData &input_prob) {
  //          return roshan_associator_creator.getDataAssociator(pg);
  //        };
  vtr::GeometricSimilarityScorerParams geometric_similiarity_scorer_params;
  geometric_similiarity_scorer_params.max_merge_distance_ = 4;
  std::function<std::pair<bool, vtr::FeatureBasedContextInfo>(
      const vtr::FrameId &, const vtr::CameraId &, const MainProbData &)>
      bb_context_retriever = [&](const vtr::FrameId &frame_id,
                                 const vtr::CameraId &camera_id,
                                 const MainProbData &problem_data) {
        vtr::FeatureBasedContextInfo context;
        bool success = false;
        if (low_level_features_map.find(frame_id) !=
            low_level_features_map.end()) {
          if (low_level_features_map.at(frame_id).find(camera_id) !=
              low_level_features_map.at(frame_id).end()) {
            context.observed_features_ =
                low_level_features_map.at(frame_id).at(camera_id);
            success = true;
          }
        }
        return std::make_pair(success, context);
      };
  vtr::FeatureBasedBoundingBoxFrontEndCreator<vtr::ReprojectionErrorFactor>
      feature_based_associator_creator = vtr::generateFeatureBasedBbCreator(
          associated_observed_corner_locations,
          all_observed_corner_locations_with_uncertainty,
          bounding_boxes_for_pending_object,
          pending_objects,
          img_heights_and_widths,
          cov_gen_params,
          geometric_similiarity_scorer_params,
          association_params);
  std::function<std::shared_ptr<vtr::AbstractBoundingBoxFrontEnd<
      vtr::ReprojectionErrorFactor,
      vtr::FeatureBasedFrontEndObjAssociationInfo,
      vtr::FeatureBasedContextInfo,
      vtr::FeatureBasedContextInfo,
      vtr::FeatureBasedSingleBbContextInfo,
      util::EmptyStruct>>(const MainPgPtr &, const MainProbData &)>
      bb_associator_retriever =
          [&](const MainPgPtr &pg, const MainProbData &input_prob) {
            return feature_based_associator_creator.getDataAssociator(pg);
          };

  vtr::YoloBoundingBoxQuerier bb_querier(node_handle);
  std::function<bool(
      const vtr::FrameId &,
      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>> &)>
      bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
                         std::unordered_map<vtr::CameraId,
                                            std::vector<vtr::RawBoundingBox>>
                             &bounding_boxes_by_cam) {
        return bb_querier.retrieveBoundingBoxes(
            frame_id_to_query_for, input_problem_data, bounding_boxes_by_cam);
      };
  //  std::function<bool(
  //      const vtr::FrameId &,
  //      std::unordered_map<vtr::CameraId, std::vector<vtr::RawBoundingBox>>
  //      &)> bb_retriever = [&](const vtr::FrameId &frame_id_to_query_for,
  //                         std::unordered_map<vtr::CameraId,
  //                                            std::vector<vtr::RawBoundingBox>>
  //                             &bounding_boxes_by_cam) {
  //        return vtr::retrievePrecomputedBoundingBoxes(
  //            frame_id_to_query_for, input_problem_data,
  //            bounding_boxes_by_cam);
  //      };
  std::function<void(
      const MainProbData &, const MainPgPtr &, const vtr::FrameId &, const vtr::FrameId &)>
      frame_data_adder = [&](const MainProbData &problem_data,
                             const MainPgPtr &pose_graph,
                             const vtr::FrameId &min_frame_id,
                             const vtr::FrameId &frame_to_add) {
        vtr::addFrameDataAssociatedBoundingBox(problem_data,
                                               pose_graph,
                                               min_frame_id,
                                               frame_to_add,
                                               reprojection_error_provider,
                                               visual_feature_frame_data_adder,
                                               bb_retriever,
                                               bb_associator_retriever,
                                               bb_context_retriever);
      };

  //  std::function<void(
  //      const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //          vtr::StructuredVisionFeatureTrack,
  //          sensor_msgs::Image::ConstPtr> &,
  //      const std::shared_ptr<const
  //      MainPg> &, ceres::Problem *,
  //      vtr::SpatialEstimateOnlyResults &)>
  //      output_data_extractor =
  //          [](const vtr::UnassociatedBoundingBoxOfflineProblemData<
  //                 vtr::StructuredVisionFeatureTrack,
  //                 sensor_msgs::Image::ConstPtr> &input_problem_data,
  //             const std::shared_ptr<
  //                 const MainPg>
  //                 &pose_graph,
  //             ceres::Problem *problem,
  //             vtr::SpatialEstimateOnlyResults &output_problem_data) {
  //            vtr::extractSpatialEstimateOnlyResults(pose_graph,
  //                                                   output_problem_data);
  //          };

  vtr::CovarianceExtractorParams ltm_covariance_params;

  // TODO maybe replace params with something that will yield more accurate
  // results
  std::function<bool(
      const vtr::FactorType &, const vtr::FeatureFactorId &, vtr::ObjectId &)>
      long_term_map_obj_retriever = [&](const vtr::FactorType &factor_type,
                                        const vtr::FeatureFactorId &factor_id,
                                        vtr::ObjectId &object_id) {
        std::unordered_set<vtr::ObjectId> obj_ids;
        if (!ltm_factor_creator.getObjectIdsForFactor(
                factor_type, factor_id, obj_ids)) {
          return false;
        }
        if (obj_ids.size() != 1) {
          LOG(ERROR) << "This only works with factors that have one object";
          exit(1);
        }
        object_id = *obj_ids.begin();
        return true;
      };

  vtr::IndependentEllipsoidsLongTermObjectMapExtractor<
      //      std::unordered_map<vtr::ObjectId, vtr::RoshanAggregateBbInfo>>
      util::EmptyStruct>
      ltm_extractor(ltm_covariance_params,
                    residual_creator,
                    long_term_map_obj_retriever,
                    residual_params,
                    solver_params);

  std::function<void(
      const MainProbData &,
      const MainPgPtr &,
      const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
      vtr::LongTermObjectMapAndResults<MainLtm> &)>
      output_data_extractor = [&](const MainProbData &input_problem_data,
                                  const MainPgPtr &pose_graph,
                                  const pose_graph_optimizer::
                                      OptimizationFactorsEnabledParams
                                          &optimization_factors_enabled_params,
                                  vtr::LongTermObjectMapAndResults<MainLtm>
                                      &output_problem_data) {
        //                    std::function<bool(
        //                        std::unordered_map<vtr::ObjectId,
        //                        vtr::RoshanAggregateBbInfo> &)>
        //                        front_end_map_data_extractor =
        //                            [&](std::unordered_map<vtr::ObjectId,
        //                                                   vtr::RoshanAggregateBbInfo>
        //                                    &front_end_data) {
        //                              roshan_associator_creator.getDataAssociator(pose_graph)
        //                                  ->getFrontEndObjMapData(front_end_data);
        //                              return true;
        //                            };
        std::function<bool(util::EmptyStruct &)> front_end_map_data_extractor =
            [&](util::EmptyStruct &front_end_data) {
              feature_based_associator_creator.getDataAssociator(pose_graph)
                  ->getFrontEndObjMapData(front_end_data);
              return true;
            };

        std::function<bool(
            const MainPgPtr &,
            const pose_graph_optimizer::OptimizationFactorsEnabledParams &,
            MainLtm &)>
            long_term_object_map_extractor =
                [&](const MainPgPtr &ltm_pose_graph,
                    const pose_graph_optimizer::OptimizationFactorsEnabledParams
                        &ltm_optimization_factors_enabled_params,
                    MainLtm &ltm_extractor_out) {
                  return ltm_extractor.extractLongTermObjectMap(
                      ltm_pose_graph,
                      ltm_optimization_factors_enabled_params,
                      front_end_map_data_extractor,
                      FLAGS_ltm_opt_jacobian_info_directory,
                      ltm_extractor_out);
                };
        vtr::extractLongTermObjectMapAndResults(
            pose_graph,
            optimization_factors_enabled_params,
            long_term_object_map_extractor,
            output_problem_data);
      };

  std::function<std::vector<std::shared_ptr<ceres::IterationCallback>>(
      const MainProbData &,
      const MainPgPtr &,
      const vtr::FrameId &,
      const vtr::FrameId &)>
      ceres_callback_creator = dummyCeresCallbackCreator;
  std::function<void(const MainProbData &,
                     const MainPgPtr &,
                     const vtr::FrameId &,
                     const vtr::FrameId &,
                     const vtr::VisualizationTypeEnum &)>
      visualization_callback = [&](const MainProbData &input_problem_data,
                                   const MainPgPtr &pose_graph,
                                   const vtr::FrameId &min_frame_id,
                                   const vtr::FrameId &max_frame_id,
                                   const vtr::VisualizationTypeEnum
                                       &visualization_type) {
        std::shared_ptr<
            vtr::ObjAndLowLevelFeaturePoseGraph<vtr::ReprojectionErrorFactor>>
            superclass_ptr = pose_graph;
        visualizationStub(vis_manager,
                          save_to_file_visualizer,
                          camera_extrinsics_by_camera,
                          camera_intrinsics_by_camera,
                          img_heights_and_widths,
                          images,
                          all_observed_corner_locations_with_uncertainty,
                          associated_observed_corner_locations,
                          bounding_boxes_for_pending_object,
                          pending_objects,
                          low_level_features_map,
                          initial_feat_positions,
                          input_problem_data,
                          superclass_ptr,
                          min_frame_id,
                          max_frame_id,
                          visualization_type,
                          cov_gen_params.near_edge_threshold_,
                          association_params.min_observations_);
      };
  vtr::OfflineProblemRunner<MainProbData,
                            vtr::ReprojectionErrorFactor,
                            vtr::LongTermObjectMapAndResults<MainLtm>,
                            util::EmptyStruct,
                            MainPg>
      offline_problem_runner(residual_params,
                             continue_opt_checker,
                             window_provider_func,
                             refresh_residual_checker,
                             residual_creator,
                             pose_graph_creator,
                             frame_data_adder,
                             output_data_extractor,
                             ceres_callback_creator,
                             visualization_callback,
                             solver_params);

  pose_graph_optimizer::OptimizationFactorsEnabledParams
      optimization_factors_enabled_params;
  optimization_factors_enabled_params.use_pom_ = false;
  optimization_factors_enabled_params.include_visual_factors_ = true;
//    optimization_factors_enabled_params.fix_poses_ = true;
  optimization_factors_enabled_params.fix_poses_ = false;
  optimization_factors_enabled_params.fix_visual_features_ = false;
  optimization_factors_enabled_params.fix_objects_ = false;
  optimization_factors_enabled_params.poses_prior_to_window_to_keep_constant_ = 1; // Tuning
  // TODO should we also optimize the poses?

  //  vtr::SpatialEstimateOnlyResults output_results;
  vtr::LongTermObjectMapAndResults<MainLtm> output_results;
  // if (!initialize_features3d(residual_params,
  //                            continue_opt_checker,
  //                            window_provider_func,
  //                            refresh_residual_checker,
  //                            residual_creator,
  //                            pose_graph_creator,
  //                            frame_data_adder,
  //                            output_data_extractor,
  //                            ceres_callback_creator,
  //                            visualization_callback,
  //                            solver_params, 
  //                            input_problem_data) ) {
  //   LOG(ERROR) << "Failed to initalize feature positions";
  //   exit(1);
  // }
  offline_problem_runner.runOptimization(
      input_problem_data, 
      optimization_factors_enabled_params, 
      output_results, 
      vtr::OptimTypeEnum::TWOPHASE_SLIDING_WINDOW);

  if (!FLAGS_visual_feature_results_file.empty()) {
    cv::FileStorage visual_feature_fs(FLAGS_visual_feature_results_file,
                                      cv::FileStorage::WRITE);

    visual_feature_fs << "visual_feats"
                      << vtr::SerializableVisualFeatureResults(
                             output_results.visual_feature_results_);
    visual_feature_fs.release();
    for (const auto &feat_and_pos :
         output_results.visual_feature_results_.visual_feature_positions_) {
      if (feat_and_pos.second.norm() > 10000) {
        LOG(WARNING) << "High norm for feature " << feat_and_pos.first << ": "
                     << feat_and_pos.second.norm();
      }
    }
  }

  cv::FileStorage ltm_out_fs(FLAGS_long_term_map_output,
                             cv::FileStorage::WRITE);
  ltm_out_fs
      << "long_term_map"
      << vtr::SerializableIndependentEllipsoidsLongTermObjectMap<
             //             std::unordered_map<vtr::ObjectId,
             //             vtr::RoshanAggregateBbInfo>,
             //             vtr::SerializableMap<vtr::ObjectId,
             //                                  vtr::SerializableObjectId,
             //                                  vtr::RoshanAggregateBbInfo,
             //                                  vtr::SerializableRoshanAggregateBbInfo>>(
             util::EmptyStruct,
             vtr::SerializableEmptyStruct>(output_results.long_term_map_);
  ltm_out_fs.release();
  //  LOG(INFO) << "Num ellipsoids "
  //            << output_results.ellipsoid_results_.ellipsoids_.size();

  if (!FLAGS_bb_associations_out_file.empty()) {
    cv::FileStorage bb_associations_out(FLAGS_bb_associations_out_file,
                                        cv::FileStorage::WRITE);
    vtr::ObjectDataAssociationResults data_assoc_results;
    data_assoc_results.ellipsoid_pose_results_ =
        output_results.ellipsoid_results_;
    data_assoc_results.associated_bounding_boxes_ =
        *associated_observed_corner_locations;
    bb_associations_out << "bounding_box_associations"
                        << vtr::SerializableObjectDataAssociationResults(
                               data_assoc_results);
    bb_associations_out.release();
  }

  // TODO save output results somewhere

  //  vtr::EllipsoidState<double> ellipsoid(
  //      vtr::Pose3D(Eigen::Vector3d(1, 2, 3),
  //                  Eigen::AngleAxisd(0.2, Eigen::Vector3d(0, 0, 1))),
  //      Eigen::Vector3d(4, 5, 6));
  //
  //  cv::FileStorage fs("test.json", cv::FileStorage::WRITE);
  //  fs << "ellipsoid" << vtr::SerializableEllipsoidState<double>(ellipsoid);
  //
  //  cv::FileStorage fs2("test.json", cv::FileStorage::READ);
  //  vtr::SerializableEllipsoidState<double> ellipsoid_state;
  //  fs2["ellipsoid"] >> ellipsoid_state;
  //  LOG(INFO) << "Read ellipsoid";
  //  LOG(INFO) << "Transl " << ellipsoid_state.getEntry().pose_.transl_;
  //  LOG(INFO) << "Orientation angle " <<
  //  ellipsoid_state.getEntry().pose_.orientation_.angle(); LOG(INFO) <<
  //  "Orientation axis" <<
  //  ellipsoid_state.getEntry().pose_.orientation_.axis(); LOG(INFO) <<
  //  "Dimensions " << ellipsoid_state.getEntry().dimensions_;

  return 0;
}