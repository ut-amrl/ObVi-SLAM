//
// Created by amanda on 8/23/22.
//

#ifndef UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H
#define UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H

#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>
#include <glog/logging.h>
#include <refactoring/output_problem_data.h>

#include <filesystem>

namespace vslam_types_refactor {
const std::string kRobotPosesKey = "robot_poses";
const std::string kEllipsoidsKey = "ellipsoids";

class SerializableEllipsoidResults
    : public FileStorageSerializable<EllipsoidResults> {
 public:
  SerializableEllipsoidResults()
      : FileStorageSerializable<EllipsoidResults>() {}
  SerializableEllipsoidResults(const EllipsoidResults &data)
      : FileStorageSerializable<EllipsoidResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kResultsMapLabel << "[";
    for (const auto &ellipsoid_entry : data_.ellipsoids_) {
      SerializableEllipsoidState<double> entry_state(
          ellipsoid_entry.second.second);
      fs << "{";
      fs << kObjectIdLabel << SerializableObjectId(ellipsoid_entry.first);
      fs << kSemanticClassLabel << ellipsoid_entry.second.first;
      fs << kEllipsoidStateLabel << entry_state;
      fs << "}";
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    cv::FileNode results_map_data = node[kResultsMapLabel];
    for (cv::FileNodeIterator it = results_map_data.begin();
         it != results_map_data.end();
         it++) {
      cv::FileNode entry = *it;
      SerializableObjectId obj_id;
      entry[kObjectIdLabel] >> obj_id;

      std::string semantic_class;
      entry[kSemanticClassLabel] >> semantic_class;

      SerializableEllipsoidState<double> ellipsoid_state;
      entry[kEllipsoidStateLabel] >> ellipsoid_state;

      data_.ellipsoids_[obj_id.getEntry()] =
          std::make_pair(semantic_class, ellipsoid_state.getEntry());
    }
  }

 protected:
  using FileStorageSerializable<EllipsoidResults>::data_;

 private:
  inline static const std::string kResultsMapLabel = "ellipsoid_results_map";
  inline static const std::string kObjectIdLabel = "object_id";
  inline static const std::string kSemanticClassLabel = "class";
  inline static const std::string kEllipsoidStateLabel = "state";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableEllipsoidResults &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableEllipsoidResults &data,
                 const SerializableEllipsoidResults &default_data =
                     SerializableEllipsoidResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableObjectDataAssociationResults
    : public FileStorageSerializable<ObjectDataAssociationResults> {
 public:
  SerializableObjectDataAssociationResults()
      : FileStorageSerializable<ObjectDataAssociationResults>() {}
  SerializableObjectDataAssociationResults(
      const ObjectDataAssociationResults &data)
      : FileStorageSerializable<ObjectDataAssociationResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kEllipsoidResultsLabel
       << SerializableEllipsoidResults(data_.ellipsoid_pose_results_);
    fs << kAssociatedBoundingBoxesLabel
       << SerializableBoundingBoxesByFrameEntry(
              data_.associated_bounding_boxes_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableEllipsoidResults serializable_ellipsoid_results;
    node[kEllipsoidResultsLabel] >> serializable_ellipsoid_results;
    SerializableBoundingBoxesByFrameEntry serializable_bb_results;
    node[kAssociatedBoundingBoxesLabel] >> serializable_bb_results;
    data_.ellipsoid_pose_results_ = serializable_ellipsoid_results.getEntry();
    data_.associated_bounding_boxes_ = serializable_bb_results.getEntry();
  }

 protected:
  using FileStorageSerializable<ObjectDataAssociationResults>::data_;

 private:
  inline static const std::string kEllipsoidResultsLabel = "ellipsoid_results";
  inline static const std::string kAssociatedBoundingBoxesLabel =
      "associated_bounding_boxes";

  using BoundingBoxEntry =
      std::pair<BbCornerPair<double>, std::optional<double>>;
  using SerializableBoundingBoxEntry =
      SerializablePair<BbCornerPair<double>,
                       SerializableBbCornerPair,
                       std::optional<double>,
                       SerializableOptional<double, SerializableDouble>>;
  using ObjectBoundingBoxMapEntry =
      std::unordered_map<ObjectId, BoundingBoxEntry>;
  using SerializableObjectBoundingBoxMapEntry =
      SerializableMap<ObjectId,
                      SerializableObjectId,
                      BoundingBoxEntry,
                      SerializableBoundingBoxEntry>;
  using BoundingBoxesByCameraEntry =
      std::unordered_map<CameraId, ObjectBoundingBoxMapEntry>;
  using SerializableBoundingBoxesByCameraEntry =
      SerializableMap<CameraId,
                      SerializableCameraId,
                      ObjectBoundingBoxMapEntry,
                      SerializableObjectBoundingBoxMapEntry>;
  using BoundingBoxesByFrameEntry =
      std::unordered_map<FrameId, BoundingBoxesByCameraEntry>;
  using SerializableBoundingBoxesByFrameEntry =
      SerializableMap<FrameId,
                      SerializableFrameId,
                      BoundingBoxesByCameraEntry,
                      SerializableBoundingBoxesByCameraEntry>;
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableObjectDataAssociationResults &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableObjectDataAssociationResults &data,
                 const SerializableObjectDataAssociationResults &default_data =
                     SerializableObjectDataAssociationResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableVisualFeatureResults
    : public FileStorageSerializable<VisualFeatureResults> {
 public:
  SerializableVisualFeatureResults()
      : FileStorageSerializable<VisualFeatureResults>() {}
  SerializableVisualFeatureResults(const VisualFeatureResults &data)
      : FileStorageSerializable<VisualFeatureResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kResultsMapLabel
       << SerializableMap<FeatureId,
                          SerializableFeatureId,
                          Position3d<double>,
                          SerializableEigenMat<double, 3, 1>>(
              data_.visual_feature_positions_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableMap<FeatureId,
                    SerializableFeatureId,
                    Position3d<double>,
                    SerializableEigenMat<double, 3, 1>>
        serializable_feat_results;
    node[kResultsMapLabel] >> serializable_feat_results;
    data_.visual_feature_positions_ = serializable_feat_results.getEntry();
  }

 protected:
  using FileStorageSerializable<VisualFeatureResults>::data_;

 private:
  inline static const std::string kResultsMapLabel =
      "visual_feature_results_map";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableVisualFeatureResults &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableVisualFeatureResults &data,
                 const SerializableVisualFeatureResults &default_data =
                     SerializableVisualFeatureResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableRobotPoseResults
    : public FileStorageSerializable<RobotPoseResults> {
 public:
  SerializableRobotPoseResults()
      : FileStorageSerializable<RobotPoseResults>() {}
  SerializableRobotPoseResults(const RobotPoseResults &data)
      : FileStorageSerializable<RobotPoseResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{" << kResultsMapLabel << "[";
    for (const auto &robot_pose_entry : data_.robot_poses_) {
      SerializablePose3D<double> ser_robot_pose(robot_pose_entry.second);
      fs << "{";
      fs << kFrameIdLabel << SerializableFrameId(robot_pose_entry.first);
      fs << kPoseLabel << ser_robot_pose;
      fs << "}";
    }
    fs << "]";
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    cv::FileNode results_map_data = node[kResultsMapLabel];
    for (cv::FileNodeIterator it = results_map_data.begin();
         it != results_map_data.end();
         it++) {
      cv::FileNode entry = *it;
      SerializableFrameId frame_id;
      entry[kFrameIdLabel] >> frame_id;

      SerializablePose3D<double> ser_robot_pose;
      entry[kPoseLabel] >> ser_robot_pose;

      data_.robot_poses_[frame_id.getEntry()] = ser_robot_pose.getEntry();
    }
  }

 protected:
  using FileStorageSerializable<RobotPoseResults>::data_;

 private:
  inline static const std::string kResultsMapLabel = "robot_pose_results_map";
  inline static const std::string kFrameIdLabel = "frame_id";
  inline static const std::string kPoseLabel = "pose";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableRobotPoseResults &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableRobotPoseResults &data,
                 const SerializableRobotPoseResults &default_data =
                     SerializableRobotPoseResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void writeRobotPoseResults(const std::string &robot_pose_file,
                           const RobotPoseResults &robot_pose_results) {
  cv::FileStorage robot_poses_results_out(robot_pose_file,
                                          cv::FileStorage::WRITE);
  robot_poses_results_out << kRobotPosesKey
                          << SerializableRobotPoseResults(robot_pose_results);
  robot_poses_results_out.release();
}

void readRobotPoseResults(const std::string &robot_pose_file,
                          RobotPoseResults &robot_pose_results) {
  if (!std::filesystem::exists(robot_pose_file)) {
    LOG(ERROR) << "Trying to read file " << robot_pose_file
               << " that does not exist";
    return;
  }
  cv::FileStorage robot_pose_in(robot_pose_file, cv::FileStorage::READ);
  SerializableRobotPoseResults serializable_robot_pose_results;
  robot_pose_in[kRobotPosesKey] >> serializable_robot_pose_results;
  robot_pose_in.release();
  robot_pose_results = serializable_robot_pose_results.getEntry();
}

void writeEllipsoidResults(const std::string &ellipsoid_results_file,
                           const EllipsoidResults &ellipsoid_results) {
  cv::FileStorage ellipsoids_results_out(ellipsoid_results_file,
                                         cv::FileStorage::WRITE);

  ellipsoids_results_out << kEllipsoidsKey
                         << SerializableEllipsoidResults(ellipsoid_results);
  ellipsoids_results_out.release();
}

void readEllipsoidResults(const std::string &ellipsoid_results_file,
                          EllipsoidResults &ellipsoid_results) {
  if (!std::filesystem::exists(ellipsoid_results_file)) {
    LOG(ERROR) << "Trying to read file " << ellipsoid_results_file
               << " that does not exist";
    return;
  }
  cv::FileStorage ellipsoids_in(ellipsoid_results_file, cv::FileStorage::READ);
  SerializableEllipsoidResults serializable_ellipsoid_results;
  ellipsoids_in[kEllipsoidsKey] >> serializable_ellipsoid_results;
  ellipsoids_in.release();
  ellipsoid_results = serializable_ellipsoid_results.getEntry();
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_OUTPUT_PROBLEM_DATA_FILE_STORAGE_IO_H
