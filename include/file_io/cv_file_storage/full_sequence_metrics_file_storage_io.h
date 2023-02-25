//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_FULL_SEQUENCE_METRICS_FILE_STORAGE_IO_H
#define UT_VSLAM_FULL_SEQUENCE_METRICS_FILE_STORAGE_IO_H

#include <evaluation/trajectory_metrics.h>
#include <file_io/cv_file_storage/file_storage_io_utils.h>

namespace vslam_types_refactor {

const std::string kMetricsEntryKey = "metrics";

class SerializableATEResults : public FileStorageSerializable<ATEResults> {
 public:
  SerializableATEResults() : FileStorageSerializable<ATEResults>() {}
  SerializableATEResults(const ATEResults &data)
      : FileStorageSerializable<ATEResults>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kRmseTranslErrLabel << data_.rmse_transl_err_;
    fs << kRmseRotErrLabel << data_.rmse_rot_err_;
    fs << kValidPosesUsedInScoreLabel << data_.valid_poses_used_in_score_;
    fs << kLostPosesLabel << data_.lost_poses_;
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.rmse_transl_err_ = node[kRmseTranslErrLabel];
    data_.rmse_rot_err_ = node[kRmseRotErrLabel];
    data_.valid_poses_used_in_score_ = node[kValidPosesUsedInScoreLabel];
    data_.lost_poses_ = node[kLostPosesLabel];
  }

 protected:
  using FileStorageSerializable<ATEResults>::data_;

 private:
  inline static const std::string kRmseTranslErrLabel = "rmse_transl_err";
  inline static const std::string kRmseRotErrLabel = "rmse_rot_err";
  inline static const std::string kValidPosesUsedInScoreLabel =
      "valid_poses_used_in_score";
  inline static const std::string kLostPosesLabel = "lost_poses";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableATEResults &data) {
  data.write(fs);
}

static void read(
    const cv::FileNode &node,
    SerializableATEResults &data,
    const SerializableATEResults &default_data = SerializableATEResults()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableTrajectoryMetrics
    : public FileStorageSerializable<TrajectoryMetrics> {
 public:
  SerializableTrajectoryMetrics()
      : FileStorageSerializable<TrajectoryMetrics>() {}
  SerializableTrajectoryMetrics(const TrajectoryMetrics &data)
      : FileStorageSerializable<TrajectoryMetrics>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kWaypointDeviationsLabel
       << SerializableMap<
              int,
              SerializableInt,
              std::pair<std::vector<double>, std::vector<double>>,
              SerializablePair<std::vector<double>,
                               SerializableVector<double, SerializableDouble>,
                               std::vector<double>,
                               SerializableVector<double, SerializableDouble>>>(
              data_.waypoint_deviations_);
    fs << kAllTranslationDeviationsLabel
       << SerializableVector<double, SerializableDouble>(
              data_.all_translation_deviations_);
    fs << kAllRotationDeviationsLabel
       << SerializableVector<double, SerializableDouble>(
              data_.all_rotation_deviations_);
    fs << kAteResultsLabel << SerializableATEResults(data_.ate_results_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableMap<
        int,
        SerializableInt,
        std::pair<std::vector<double>, std::vector<double>>,
        SerializablePair<std::vector<double>,
                         SerializableVector<double, SerializableDouble>,
                         std::vector<double>,
                         SerializableVector<double, SerializableDouble>>>
        ser_waypoint_deviations;
    node[kWaypointDeviationsLabel] >> ser_waypoint_deviations;
    data_.waypoint_deviations_ = ser_waypoint_deviations.getEntry();
    SerializableVector<double, SerializableDouble>
        ser_all_translation_deviations;
    node[kAllTranslationDeviationsLabel] >> ser_all_translation_deviations;
    data_.all_translation_deviations_ =
        ser_all_translation_deviations.getEntry();
    SerializableVector<double, SerializableDouble> ser_all_rotation_deviations;
    node[kAllRotationDeviationsLabel] >> ser_all_rotation_deviations;
    data_.all_rotation_deviations_ = ser_all_rotation_deviations.getEntry();
    SerializableATEResults ser_trajectory_sequence_ate_results;
    node[kAteResultsLabel] >> ser_trajectory_sequence_ate_results;
    data_.ate_results_ = ser_trajectory_sequence_ate_results.getEntry();
  }

 protected:
  using FileStorageSerializable<TrajectoryMetrics>::data_;

 private:
  inline static const std::string kWaypointDeviationsLabel =
      "waypoint_deviations";
  inline static const std::string kAllTranslationDeviationsLabel =
      "all_translation_deviations";
  inline static const std::string kAllRotationDeviationsLabel =
      "all_rotation_deviations";
  inline static const std::string kAteResultsLabel =
      "trajectory_sequence_ate_results";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableTrajectoryMetrics &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableTrajectoryMetrics &data,
                 const SerializableTrajectoryMetrics &default_data =
                     SerializableTrajectoryMetrics()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableFullSequenceMetrics
    : public FileStorageSerializable<FullSequenceMetrics> {
 public:
  SerializableFullSequenceMetrics()
      : FileStorageSerializable<FullSequenceMetrics>() {}
  SerializableFullSequenceMetrics(const FullSequenceMetrics &data)
      : FileStorageSerializable<FullSequenceMetrics>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kIndivTrajectoryMetricsLabel
       << SerializableVector<TrajectoryMetrics, SerializableTrajectoryMetrics>(
              data_.indiv_trajectory_metrics_);
    fs << kSequenceMetricsLabel
       << SerializableTrajectoryMetrics(data_.sequence_metrics_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableVector<TrajectoryMetrics, SerializableTrajectoryMetrics>
        serializable_single_trajectory_metrics;
    node[kIndivTrajectoryMetricsLabel] >>
        serializable_single_trajectory_metrics;
    data_.indiv_trajectory_metrics_ =
        serializable_single_trajectory_metrics.getEntry();
    SerializableTrajectoryMetrics ser_sequence_metrics;
    node[kSequenceMetricsLabel] >> ser_sequence_metrics;
    data_.sequence_metrics_ = ser_sequence_metrics.getEntry();
  }

 protected:
  using FileStorageSerializable<FullSequenceMetrics>::data_;

 private:
  inline static const std::string kIndivTrajectoryMetricsLabel =
      "indiv_trajectory_metrics";
  inline static const std::string kSequenceMetricsLabel = "sequence_metrics";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableFullSequenceMetrics &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableFullSequenceMetrics &data,
                 const SerializableFullSequenceMetrics &default_data =
                     SerializableFullSequenceMetrics()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void writeFullSequenceMetrics(const std::string &metrics_file_name,
                              const FullSequenceMetrics &metrics) {
  cv::FileStorage metrics_file_out(metrics_file_name, cv::FileStorage::WRITE);

  metrics_file_out << kMetricsEntryKey
                   << SerializableFullSequenceMetrics(metrics);
  metrics_file_out.release();
}

void readFullSequenceMetrics(const std::string &metrics_file_name,
                             FullSequenceMetrics &metrics) {
  cv::FileStorage metrics_file_in(metrics_file_name, cv::FileStorage::READ);
  SerializableFullSequenceMetrics serializable_metrics;
  metrics_file_in[kMetricsEntryKey] >> serializable_metrics;
  metrics_file_in.release();
  metrics = serializable_metrics.getEntry();
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CONFIG_FILE_STORAGE_IO_H
