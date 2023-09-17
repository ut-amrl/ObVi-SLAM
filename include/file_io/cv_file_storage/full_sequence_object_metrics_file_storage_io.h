//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_FULL_SEQUENCE_OBJECT_METRICS_FILE_STORAGE_IO_H
#define UT_VSLAM_FULL_SEQUENCE_OBJECT_METRICS_FILE_STORAGE_IO_H

#include <evaluation/object_evaluation_utils.h>
#include <file_io/cv_file_storage/base_metrics_file_storage_io.h>
#include <file_io/cv_file_storage/file_storage_io_utils.h>
#include <file_io/cv_file_storage/vslam_obj_types_file_storage_io.h>

#include <filesystem>

namespace vslam_types_refactor {

const std::string kMetricsEntryKey = "obj_metrics";

class SerializableSingleTrajectoryObjectMetrics
    : public FileStorageSerializable<SingleTrajectoryObjectMetrics> {
 public:
  SerializableSingleTrajectoryObjectMetrics()
      : FileStorageSerializable<SingleTrajectoryObjectMetrics>() {}
  SerializableSingleTrajectoryObjectMetrics(
      const SingleTrajectoryObjectMetrics &data)
      : FileStorageSerializable<SingleTrajectoryObjectMetrics>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kOptGtObjForEstObjLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          std::optional<ObjectId>,
                          SerializableOptional<ObjectId, SerializableObjectId>>(
              data_.opt_gt_obj_for_est_obj_);
    fs << kIouForGtObjLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          double,
                          SerializableDouble>(data_.iou_for_gt_obj_);
    fs << kPosDiffForEstObjLabel
       << SerializableMap<ObjectId,
                          SerializableObjectId,
                          std::optional<double>,
                          SerializableOptional<double, SerializableDouble>>(
              data_.pos_diff_for_est_obj_);
    fs << kRecallLabel << data_.recall_;
    fs << kNumGtObjsLabel << data_.num_gt_objs_;
    fs << kMissedGtObjsLabel << data_.missed_gt_objs_;
    fs << kObjectsPerGtObjLabel << data_.objects_per_gt_obj_;
    fs << kAveragePosDeviationLabel << data_.average_pos_deviation_;
    fs << kAvgIouLabel << data_.avg_iou_;
    fs << kMedianPosDeviationLabel << data_.median_pos_deviation_;
    fs << kMedianIouLabel << data_.median_iou_;
    fs << kPosDevStatsLabel
       << SerializableMetricsDistributionStatistics(data_.pos_dev_stats_);
    fs << kIouStatsLabel
       << SerializableMetricsDistributionStatistics(data_.iou_stats_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableMap<ObjectId,
                    SerializableObjectId,
                    std::optional<ObjectId>,
                    SerializableOptional<ObjectId, SerializableObjectId>>
        ser_opt_gt_obj_for_est_obj;
    node[kOptGtObjForEstObjLabel] >> ser_opt_gt_obj_for_est_obj;
    data_.opt_gt_obj_for_est_obj_ = ser_opt_gt_obj_for_est_obj.getEntry();

    SerializableMap<ObjectId, SerializableObjectId, double, SerializableDouble>
        ser_iou_for_gt_obj;
    node[kIouForGtObjLabel] >> ser_iou_for_gt_obj;
    data_.iou_for_gt_obj_ = ser_iou_for_gt_obj.getEntry();

    SerializableMap<ObjectId,
                    SerializableObjectId,
                    std::optional<double>,
                    SerializableOptional<double, SerializableDouble>>
        ser_pos_diff_for_est_obj;
    node[kPosDiffForEstObjLabel] >> ser_pos_diff_for_est_obj;
    data_.pos_diff_for_est_obj_ = ser_pos_diff_for_est_obj.getEntry();

    data_.recall_ = node[kRecallLabel];
    data_.num_gt_objs_ = node[kNumGtObjsLabel];
    data_.missed_gt_objs_ = node[kMissedGtObjsLabel];
    data_.objects_per_gt_obj_ = node[kObjectsPerGtObjLabel];
    data_.average_pos_deviation_ = node[kAveragePosDeviationLabel];
    data_.avg_iou_ = node[kAvgIouLabel];
    data_.median_pos_deviation_ = node[kMedianPosDeviationLabel];
    data_.median_iou_ = node[kMedianIouLabel];

    SerializableMetricsDistributionStatistics ser_pos_dev_stats;
    node[kPosDevStatsLabel] >> ser_pos_dev_stats;
    data_.pos_dev_stats_ = ser_pos_dev_stats.getEntry();

    SerializableMetricsDistributionStatistics ser_iou_stats;
    node[kIouStatsLabel] >> ser_iou_stats;
    data_.iou_stats_ = ser_iou_stats.getEntry();
  }

 protected:
  using FileStorageSerializable<SingleTrajectoryObjectMetrics>::data_;

 private:
  inline static const std::string kOptGtObjForEstObjLabel =
      "opt_gt_obj_for_est_obj";
  inline static const std::string kIouForGtObjLabel = "iou_for_gt_obj";
  inline static const std::string kPosDiffForEstObjLabel =
      "pos_diff_for_est_obj";
  inline static const std::string kRecallLabel = "recall";
  inline static const std::string kNumGtObjsLabel = "num_gt_objs";
  inline static const std::string kMissedGtObjsLabel = "missed_gt_objs";
  inline static const std::string kObjectsPerGtObjLabel = "objects_per_gt_obj";
  inline static const std::string kAveragePosDeviationLabel =
      "average_pos_deviation";
  inline static const std::string kAvgIouLabel = "avg_iou";
  inline static const std::string kMedianPosDeviationLabel =
      "median_pos_deviation";
  inline static const std::string kMedianIouLabel = "median_iou";

  inline static const std::string kPosDevStatsLabel = "pos_dev_stats";
  inline static const std::string kIouStatsLabel = "iou_stats";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableSingleTrajectoryObjectMetrics &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableSingleTrajectoryObjectMetrics &data,
                 const SerializableSingleTrajectoryObjectMetrics &default_data =
                     SerializableSingleTrajectoryObjectMetrics()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

class SerializableFullSequenceObjectMetrics
    : public FileStorageSerializable<FullSequenceObjectMetrics> {
 public:
  SerializableFullSequenceObjectMetrics()
      : FileStorageSerializable<FullSequenceObjectMetrics>() {}
  SerializableFullSequenceObjectMetrics(const FullSequenceObjectMetrics &data)
      : FileStorageSerializable<FullSequenceObjectMetrics>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kIndivTrajectoryObjectMetricsLabel
       << SerializableVector<SingleTrajectoryObjectMetrics,
                             SerializableSingleTrajectoryObjectMetrics>(
              data_.indiv_trajectory_object_metrics_);
    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    SerializableVector<SingleTrajectoryObjectMetrics,
                       SerializableSingleTrajectoryObjectMetrics>
        serializable_single_trajectory_object_metrics;
    node[kIndivTrajectoryObjectMetricsLabel] >>
        serializable_single_trajectory_object_metrics;
    data_.indiv_trajectory_object_metrics_ =
        serializable_single_trajectory_object_metrics.getEntry();
  }

 protected:
  using FileStorageSerializable<FullSequenceObjectMetrics>::data_;

 private:
  inline static const std::string kIndivTrajectoryObjectMetricsLabel =
      "indiv_trajectory_object_metrics";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableFullSequenceObjectMetrics &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableFullSequenceObjectMetrics &data,
                 const SerializableFullSequenceObjectMetrics &default_data =
                     SerializableFullSequenceObjectMetrics()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

void writeFullSequenceObjectMetrics(const std::string &metrics_file_name,
                                    const FullSequenceObjectMetrics &metrics) {
  cv::FileStorage metrics_file_out(metrics_file_name, cv::FileStorage::WRITE);

  metrics_file_out << kMetricsEntryKey
                   << SerializableFullSequenceObjectMetrics(metrics);
  metrics_file_out.release();
}

void readFullSequenceObjectMetrics(const std::string &metrics_file_name,
                                   FullSequenceObjectMetrics &metrics) {
  if (!std::filesystem::exists(metrics_file_name)) {
    LOG(ERROR) << "Trying to read file " << metrics_file_name
               << " that does not exist";
    return;
  }
  cv::FileStorage metrics_file_in(metrics_file_name, cv::FileStorage::READ);
  SerializableFullSequenceObjectMetrics serializable_metrics;
  metrics_file_in[kMetricsEntryKey] >> serializable_metrics;
  metrics_file_in.release();
  metrics = serializable_metrics.getEntry();
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_FULL_SEQUENCE_OBJECT_METRICS_FILE_STORAGE_IO_H
