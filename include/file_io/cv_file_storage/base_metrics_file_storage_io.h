//
// Created by amanda on 2/10/23.
//

#ifndef UT_VSLAM_BASE_METRICS_FILE_STORAGE_IO_H
#define UT_VSLAM_BASE_METRICS_FILE_STORAGE_IO_H

#include <evaluation/evaluation_utils.h>
#include <file_io/cv_file_storage/file_storage_io_utils.h>

#include <filesystem>

namespace vslam_types_refactor {

class SerializableMetricsDistributionStatistics
    : public FileStorageSerializable<MetricsDistributionStatistics> {
 public:
  SerializableMetricsDistributionStatistics()
      : FileStorageSerializable<MetricsDistributionStatistics>() {}
  SerializableMetricsDistributionStatistics(
      const MetricsDistributionStatistics &data)
      : FileStorageSerializable<MetricsDistributionStatistics>(data) {}

  virtual void write(cv::FileStorage &fs) const override {
    fs << "{";
    fs << kNumValsLabel << data_.num_vals_;
    fs << kAverageLabel << data_.average_;
    fs << kStdDevLabel << data_.std_dev_;
    fs << kMedianLabel << data_.median_;
    fs << kMinLabel << data_.min_;
    fs << kMaxLabel << data_.max_;
    fs << kLowerQuartileLabel << data_.lower_quartile_;
    fs << kUpperQuartileLabel << data_.upper_quartile_;
    fs << kRmseLabel << data_.rmse_;
    fs << kSquaredErrStdDevLabel << data_.squared_err_std_dev_;

    fs << kErrorsLabel
       << SerializableVector<double, SerializableDouble>(data_.errors_);

    fs << "}";
  }

  virtual void read(const cv::FileNode &node) override {
    data_.num_vals_ = node[kNumValsLabel];
    data_.average_ = node[kAverageLabel];
    data_.std_dev_ = node[kStdDevLabel];
    data_.median_ = node[kMedianLabel];
    data_.min_ = node[kMinLabel];
    data_.max_ = node[kMaxLabel];

    data_.lower_quartile_ = node[kLowerQuartileLabel];
    data_.upper_quartile_ = node[kUpperQuartileLabel];
    data_.rmse_ = node[kRmseLabel];
    data_.squared_err_std_dev_ = node[kSquaredErrStdDevLabel];

    SerializableVector<double, SerializableDouble> serializable_errors;
    node[kErrorsLabel] >> serializable_errors;
    data_.errors_ = serializable_errors.getEntry();
  }

 protected:
  using FileStorageSerializable<MetricsDistributionStatistics>::data_;

 private:
  inline static const std::string kNumValsLabel = "num_vals";
  inline static const std::string kAverageLabel = "average";
  inline static const std::string kStdDevLabel = "std_dev";
  inline static const std::string kMedianLabel = "median";
  inline static const std::string kMinLabel = "min";
  inline static const std::string kMaxLabel = "max";
  inline static const std::string kLowerQuartileLabel = "lower_quartile";
  inline static const std::string kUpperQuartileLabel = "upper_quartile";
  inline static const std::string kRmseLabel = "rmse";
  inline static const std::string kSquaredErrStdDevLabel =
      "squared_err_std_dev";

  inline static const std::string kErrorsLabel = "errors";
};

static void write(cv::FileStorage &fs,
                  const std::string &,
                  const SerializableMetricsDistributionStatistics &data) {
  data.write(fs);
}

static void read(const cv::FileNode &node,
                 SerializableMetricsDistributionStatistics &data,
                 const SerializableMetricsDistributionStatistics &default_data =
                     SerializableMetricsDistributionStatistics()) {
  if (node.empty()) {
    data = default_data;
  } else {
    data.read(node);
  }
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_BASE_METRICS_FILE_STORAGE_IO_H
