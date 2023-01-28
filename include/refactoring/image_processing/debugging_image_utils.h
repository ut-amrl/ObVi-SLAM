//
// Created by amanda on 1/27/23.
//

#ifndef UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
#define UT_VSLAM_DEBUGGING_IMAGE_UTILS_H

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/ColorRGBA.h>

#include <opencv2/highgui.hpp>
#include <string>
#include <utility>

namespace vslam_types_refactor {
const static constexpr double kCornerTextFontSize = 1.5;
const static int kCornerTextLeftLabelXOffset = 10;
const static int kBottomCornerLabelYOffsetFromBottom = 10;
const static int kCornerTextThickness = 2;

cv::Scalar convertColorMsgToOpenCvColor(const std_msgs::ColorRGBA &color) {
  return CV_RGB(color.r * 255, color.g * 255, color.b * 255);
}

void optionallyDisplayLeftCornerTextOnImage(
    const std::string &img_disp_text,
    const std::pair<double, double> &img_height_and_width,
    const cv_bridge::CvImagePtr &cv_ptr) {
  if (!img_disp_text.empty()) {
    std_msgs::ColorRGBA text_color;
    text_color.a = text_color.r = text_color.g = text_color.b = 1;
    cv::putText(cv_ptr->image,
                img_disp_text,
                cv::Point(kCornerTextLeftLabelXOffset,
                          img_height_and_width.first -
                              kBottomCornerLabelYOffsetFromBottom),
                cv::FONT_HERSHEY_SIMPLEX,
                kCornerTextFontSize,
                convertColorMsgToOpenCvColor(text_color),
                kCornerTextThickness);
  }
}

void optionallyDisplayRightCornerTextOnImage(
    const std::string &img_disp_text,
    const std::pair<double, double> &img_height_and_width,
    const cv_bridge::CvImagePtr &cv_ptr) {
  if (!img_disp_text.empty()) {
    std_msgs::ColorRGBA text_color;
    text_color.a = text_color.r = text_color.g = text_color.b = 1;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(img_disp_text,
                                         cv::FONT_HERSHEY_SIMPLEX,
                                         kCornerTextFontSize,
                                         cv::LINE_8,
                                         &baseline);
    cv::putText(cv_ptr->image,
                img_disp_text,
                cv::Point(img_height_and_width.second -
                              kCornerTextLeftLabelXOffset - text_size.width,
                          img_height_and_width.first -
                              kBottomCornerLabelYOffsetFromBottom),
                cv::FONT_HERSHEY_SIMPLEX,
                kCornerTextFontSize,
                convertColorMsgToOpenCvColor(text_color),
                kCornerTextThickness);
  }
}

std_msgs::ColorRGBA brightenColor(const std_msgs::ColorRGBA &original,
                                  const double &brighter_percent) {
  std_msgs::ColorRGBA brighter = original;
  brighter.r = 1.0 - (1.0 - original.r) * (1 - brighter_percent);
  brighter.g = 1.0 - (1.0 - original.g) * (1 - brighter_percent);
  brighter.b = 1.0 - (1.0 - original.b) * (1 - brighter_percent);
  return brighter;
}

void drawTinyCircleOnImage(const PixelCoord<double> &px,
                           const std_msgs::ColorRGBA &color,
                           cv_bridge::CvImagePtr &cv_ptr,
                           const int &circle_rad = 3,
                           const int &thickness = 2) {
  // TODO verify
  cv::circle(cv_ptr->image,
             cv::Point(px.x(), px.y()),
             circle_rad,
             convertColorMsgToOpenCvColor(color),
             thickness);
}

// assume each visualization has the same size
cv::Mat generateMosaic(const std::vector<cv::Mat> &indiv_visualizations,
                       const int &nimages_each_row = -1) {
  if (indiv_visualizations.empty()) {
    LOG(FATAL) << "Doesn't support this control path! "
               << "Please check ids_and_visualizations is non-empty before "
                  "calling SummarizeVisualization()";
  }

  size_t summary_height, summary_width;
  const size_t &height = indiv_visualizations.begin()->rows;
  const size_t &width = indiv_visualizations.begin()->cols;
  const size_t &nvisualization = indiv_visualizations.size();
  const size_t &nimages_per_row =
      (nimages_each_row == -1) ? nvisualization : (size_t)nimages_each_row;
  const size_t &nimages_per_col = (nvisualization % nimages_per_row == 0)
                                      ? nvisualization / nimages_per_row
                                      : nvisualization / nimages_per_row + 1;
  summary_height = nimages_per_col * height;
  summary_width = nimages_per_row * width;
  cv::Mat summary = cv::Mat::zeros(summary_height, summary_width, CV_8UC3);
  std::vector<std::pair<int, int>> coordinates;
  for (size_t i = 0; i < nvisualization; ++i) {
    coordinates.emplace_back((i / nimages_per_row) * height,
                             (i % nimages_per_row) * width);
  }

  for (size_t i = 0; i < indiv_visualizations.size(); ++i) {
    const cv::Mat &visualization = indiv_visualizations[i];
    const int &row = coordinates[i].first;
    const int &col = coordinates[i].second;
    visualization.copyTo(
        summary(cv::Range(row, row + height), cv::Range(col, col + width)));
  }
  return summary;
}

// assume each visualization has the same size
template <typename IdType>
cv::Mat generateMosaic(
    const util::BoostHashMap<IdType, cv::Mat> &ids_and_visualizations,
    const int &nimages_each_row = -1) {
  if (ids_and_visualizations.empty()) {
    LOG(FATAL) << "Doesn't support this control path! "
               << "Please check ids_and_visualizations is non-empty before "
                  "calling SummarizeVisualization()";
  }
  std::vector<IdType> ids;
  for (const auto &id_and_visualization : ids_and_visualizations) {
    ids.push_back(id_and_visualization.first);
  }
  std::sort(ids.begin(), ids.end());

  std::vector<cv::Mat> indiv_visualizations;
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto &id = ids[i];
    const cv::Mat &visualization = ids_and_visualizations.at(id);
    indiv_visualizations.emplace_back(visualization);
  }
  return generateMosaic(indiv_visualizations, nimages_each_row);
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
