//
// Created by amanda on 1/27/23.
//

#ifndef UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
#define UT_VSLAM_DEBUGGING_IMAGE_UTILS_H

#include <cv_bridge/cv_bridge.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>
#include <std_msgs/ColorRGBA.h>
#include <util/random.h>

#include <opencv2/highgui.hpp>
#include <string>
#include <utility>

namespace vslam_types_refactor {
const static constexpr double kCornerTextFontSize = 1.5;
const static int kCornerTextLeftLabelXOffset = 10;
const static int kBottomCornerLabelYOffsetFromBottom = 10;
const static int kCornerTextThickness = 2;

const static constexpr double kDefaultBbTextSize = 0.75;
const static int kDefaultBbTextThickness = 2;
const static int kDefaultBbLineThickness = 2;
const static int kBoundingBoxCornerYLabelOffset = 30;

const static std::string kPngExtension = ".png";

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

void drawTextLeftAlign(const cv_bridge::CvImagePtr &cv_ptr,
                       const std::string &img_disp_text,
                       const cv::Point &anchor_point,
                       const std::pair<double, double> &img_height_and_width,
                       const std_msgs::ColorRGBA &text_color,
                       const double &text_font_size,
                       const int &text_thickness) {
  cv::Point pos_checked_anchor_point = anchor_point;
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(img_disp_text,
                                       cv::FONT_HERSHEY_SIMPLEX,
                                       text_font_size,
                                       cv::LINE_8,
                                       &baseline);
  if ((anchor_point.x + text_size.width) > img_height_and_width.second) {
    pos_checked_anchor_point.x = img_height_and_width.second - text_size.width;
  }

  cv::putText(cv_ptr->image,
              img_disp_text,
              pos_checked_anchor_point,
              cv::FONT_HERSHEY_SIMPLEX,
              text_font_size,
              convertColorMsgToOpenCvColor(text_color),
              text_thickness);
}

void drawTextRightAlign(const cv_bridge::CvImagePtr &cv_ptr,
                        const std::string &img_disp_text,
                        const cv::Point &anchor_point,
                        const std::pair<double, double> &img_height_and_width,
                        const std_msgs::ColorRGBA &text_color,
                        const double &text_font_size,
                        const int &text_thickness) {
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(img_disp_text,
                                       cv::FONT_HERSHEY_SIMPLEX,
                                       text_font_size,
                                       cv::LINE_8,
                                       &baseline);
  drawTextLeftAlign(cv_ptr,
                    img_disp_text,
                    cv::Point(anchor_point.x - text_size.width, anchor_point.y),
                    img_height_and_width,
                    text_color,
                    text_font_size,
                    text_thickness);
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

void displayBoundingBoxOnImage(
    const BbCornerPair<double> &bounding_box_corners,
    const std::pair<double, double> &img_height_and_width,
    const std_msgs::ColorRGBA &color,
    const cv_bridge::CvImagePtr &cv_ptr,
    const std::optional<std::string> &upper_left_corner_text,
    const std::optional<std::string> &upper_right_corner_text,
    const std::optional<std::string> &lower_left_corner_text,
    const std::optional<std::string> &lower_right_corner_text,
    const double &text_size = kDefaultBbTextSize,
    const int &bb_line_thickness = kDefaultBbLineThickness,
    const int &text_thickness = kDefaultBbTextThickness) {
  cv::rectangle(
      cv_ptr->image,
      cv::Point(bounding_box_corners.first.x(), bounding_box_corners.first.y()),
      cv::Point(bounding_box_corners.second.x(),
                bounding_box_corners.second.y()),
      convertColorMsgToOpenCvColor(color),
      bb_line_thickness);

  if (upper_left_corner_text.has_value()) {
    drawTextLeftAlign(cv_ptr,
                      upper_left_corner_text.value(),
                      cv::Point(bounding_box_corners.first.x(),
                                bounding_box_corners.first.y() +
                                    kBoundingBoxCornerYLabelOffset),
                      img_height_and_width,
                      color,
                      text_size,
                      text_thickness);
  }
  if (upper_right_corner_text.has_value()) {
    drawTextLeftAlign(cv_ptr,
                      upper_right_corner_text.value(),
                      cv::Point(bounding_box_corners.second.x(),
                                bounding_box_corners.first.y() +
                                    kBoundingBoxCornerYLabelOffset),
                      img_height_and_width,
                      color,
                      text_size,
                      text_thickness);
  }

  if (lower_left_corner_text.has_value()) {
    drawTextLeftAlign(cv_ptr,
                      lower_left_corner_text.value(),
                      cv::Point(bounding_box_corners.first.x(),
                                bounding_box_corners.second.y() +
                                    kBoundingBoxCornerYLabelOffset),
                      img_height_and_width,
                      color,
                      text_size,
                      text_thickness);
  }
  if (lower_right_corner_text.has_value()) {
    drawTextLeftAlign(cv_ptr,
                      lower_right_corner_text.value(),
                      cv::Point(bounding_box_corners.second.x(),
                                bounding_box_corners.second.y() +
                                    kBoundingBoxCornerYLabelOffset),
                      img_height_and_width,
                      color,
                      text_size,
                      text_thickness);
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

std_msgs::ColorRGBA generateRandomColor(util_random::Random &rand_gen) {
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = (rand_gen.UniformRandom() + rand_gen.UniformRandom()) / 2;
  color.g = rand_gen.UniformRandom();
  color.b = rand_gen.UniformRandom();
  return color;
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

cv::Mat scaleImage(const double &scaling_ratio, const cv::Mat &orig_img) {
  cv::Mat resized;
  cv::resize(orig_img,
             resized,
             cv::Size(),
             scaling_ratio,
             scaling_ratio,
             cv::INTER_LANCZOS4);
  return resized;
}

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
