//
// Created by amanda on 1/27/23.
//

#ifndef UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
#define UT_VSLAM_DEBUGGING_IMAGE_UTILS_H

#include <base_lib/basic_utils.h>
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

cv::Scalar convertColorMsgToOpenCvColor(const std_msgs::ColorRGBA &color);

void optionallyDisplayLeftCornerTextOnImage(
    const std::string &img_disp_text,
    const std::pair<double, double> &img_height_and_width,
    const cv_bridge::CvImagePtr &cv_ptr);

void drawTextLeftAlign(const cv_bridge::CvImagePtr &cv_ptr,
                       const std::string &img_disp_text,
                       const cv::Point &anchor_point,
                       const std::pair<double, double> &img_height_and_width,
                       const std_msgs::ColorRGBA &text_color,
                       const double &text_font_size,
                       const int &text_thickness);

void drawTextRightAlign(const cv_bridge::CvImagePtr &cv_ptr,
                        const std::string &img_disp_text,
                        const cv::Point &anchor_point,
                        const std::pair<double, double> &img_height_and_width,
                        const std_msgs::ColorRGBA &text_color,
                        const double &text_font_size,
                        const int &text_thickness);

void optionallyDisplayRightCornerTextOnImage(
    const std::string &img_disp_text,
    const std::pair<double, double> &img_height_and_width,
    const cv_bridge::CvImagePtr &cv_ptr);

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
    const int &text_thickness = kDefaultBbTextThickness);

std_msgs::ColorRGBA brightenColor(const std_msgs::ColorRGBA &original,
                                  const double &brighter_percent);

void drawTinyCircleOnImage(const PixelCoord<double> &px,
                           const std_msgs::ColorRGBA &color,
                           cv_bridge::CvImagePtr &cv_ptr,
                           const int &circle_rad = 3,
                           const int &thickness = 2);

std_msgs::ColorRGBA generateRandomColor(util_random::Random &rand_gen);

// assume each visualization has the same size
cv::Mat generateMosaic(const std::vector<cv::Mat> &indiv_visualizations,
                       const int &nimages_each_row = -1);

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

cv::Mat scaleImage(const double &scaling_ratio, const cv::Mat &orig_img);

}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_DEBUGGING_IMAGE_UTILS_H
