
#include <cv_bridge/cv_bridge.h>
#include <file_io/bounding_box_by_timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(bounding_boxes_file,
              "",
              "File containing id-less and unfiltered bounding boxes");
DEFINE_string(depth_img_rosbag, "", "");
DEFINE_uint32(min_depth_mm,
              std::numeric_limits<uint32_t>::max(),
              "Min depth that the lower percentile should have in mm");
DEFINE_uint32(max_depth_mm,
              0,
              "Maximum depth that the upper percentile should have in mm");
DEFINE_double(
    min_depth_percentile,
    -1,
    "Percentile (0 to 1) that we should compute the min depth range for");
DEFINE_double(
    max_depth_percentile,
    -1,
    "Percentile (0 to 1) that we should compute the max depth range for");
DEFINE_string(class_to_use, "", "");
DEFINE_string(bounding_boxes_with_ids_file,
              "",
              "File to add filtered boxes to");
DEFINE_uint64(camera_id, 0, "Camera ID to use");

std::pair<double, double> getDepthsAtPercentile(
    const double &min_percentile,
    const double &max_percentile,
    const uint64_t &min_x,
    const uint64_t &min_y,
    const uint64_t &max_x,
    const uint64_t &max_y,
    const sensor_msgs::ImageConstPtr &image,
    ros::Publisher pub,
    ros::Publisher bb_pub) {
  //  LOG(INFO) << "Msg encoding " << image->encoding;
  cv_bridge::CvImageConstPtr cv_img =
      cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO16);
  pub.publish(image);

  double min_val;
  double max_val;
  cv::Point min_loc;
  cv::Point max_loc;
  cv::minMaxLoc(cv_img->image, &min_val, &max_val, &min_loc, &max_loc);

  cv::Mat basic_img_copy = cv_img->image.clone();

  cv::rectangle(basic_img_copy,
                cv::Point(min_x, min_y),
                cv::Point(max_x, max_y),
                max_val,
                2);

  cv_bridge::CvImage cv_img_with_bb;
  cv_img_with_bb.image = basic_img_copy;
  cv_img_with_bb.header = cv_img->header;
  cv_img_with_bb.encoding = cv_img->encoding;
  //  LOG(INFO) << cv_img->encoding;

  bb_pub.publish(cv_img_with_bb);

  //  LOG(INFO) << "Encoding " << cv_img->encoding;
  cv::Point point;
  point.x = max_x;
  point.y = max_y;
  //  LOG(INFO) << "Depth val: " << cv_img->image.at<uint16_t>(point);

  //  cv::imshow("orig image", cv_img->image);
  //  cv::waitKey(0);

  LOG(INFO) << "Image width, height: " << image->width << ", " << image->height;
  LOG(INFO) << "Max x,y: " << max_x << ", " << max_y;

  cv::Mat cropped_image = cv_img->image(
      cv::Range(min_y, std::min((unsigned int)max_y + 1, image->height)),
      cv::Range(min_x, std::min(image->width, (unsigned int)max_x + 1)));
  //  LOG(INFO) << "Done cropping";

  //  cropped_image.convertTo(cropped_image, CV_16U);

  std::vector<uint16_t> cropped_as_vec_orig(cropped_image.begin<uint16_t>(),
                                            cropped_image.end<uint16_t>());

  std::vector<uint16_t> cropped_as_vec = cropped_as_vec_orig;
  for (size_t cropped_entry_idx = 0; cropped_entry_idx < cropped_as_vec.size();
       cropped_entry_idx++) {
    if (cropped_as_vec[cropped_entry_idx] == 0) {
      cropped_as_vec[cropped_entry_idx] = std::numeric_limits<uint16_t>::max();
    }
  }

  std::sort(cropped_as_vec.begin(), cropped_as_vec.end());

  //  for (const uint16_t &pixel_val : cropped_as_vec) {
  //    LOG(INFO) << pixel_val;
  //  }

  size_t min_percentile_idx = (cropped_as_vec.size() - 1) * min_percentile;
  size_t max_percentile_idx = (cropped_as_vec.size() - 1) * max_percentile;

  uint16_t min_perc_val = cropped_as_vec[min_percentile_idx];
  uint16_t max_perc_val = cropped_as_vec[max_percentile_idx];

  //  for (size_t perc_check = 0; perc_check <= 100; perc_check += 5) {
  //    LOG(INFO) << "Pixel value at percentile " << perc_check << " is "
  //              << cropped_as_vec[(cropped_as_vec.size() - 1) *
  //                                (((float)perc_check) / 100.0)];
  //  }

  cv::Mat img_copy = cv_img->image.clone();
  cv::rectangle(
      img_copy, cv::Point(min_x, min_y), cv::Point(max_x, max_y), max_val, 2);

  //  LOG(INFO) << "Min/max perc vals " << min_perc_val << ", " << max_perc_val;
  //  cv::imshow("Depth img", img_copy);
  //  cv::waitKey(0);
  //  exit(1);

  //  cv_bridge::CvImage bb_img;
  //  bb_img.image = img_copy;
  //  bb_img.header = cv_img->header;
  //  bb_img.encoding = cv_img->encoding;
  //  bb_pub.publish(bb_img.toImageMsg());

  //  cv::Mat hist_out;
  //  int hist_num_bins = 256;  // Is this sufficient
  //  int channel = 0;
  //  float hist_pixel_range[] = {
  //      0, ((float)std::numeric_limits<uint16_t>::max()) + 1};
  //
  //  float *pixel_range_nested[] = {hist_pixel_range};
  //  cv::Mat *hist_img = &cropped_image;
  //  int hist_nimages = 1;
  //  int *hist_channels = &channel;
  //  cv::InputArray hist_mask = cv::Mat();
  //  cv::OutputArray hist_hist = hist_out;
  //  int hist_dims = 1;
  //  int *hist_hist_size = &hist_num_bins;
  //  float **hist_ranges = pixel_range_nested;
  //
  //  cv::calcHist(hist_img,
  //               hist_nimages,
  //               hist_channels,
  //               hist_mask,
  //               hist_hist,
  //               hist_dims,
  //               hist_num_bins,
  //               hist_ranges);
  // TODO use calc hist
  return std::make_pair(min_perc_val, max_perc_val);
}

typedef std::pair<uint32_t, uint32_t> Timestamp;

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(std::pair<T1, T2> const &pair) const {
    std::size_t h1 = std::hash<T1>()(pair.first);
    std::size_t h2 = std::hash<T2>()(pair.second);

    return h1 ^ h2;
  }
};

bool timestamp1LessThanOrEqualToT2(const Timestamp &timestamp1,
                                   const Timestamp &timestamp2) {
  if (timestamp1.first != timestamp2.second) {
    return timestamp1.first < timestamp2.first;
  }
  return timestamp1.second <= timestamp2.second;
}

struct timestamp_sort {
  inline bool operator()(const Timestamp &timestamp1,
                         const Timestamp &timestamp2) {
    return timestamp1LessThanOrEqualToT2(timestamp1, timestamp2);
  }
};

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(argc, argv, node_prefix + "bounding_box_extractor");
  ros::NodeHandle n;

  LOG(INFO) << "Reading bounding boxes ";

  std::vector<file_io::BoundingBoxWithTimestamp> unfiltered_bounding_boxes;
  file_io::readBoundingBoxWithTimestampsFromFile(FLAGS_bounding_boxes_file,
                                                 unfiltered_bounding_boxes);

  LOG(INFO) << "Read " << unfiltered_bounding_boxes.size()
            << " unfiltered bounding boxes from file";

  std::unordered_map<Timestamp,
                     std::vector<file_io::BoundingBoxWithTimestamp>,
                     pair_hash>
      bounding_boxes_by_timestamp;
  for (const file_io::BoundingBoxWithTimestamp &candidate_bb :
       unfiltered_bounding_boxes) {
    if (candidate_bb.semantic_class == FLAGS_class_to_use) {
      Timestamp timestamp =
          std::make_pair(candidate_bb.seconds, candidate_bb.nano_seconds);
      std::vector<file_io::BoundingBoxWithTimestamp>
          bounding_boxes_for_timestamp;
      if (bounding_boxes_by_timestamp.find(timestamp) !=
          bounding_boxes_by_timestamp.end()) {
        bounding_boxes_for_timestamp =
            bounding_boxes_by_timestamp.at(timestamp);
      }
      bounding_boxes_for_timestamp.emplace_back(candidate_bb);
      bounding_boxes_by_timestamp[timestamp] = bounding_boxes_for_timestamp;

      //      LOG(INFO) << "Timestamp from bb " << timestamp.first << ", "
      //                << timestamp.second;
    }
  }

  std::unordered_map<Timestamp, sensor_msgs::ImageConstPtr, pair_hash>
      closest_img_to_timestamp;
  rosbag::Bag bag;
  bag.open(FLAGS_depth_img_rosbag, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"/camera/depth/image_raw"};

  ros::Publisher img_publisher =
      n.advertise<sensor_msgs::Image>("depth_img", 10);
  ros::Publisher bb_img_publisher =
      n.advertise<sensor_msgs::Image>("bb_img", 10);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<file_io::BoundingBoxWithTimestamp> filtered_bbs_no_id;

  LOG(INFO) << "Bounding boxes by timestamp size "
            << bounding_boxes_by_timestamp.size();

  LOG(INFO) << "Reading images from file";
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
    Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);

    for (const auto &timestamp_and_bbs : bounding_boxes_by_timestamp) {
      if (timestamp1LessThanOrEqualToT2(timestamp_and_bbs.first,
                                        img_timestamp)) {
        if (closest_img_to_timestamp.find(timestamp_and_bbs.first) ==
            closest_img_to_timestamp.end()) {
          closest_img_to_timestamp[timestamp_and_bbs.first] = msg;
        } else {
          sensor_msgs::Image::ConstPtr curr_closest_img =
              closest_img_to_timestamp.at(timestamp_and_bbs.first);
          Timestamp curr_closest =
              std::make_pair(curr_closest_img->header.stamp.sec,
                             curr_closest_img->header.stamp.nsec);
          if (timestamp1LessThanOrEqualToT2(img_timestamp, curr_closest)) {
            closest_img_to_timestamp[timestamp_and_bbs.first] = msg;
          }
        }
      }
    }
  }

  //  LOG(INFO) << "Closest timestamps to images size "
  //            << closest_img_to_timestamp.size();
  //  for (const auto &timestamp_and_img : closest_img_to_timestamp) {
  //    LOG(INFO) << "Timestamp: " << timestamp_and_img.first.first << ", "
  //              << timestamp_and_img.first.second << ", img timestamp "
  //              << timestamp_and_img.second->header.stamp.sec << ", "
  //              << timestamp_and_img.second->header.stamp.nsec;
  //    LOG(INFO) << "BBs for timestamp "
  //              <<
  //              bounding_boxes_by_timestamp[timestamp_and_img.first].size();
  //  }

  int count = 0;
  for (const auto &timestamp_and_img : closest_img_to_timestamp) {
    //    LOG(INFO) << "Image timestamp: " <<
    //    timestamp_and_img.second->header.stamp;
    std::vector<file_io::BoundingBoxWithTimestamp>
        bounding_boxes_for_timestamp =
            bounding_boxes_by_timestamp.at(timestamp_and_img.first);

    std::vector<
        std::pair<std::pair<double, double>, file_io::BoundingBoxWithTimestamp>>
        appropriate_depth_bbs;
    for (const file_io::BoundingBoxWithTimestamp &bounding_box :
         bounding_boxes_for_timestamp) {
      std::pair<double, double> depths_at_percentiles =
          getDepthsAtPercentile(FLAGS_min_depth_percentile,
                                FLAGS_max_depth_percentile,
                                bounding_box.min_pixel_x,
                                bounding_box.min_pixel_y,
                                bounding_box.max_pixel_x,
                                bounding_box.max_pixel_y,
                                timestamp_and_img.second,
                                img_publisher,
                                bb_img_publisher);
      if ((depths_at_percentiles.first >= FLAGS_min_depth_mm) &&
          (depths_at_percentiles.second <= FLAGS_max_depth_mm)) {
        appropriate_depth_bbs.emplace_back(
            std::make_pair(depths_at_percentiles, bounding_box));
      }
    }
    if (!appropriate_depth_bbs.empty()) {
      if (appropriate_depth_bbs.size() == 1) {
        filtered_bbs_no_id.emplace_back(appropriate_depth_bbs.front().second);
      } else {
        // If multiple, pick the closest one
        std::pair<std::pair<double, double>, file_io::BoundingBoxWithTimestamp>
            closest_bb = appropriate_depth_bbs.front();
        for (size_t i = 1; i < appropriate_depth_bbs.size(); i++) {
          std::pair<std::pair<double, double>,
                    file_io::BoundingBoxWithTimestamp>
              curr_bb = appropriate_depth_bbs[i];
          if (curr_bb.first.first <= closest_bb.first.first) {
            closest_bb = curr_bb;
          }
        }
        filtered_bbs_no_id.emplace_back(closest_bb.second);
      }
    }
    count++;
    if (count > 10) {
      //      exit(1);
    }
  }

  std::vector<file_io::BoundingBoxWithTimestampAndId> bounding_boxes_with_ids;
  for (const file_io::BoundingBoxWithTimestamp &bb : filtered_bbs_no_id) {
    file_io::BoundingBoxWithTimestampAndId bb_with_id;
    bb_with_id.min_pixel_x = bb.min_pixel_x;
    bb_with_id.min_pixel_y = bb.min_pixel_y;
    bb_with_id.max_pixel_x = bb.max_pixel_x;
    bb_with_id.max_pixel_y = bb.max_pixel_y;
    bb_with_id.seconds = bb.seconds;
    bb_with_id.nano_seconds = bb.nano_seconds;
    bb_with_id.semantic_class = bb.semantic_class;
    if (bb.camera_id != file_io::kDefaultCameraId) {
      bb_with_id.camera_id = bb.camera_id;
    } else {
      bb_with_id.camera_id = FLAGS_camera_id;
    }
    bb_with_id.ellipsoid_idx = 0;
    bounding_boxes_with_ids.emplace_back(bb_with_id);
  }

  LOG(INFO) << "Writing " << bounding_boxes_with_ids.size() << " to file ";

  file_io::writeBoundingBoxWithTimestampAndIdsToFile(
      FLAGS_bounding_boxes_with_ids_file, bounding_boxes_with_ids);

  return 0;
}