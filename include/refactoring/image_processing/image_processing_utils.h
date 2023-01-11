//
// Created by amanda on 9/30/22.
//

#ifndef UT_VSLAM_IMAGE_PROCESSING_UTILS_H
#define UT_VSLAM_IMAGE_PROCESSING_UTILS_H

#include <base_lib/basic_utils.h>
#include <base_lib/pose_reps.h>
#include <base_lib/pose_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_utils {

namespace enc = sensor_msgs::image_encodings;

static const cv::ImreadModes kImageDecodeFlag = cv::IMREAD_UNCHANGED;

static const std::string kCompressedImageSuffix = "compressed";
void decompressImage(const sensor_msgs::CompressedImageConstPtr &message,
                     sensor_msgs::ImageConstPtr &output_img)

{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try {
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), kImageDecodeFlag);

    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    if (split_pos == std::string::npos) {
      // Older version of compressed_image_transport does not signal image
      // format
      switch (cv_ptr->image.channels()) {
        case 1:
          cv_ptr->encoding = enc::MONO8;
          break;
        case 3:
          cv_ptr->encoding = enc::BGR8;
          break;
        default:
          ROS_ERROR("Unsupported number of channels: %i",
                    cv_ptr->image.channels());
          break;
      }
    } else {
      std::string image_encoding = message->format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if (enc::isColor(image_encoding)) {
        std::string compressed_encoding = message->format.substr(split_pos);
        bool compressed_bgr_image =
            (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
        } else {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
        }
      }
    }
  } catch (cv::Exception &e) {
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    // Publish message to user callback
    output_img = (cv_ptr->toImageMsg());
}

std::unordered_map<vslam_types_refactor::FrameId,
                   std::unordered_map<vslam_types_refactor::CameraId,
                                      sensor_msgs::Image::ConstPtr>>
getImagesFromRosbag(
    const std::string &rosbag_file_name,
    const std::string &nodes_by_timestamp_file,
    const std::unordered_map<std::string, vslam_types_refactor::CameraId>
        &camera_topic_to_camera_id) {
  std::vector<file_io::NodeIdAndTimestamp> nodes_by_timestamps_vec;
  util::BoostHashMap<pose::Timestamp, vslam_types_refactor::FrameId>
      nodes_for_timestamps_map;
  file_io::readNodeIdsAndTimestampsFromFile(nodes_by_timestamp_file,
                                            nodes_by_timestamps_vec);

  for (const file_io::NodeIdAndTimestamp &raw_node_id_and_timestamp :
       nodes_by_timestamps_vec) {
    nodes_for_timestamps_map[std::make_pair(
        raw_node_id_and_timestamp.seconds_,
        raw_node_id_and_timestamp.nano_seconds_)] =
        raw_node_id_and_timestamp.node_id_;
  }

  // Read the images
  rosbag::Bag bag;
  bag.open(rosbag_file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  for (const auto &camera_topic_and_id : camera_topic_to_camera_id) {
    topics.emplace_back(camera_topic_and_id.first);
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::unordered_map<vslam_types_refactor::FrameId,
                     std::unordered_map<vslam_types_refactor::CameraId,
                                        sensor_msgs::Image::ConstPtr>>
      images_by_frame_and_cam;
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(kCompressedImageSuffix) != std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }

    pose::Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
    if (nodes_for_timestamps_map.find(img_timestamp) !=
        nodes_for_timestamps_map.end()) {
      vslam_types_refactor::CameraId cam =
          camera_topic_to_camera_id.at(m.getTopic());
      vslam_types_refactor::FrameId frame_id =
          nodes_for_timestamps_map[img_timestamp];
      images_by_frame_and_cam[frame_id][cam] = msg;
    }
  }
  return images_by_frame_and_cam;
}

}  // namespace image_utils

#endif  // UT_VSLAM_UTILS_H
