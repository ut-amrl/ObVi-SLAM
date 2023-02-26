//
// Created by amanda on 10/17/22.
//

#include <amrl_msgs/ObjectDetectionSrv.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui.hpp>

DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(img_topic_name, "", "");
std::string kCompressedImageSuffix = "compressed";

std::vector<sensor_msgs::Image::ConstPtr> getImagesFromRosbag(
    const std::string &rosbag_file_name, const std::string &img_topic_name) {
  // Read the images
  rosbag::Bag bag;
  bag.open(FLAGS_rosbag_file, rosbag::bagmode::Read);
  // TODO do we want to make a new back with uncompressed images or handle the
  // compression here?

  std::vector<std::string> topics = {img_topic_name};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<sensor_msgs::Image::ConstPtr> imgs;
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(kCompressedImageSuffix) != std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }
    imgs.emplace_back(msg);
  }
  return imgs;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = true;  // Don't log to disk - log to terminal

  ros::init(argc, argv, "obj_det_tester");
  ros::NodeHandle n;

  std::vector<sensor_msgs::Image::ConstPtr> images =
      getImagesFromRosbag(FLAGS_rosbag_file, FLAGS_img_topic_name);

  LOG(INFO) << "Making service client";
  ros::ServiceClient client =
      n.serviceClient<amrl_msgs::ObjectDetectionSrv>("yolov5_detect_objs");
  LOG(INFO) << "Done making service client";
//  ros::Duration(1).sleep();

  int failure_count = 0;
  int success_count = 0;
  double average_bounding_boxes_per_frame = 0;
  for (size_t i = 0; i < images.size(); i++) {
    amrl_msgs::ObjectDetectionSrv obj_det_srv_call;
    sensor_msgs::Image::ConstPtr curr_img = images[i];
    obj_det_srv_call.request.query_image = *curr_img;

    if (client.call(obj_det_srv_call)) {
      success_count++;

      cv_bridge::CvImagePtr cv_ptr;
      ros::Time image_stamp;

      try {
        cv_ptr =
            cv_bridge::toCvCopy(curr_img, sensor_msgs::image_encodings::BGR8);
        cv_ptr->header.frame_id = "img";
        image_stamp = ros::Time::now();
        cv_ptr->header.stamp = image_stamp;
      } catch (cv_bridge::Exception &e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
        exit(1);
      }
      for (size_t bb_idx = 0;
           bb_idx < obj_det_srv_call.response.bounding_boxes.bboxes.size();
           bb_idx++) {
        amrl_msgs::BBox2DMsg bb =
            obj_det_srv_call.response.bounding_boxes.bboxes[bb_idx];
        if (bb.conf >= 0.2) {
          average_bounding_boxes_per_frame++;
        }
      }
    } else {
      LOG(INFO) << "Failed";
      failure_count++;
    }
  }
  LOG(INFO) << "Successes: " << success_count
            << ", failures: " << failure_count;
  LOG(INFO) << "Num frames " << images.size();
  average_bounding_boxes_per_frame =
      average_bounding_boxes_per_frame / images.size();
  LOG(INFO) << "Average bounding boxes per frame "
            << average_bounding_boxes_per_frame;
}