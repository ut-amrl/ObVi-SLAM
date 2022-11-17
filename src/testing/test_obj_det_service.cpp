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
  ros::Publisher img_publisher = n.advertise<sensor_msgs::Image>("bb_img", 10);

  ros::ServiceClient client =
      n.serviceClient<amrl_msgs::ObjectDetectionSrv>("yolov5_detect_objs");
  ros::Duration(1).sleep();
  cv::Scalar color = CV_RGB(255, 255, 0);

  int failure_count = 0;
  int success_count = 0;
  for (size_t i = 0; i < images.size(); i += 10) {
    amrl_msgs::ObjectDetectionSrv obj_det_srv_call;
    sensor_msgs::Image::ConstPtr curr_img = images[i];
    obj_det_srv_call.request.query_image = *curr_img;

    if (client.call(obj_det_srv_call)) {
      success_count++;

      cv_bridge::CvImagePtr cv_ptr;
      ros::Time image_stamp;

      try {
        cv_ptr = cv_bridge::toCvCopy(curr_img,
                                     sensor_msgs::image_encodings::BGR8);
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
        amrl_msgs::BBox2DMsg bb = obj_det_srv_call.response.bounding_boxes.bboxes[bb_idx];
        cv::rectangle(cv_ptr->image,
                      cv::Point(bb.xyxy[0],
                                bb.xyxy[1]),
                      cv::Point(bb.xyxy[2],
                                bb.xyxy[3]),
                      color,
                      3);
      }

      img_publisher.publish(cv_ptr->toImageMsg());
    } else {
      LOG(INFO) << "Failed";
      failure_count++;
    }
  }
  LOG(INFO) << "Successes: " << success_count
            << ", failures: " << failure_count;
}