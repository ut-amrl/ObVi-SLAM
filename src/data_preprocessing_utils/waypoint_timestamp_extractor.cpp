//
// Created by amanda on 5/5/23.
//

#include <base_lib/pose_utils.h>
#include <file_io/timestamp_and_waypoint_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <refactoring/image_processing/image_processing_utils.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sys/select.h>
#include <types/timestamped_data_to_frames_utils.h>

DEFINE_string(rosbag_file,
              "",
              "ROS bag file name that contains the images for this run");
DEFINE_string(timestamps_for_waypoints_out_file,
              "",
              "File to write timestamps to");
DEFINE_string(camera_topics_list_file,
              "",
              "File that contains topics for the cameras that we can use the "
              "timestamps for");
DEFINE_string(waypoint_trigger_topic,
              "",
              "Topic that includes waypoint triggers");

using namespace pose;
using namespace vslam_types_refactor;

const uint64_t kMillisThreshold = 2000;

std::vector<std::string> readTopicsListFromFile(const std::string &file_name) {
  std::ifstream file_obj(file_name);
  std::string line;
  std::vector<std::string> topics;
  bool first_line = true;
  while (std::getline(file_obj, line)) {
    if (first_line) {
      first_line = false;
    }
    topics.emplace_back(line);
  }
  if (first_line) {
    LOG(ERROR)
        << "The file was completely empty (and likely doesn't exist). File "
        << file_name;
    exit(1);
  }
  return topics;
}

std::vector<Timestamp> getWaypointTimestampsFromFile(
    const std::string &rosbag_file, const std::string &waypoint_topic_name) {
  rosbag::Bag bag;
  bag.open(rosbag_file, rosbag::bagmode::Read);

  std::vector<std::string> waypoint_topics = {waypoint_topic_name};

  rosbag::View view(bag, rosbag::TopicQuery(waypoint_topics));

  std::vector<Timestamp> all_timestamps;
  for (rosbag::MessageInstance const &m : view) {
    std_msgs::Header::ConstPtr msg = m.instantiate<std_msgs::Header>();

    Timestamp curr_stamp = std::make_pair(msg->stamp.sec, msg->stamp.nsec);
    all_timestamps.emplace_back(curr_stamp);
  }
  bag.close();

  std::vector<Timestamp> timestamps;
  for (const Timestamp &timestamp : all_timestamps) {
    if (timestamps.empty()) {
      timestamps.emplace_back(timestamp);
    } else {
      uint64_t timestamp_millis = timestampToMillis(timestamps.back());
      uint64_t candidate_stamp_millis = timestampToMillis(timestamp);
      if ((candidate_stamp_millis - timestamp_millis) > kMillisThreshold) {
        timestamps.emplace_back(timestamp);
      } else {
        LOG(INFO) << "Skipping waypoint";
      }
    }
  }
  return timestamps;
}

std::vector<Timestamp> getImageTimestampsFromFile(
    const std::string &rosbag_file,
    const std::vector<std::string> &image_topics) {
  rosbag::Bag bag;
  bag.open(rosbag_file, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(image_topics));

  std::vector<Timestamp> timestamps;
  for (const rosbag::MessageInstance &m : view) {
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(image_utils::kCompressedImageSuffix) !=
        std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }

    Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
    timestamps.emplace_back(img_timestamp);
  }
  bag.close();

  return timestamps;
}

bool readCharFromStdIn() {
  // Stolen from rosbag player.cpp
  fd_set stdin_fdset;
  //  const int fd = fileno(stdin);

  FD_ZERO(&stdin_fdset);
  //  FD_SET(fd, &stdin_fdset);
  FD_SET(0, &stdin_fdset);

  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  if (select(1, &stdin_fdset, NULL, NULL, &tv) <= 0) {
    return false;
  }
  getc(stdin);
  return true;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "waypoint_labeler");
  ros::NodeHandle node_handle;

  CHECK(!FLAGS_waypoint_trigger_topic.empty())
      << "Waypoint topic cannot be empty";
  CHECK(!FLAGS_rosbag_file.empty()) << "Rosbag file name cannot be empty";
  CHECK(!FLAGS_timestamps_for_waypoints_out_file.empty())
      << "Output file for waypoints cannot be empty";
  CHECK(!FLAGS_camera_topics_list_file.empty())
      << "Camera topics file cannot be empty";

  // Read topics from file
  LOG(INFO) << "Reading image topics from file";
  std::vector<std::string> image_topics =
      readTopicsListFromFile(FLAGS_camera_topics_list_file);

  LOG(INFO) << "Reading image timestamps from file";
  std::vector<Timestamp> image_timestamps =
      getImageTimestampsFromFile(FLAGS_rosbag_file, image_topics);

  // Iterate through waypoint topic -- find timestamps
  LOG(INFO) << "Reading waypoint timestamps from file";
  std::vector<Timestamp> waypoint_stamps = getWaypointTimestampsFromFile(
      FLAGS_rosbag_file, FLAGS_waypoint_trigger_topic);

  // Iterate through image topics -- find timestamps closet to waypoints
  LOG(INFO) << "Finding closest timestamps to waypoints from file";
  std::vector<Timestamp> closest_timestamps;
  if (!waypoint_stamps.empty()) {
    while (timestamp_sort()(waypoint_stamps[closest_timestamps.size()],
                            image_timestamps.front())) {
      closest_timestamps.emplace_back(image_timestamps.front());
    }

    for (size_t curr_image_timestamp_idx = 1;
         curr_image_timestamp_idx < image_timestamps.size();
         curr_image_timestamp_idx++) {
      if (closest_timestamps.size() >= waypoint_stamps.size()) {
        break;
      }
      while (timestamp_sort()(waypoint_stamps[closest_timestamps.size()],
                              image_timestamps[curr_image_timestamp_idx])) {
        uint64_t prev_image_millis =
            timestampToMillis(image_timestamps[curr_image_timestamp_idx - 1]);
        uint64_t following_image_millis =
            timestampToMillis(image_timestamps[curr_image_timestamp_idx]);
        uint64_t waypoint_millis =
            timestampToMillis(waypoint_stamps[closest_timestamps.size()]);

        if ((following_image_millis - waypoint_millis) <
            (waypoint_millis - prev_image_millis)) {
          closest_timestamps.emplace_back(
              image_timestamps[curr_image_timestamp_idx]);
        } else {
          closest_timestamps.emplace_back(
              image_timestamps[curr_image_timestamp_idx - 1]);
        }

        if (closest_timestamps.size() >= waypoint_stamps.size()) {
          break;
        }
      }
    }

    while (closest_timestamps.size() < waypoint_stamps.size()) {
      closest_timestamps.emplace_back(image_timestamps.back());
    }
  }

  util::BoostHashMap<Timestamp, std::pair<WaypointId, bool>>
      labels_for_timestamps;
  util::BoostHashSet<Timestamp> timestamps_set(closest_timestamps.begin(),
                                               closest_timestamps.end());

  // Play through bag -- checking for pause or timestamp of waypoint --
  // if paused or waypoint, get waypoint number from command line
  std::unordered_map<std::string, ros::Publisher> topic_publisher_map;
  for (const std::string &image_topic : image_topics) {
    LOG(INFO) << image_topic;
    if (image_topic.find(image_utils::kCompressedImageSuffix) !=
        std::string::npos) {
      LOG(INFO) << "Compressed publisher";
      ros::Publisher pub_for_topic =
          node_handle.advertise<sensor_msgs::CompressedImage>(image_topic, 1);
      topic_publisher_map[image_topic] = pub_for_topic;
    } else {
      LOG(INFO) << "Uncompressed publisher";
      ros::Publisher pub_for_topic =
          node_handle.advertise<sensor_msgs::Image>(image_topic, 1);
      topic_publisher_map[image_topic] = pub_for_topic;
    }
  }

  ros::Duration(2).sleep();

  rosbag::Bag bag;
  bag.open(FLAGS_rosbag_file, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(image_topics));

  std::vector<Timestamp> timestamps;
  ros::Time last_time;
  bool first = true;
  for (const rosbag::MessageInstance &m : view) {
    if (!ros::ok()) {
      exit(1);
    }
    sensor_msgs::Image::ConstPtr msg;
    if (m.getTopic().find(image_utils::kCompressedImageSuffix) !=
        std::string::npos) {
      sensor_msgs::CompressedImage::ConstPtr compressed_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      image_utils::decompressImage(compressed_msg, msg);
    } else {
      msg = m.instantiate<sensor_msgs::Image>();
    }
    if (first) {
      first = false;
    } else {
      ros::Duration sleep_time = (msg->header.stamp - last_time) * 0.2;
      sleep_time.sleep();
    }
    last_time = msg->header.stamp;
    topic_publisher_map[m.getTopic()].publish(m);

    Timestamp img_timestamp =
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
    bool waitForNumber = false;
    if ((readCharFromStdIn()) ||
        (timestamps_set.find(img_timestamp) != timestamps_set.end())) {
      waitForNumber = true;
    }

    if (waitForNumber) {
      LOG(INFO)
          << "Enter waypoint number (if not number, waypoint will be skipped)";
      // Get number & reversed
      std::string line;
      std::getline(std::cin, line);

      bool numeric = all_of(line.begin(), line.end(), ::isdigit);
      if (!numeric) {
        LOG(INFO) << "Not numeric -- not treating as waypoint";
        continue;
      }
      WaypointId waypoint_id = std::stoi(line);

      LOG(INFO) << "Enter r for a reversed waypoint, any other character for "
                   "not reversed";
      std::getline(std::cin, line);
      bool reversed = false;
      if (line == "r") {
        reversed = true;
      }
      labels_for_timestamps[img_timestamp] =
          std::make_pair(waypoint_id, reversed);
    }
  }
  bag.close();

  std::vector<Timestamp> labeled_timestamps;
  for (const auto &timestamp_and_label : labels_for_timestamps) {
    labeled_timestamps.emplace_back(timestamp_and_label.first);
  }
  std::sort(
      labeled_timestamps.begin(), labeled_timestamps.end(), timestamp_sort());
  std::vector<file_io::TimestampAndWaypointInfo> timestamp_and_waypoint_infos;
  for (const Timestamp &timestamp : labeled_timestamps) {
    std::pair<WaypointId, bool> label = labels_for_timestamps.at(timestamp);
    LOG(INFO) << "Timestamp " << timestamp.first << ", " << timestamp.second
              << ", " << label.first << ", " << label.second;
    file_io::TimestampAndWaypointInfo timestamp_and_waypoint_info;
    timestamp_and_waypoint_info.waypoint_id_ = label.first;
    timestamp_and_waypoint_info.seconds_ = timestamp.first;
    timestamp_and_waypoint_info.nano_seconds_ = timestamp.second;
    timestamp_and_waypoint_info.reversed_ = label.second;
    timestamp_and_waypoint_infos.emplace_back(timestamp_and_waypoint_info);
  }

  writeTimestampAndWaypointInfosToFile(FLAGS_timestamps_for_waypoints_out_file,
                                       timestamp_and_waypoint_infos);

  return 0;
}