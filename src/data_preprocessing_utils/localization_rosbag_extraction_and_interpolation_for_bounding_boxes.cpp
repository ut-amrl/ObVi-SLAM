#include <amrl_msgs/Localization2DMsg.h>
#include <base_lib/pose_reps.h>
#include <base_lib/pose_utils.h>
#include <file_io/bounding_box_by_node_id_io.h>
#include <file_io/bounding_box_by_timestamp_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/pose_3d_with_node_id_io.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <unordered_map>
#include <unordered_set>

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_string(rosbag_file_name,
              "",
              "Rosbag file name containing localization/odometry");
DEFINE_string(
    localization_topic,
    "/Cobot/AmrlLocalization",
    "Topic name for localization (assumes amrl_msgs/Localization2DMsg");
DEFINE_string(bb_by_timestamp_file_with_association,
              "",
              "File name that contains bounding boxes by timestamp (already "
              "data associated)");
DEFINE_string(
    bb_by_timestamp_file_no_association,
    "",
    "File name that contains bounding boxes by timestamp (not associated)");
DEFINE_string(bb_by_node_out_file,
              "",
              "File name to write bounding boxes by node id");
DEFINE_string(
    localization_est_out_file,
    "",
    "File name to write localization (expressed as 3D poses) by node id");
DEFINE_string(timestamp_by_node_id_file,
              "",
              "File name to write node id to timestamp correspondence");
DEFINE_bool(merge_deduped_poses,
            false,
            "True if deduped poses should have their bbs merged; false if the "
            "dupes should just be dropped");

const double kMaxPoseIncThresholdTransl = 1.0;
const double kMaxPoseIncThresholdRot = 0.25;  // TODO?

// const double kPoseEquivTolerance = 1e-3;

// const double kPoseEquivToleranceTransl = 0.2;
// const double kPoseEquivToleranceRot = 0.2;
//const double kPoseEquivToleranceTransl = 2e-3;
//const double kPoseEquivToleranceRot = 2e-3;
const double kPoseEquivToleranceTransl = 0.1;
const double kPoseEquivToleranceRot = 0.1;

// const double kPoseEquivTolerance = 0;

using namespace pose;

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(std::pair<T1, T2> const &pair) const {
    std::size_t h1 = std::hash<T1>()(pair.first);
    std::size_t h2 = std::hash<T2>()(pair.second);

    return h1 ^ h2;
  }
};

struct timestamp_sort {
  inline bool operator()(const Timestamp &timestamp1,
                         const Timestamp &timestamp2) {
    if (timestamp1.first != timestamp2.first) {
      return timestamp1.first < timestamp2.first;
    }
    return timestamp1.second <= timestamp2.second;
  }
};

template <typename BbByTimestampType>

void readBbsAndFilterBySemanticClass(
    const std::string &bb_file_name,
    const std::unordered_set<std::string> &acceptable_semantic_classes,
    const std::function<void(const std::string &,
                             std::vector<BbByTimestampType> &)> &bb_reader,
    std::vector<BbByTimestampType> &bb_out) {
  std::vector<BbByTimestampType> unfiltered_bbs;
  bb_reader(bb_file_name, unfiltered_bbs);
  for (const BbByTimestampType &bb_by_timestamp : unfiltered_bbs) {
    if (acceptable_semantic_classes.find(bb_by_timestamp.semantic_class) !=
        acceptable_semantic_classes.end()) {
      bb_out.emplace_back(bb_by_timestamp);
    }
  }
}

template <typename BbByTimestampType, typename BbByNodeType>
void interpolateTimestamps(
    const std::string &bb_by_timestamp_file_name,
    const std::function<void(const std::string &,
                             std::vector<BbByTimestampType> &)>
        &bb_by_timestamp_reader,
    const std::string &bb_by_node_file_name,
    const std::function<BbByNodeType(
        const BbByTimestampType &, const uint64_t &node_id)> bb_by_node_creator,
    const std::function<void(const std::string &,
                             const std::vector<BbByNodeType> &)>
        &bb_out_writer) {
  rosbag::Bag bag;
  bag.open(FLAGS_rosbag_file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics = {FLAGS_localization_topic};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<pose::Pose2d> full_odom_frame_poses;
  std::vector<Timestamp> full_timestamps;

  for (rosbag::MessageInstance const &m : view) {
    amrl_msgs::Localization2DMsg::ConstPtr msg =
        m.instantiate<amrl_msgs::Localization2DMsg>();
    if (!full_timestamps.empty()) {
      Timestamp prev_stamp = full_timestamps.back();
      if ((prev_stamp.first > msg->header.stamp.sec) ||
          ((prev_stamp.first == msg->header.stamp.sec) &&
           (prev_stamp.second > msg->header.stamp.nsec))) {
        LOG(INFO) << "Out of order messages!";
      }
    }
    full_timestamps.emplace_back(
        std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec));

    pose::Pose2d pose = pose::createPose2d(
        (double)msg->pose.x, (double)msg->pose.y, (double)msg->pose.theta);
    full_odom_frame_poses.emplace_back(pose);
  }

  LOG(INFO) << "Min odom timestamp " << full_timestamps.front().first << ", "
            << full_timestamps.front().second;
  LOG(INFO) << "Max odom timestamp " << full_timestamps.back().first << ", "
            << full_timestamps.back().second;

  std::unordered_set<Timestamp, pair_hash> bounding_boxes_timestamp_set;
  std::vector<BbByTimestampType> bounding_boxes_by_timestamp;
  bb_by_timestamp_reader(bb_by_timestamp_file_name,
                         bounding_boxes_by_timestamp);

  for (const BbByTimestampType &bounding_box_with_timestamp :
       bounding_boxes_by_timestamp) {
    bounding_boxes_timestamp_set.insert(
        std::make_pair(bounding_box_with_timestamp.seconds,
                       bounding_box_with_timestamp.nano_seconds));
  }
  // This assumes the file is read in time order
  Timestamp first_bb_timestamp =
      std::make_pair(bounding_boxes_by_timestamp.front().seconds,
                     bounding_boxes_by_timestamp.front().nano_seconds);
  Timestamp last_bb_timestamp =
      std::make_pair(bounding_boxes_by_timestamp.back().seconds,
                     bounding_boxes_by_timestamp.back().nano_seconds);
  if (!timestamp_sort()(full_timestamps.front(), first_bb_timestamp)) {
    LOG(INFO) << "The first odom timestamp is greater than the first "
                 "bb timestamp";
  }
  if (!timestamp_sort()(last_bb_timestamp, full_timestamps.back())) {
    LOG(INFO) << "Last bb timestamp is not less than or equal to "
                 "the odom timestamp";
  }
  std::vector<Timestamp> sorted_bb_timestamps;
  sorted_bb_timestamps.insert(sorted_bb_timestamps.end(),
                              bounding_boxes_timestamp_set.begin(),
                              bounding_boxes_timestamp_set.end());
  std::sort(sorted_bb_timestamps.begin(),
            sorted_bb_timestamps.end(),
            timestamp_sort());

  LOG(INFO) << "Min bb timestamp " << sorted_bb_timestamps.front().first << ", "
            << sorted_bb_timestamps.front().second;
  LOG(INFO) << "Max bb timestamp " << sorted_bb_timestamps.back().first << ", "
            << sorted_bb_timestamps.back().second;
  std::vector<pose::Pose2d> poses_to_use;
  std::vector<Timestamp> timestamps_to_use;

  poses_to_use.emplace_back(full_odom_frame_poses[0]);

  timestamps_to_use.emplace_back(full_timestamps[0]);
  LOG(INFO) << "Added first timestamp " << full_timestamps[0];

  size_t index_next_semantic_point_timestamp_to_check = 0;
  if (!sorted_bb_timestamps.empty()) {
    // If the first full timestamp is not less than or equal to the first sorted
    // timestamp, need to start with the next detection/waypoint timestamp
    if (!timestamp_sort()(full_timestamps[0], sorted_bb_timestamps[0])) {
      LOG(INFO) << "Skipping first sorted timestamp";
      index_next_semantic_point_timestamp_to_check = 1;
    }
  }
  for (size_t i = 1; i < full_timestamps.size(); i++) {
    bool added_pose = false;

    pose::Pose2d curr_pose = full_odom_frame_poses[i];
    Timestamp curr_timestamp = full_timestamps[i];
    bool keep_checking = true;
    while (keep_checking) {
      if (index_next_semantic_point_timestamp_to_check <
          sorted_bb_timestamps.size()) {
        Timestamp next_semantic_point_timestamp =
            sorted_bb_timestamps[index_next_semantic_point_timestamp_to_check];
        if (timestamp_sort()(next_semantic_point_timestamp, curr_timestamp)) {
          if ((next_semantic_point_timestamp.first == curr_timestamp.first) &&
              (next_semantic_point_timestamp.second == curr_timestamp.second)) {
            timestamps_to_use.emplace_back(curr_timestamp);
            poses_to_use.emplace_back(full_odom_frame_poses[i]);
          } else {
            pose::Pose2d prev_pose = full_odom_frame_poses[i - 1];
            Timestamp prev_timestamp = full_timestamps[i - 1];

            pose::Pose2d rel_pose_interp_global = pose::interpolatePoses(
                std::make_pair(prev_timestamp, prev_pose),
                std::make_pair(curr_timestamp, curr_pose),
                next_semantic_point_timestamp);
            if (timestamp_sort()(next_semantic_point_timestamp,
                                 prev_timestamp)) {
              LOG(ERROR) << "Out of order timestamps";
              exit(1);
            }
            timestamps_to_use.emplace_back(next_semantic_point_timestamp);
            poses_to_use.emplace_back(rel_pose_interp_global);
          }
          added_pose = true;
          index_next_semantic_point_timestamp_to_check++;
        } else {
          keep_checking = false;
        }
      } else {
        keep_checking = false;
      }
    }
    if (!added_pose) {
      pose::Pose2d last_added_pose = poses_to_use.back();
      pose::Pose2d rel_pose =
          pose::getPoseOfObj1RelToObj2(curr_pose, last_added_pose);
      if ((rel_pose.first.norm() > kMaxPoseIncThresholdTransl) ||
          (rel_pose.second > kMaxPoseIncThresholdRot)) {
        poses_to_use.emplace_back(curr_pose);
        timestamps_to_use.emplace_back(curr_timestamp);
      }
    }
  }
  pose::Pose2d last_pose_in_used_list = poses_to_use.back();
  pose::Pose2d last_pose = full_odom_frame_poses.back();
  if (!pose::posesAlmostSame(last_pose,
                             last_pose_in_used_list,
                             kPoseEquivToleranceTransl,
                             kPoseEquivToleranceRot)) {
    poses_to_use.emplace_back(last_pose);
  } else {
    LOG(INFO) << "Deduped";
  }
  std::vector<pose::Pose2d> poses_rel_to_origin;
  poses_rel_to_origin.emplace_back(pose::createPose2d(0, 0, 0));
  pose::Pose2d first_pose = poses_to_use[0];
  for (size_t i = 1; i < poses_to_use.size(); i++) {
    pose::Pose2d curr_pose = poses_to_use[i];
    poses_rel_to_origin.emplace_back(
        pose::getPoseOfObj1RelToObj2(curr_pose, first_pose));
  }

  std::unordered_map<Timestamp, uint64_t, pair_hash> nodes_by_timestamp;
  std::vector<pose::Pose2d> deduped_poses_rel_to_origin;
  std::vector<file_io::NodeIdAndTimestamp> nodes_with_timestamps;
  file_io::NodeIdAndTimestamp first_node;
  first_node.node_id_ = 0;
  first_node.seconds_ = timestamps_to_use[0].first;
  first_node.nano_seconds_ = timestamps_to_use[0].second;
  nodes_with_timestamps.emplace_back(first_node);
  nodes_by_timestamp[timestamps_to_use[0]] = 0;
  deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[0]);
  for (size_t i = 1; i < poses_rel_to_origin.size(); i++) {
    //        if (!pose::posesSame(poses_rel_to_origin[i - 1],
    //        poses_rel_to_origin[i])) {
    if (!pose::posesAlmostSame(deduped_poses_rel_to_origin.back(),
                               poses_rel_to_origin[i],
                               kPoseEquivToleranceTransl,
                               kPoseEquivToleranceRot)) {
      deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[i]);

      if (!FLAGS_merge_deduped_poses) {
        file_io::NodeIdAndTimestamp node_with_timestamp;
        node_with_timestamp.node_id_ = deduped_poses_rel_to_origin.size() - 1;
        node_with_timestamp.seconds_ = timestamps_to_use[i].first;
        node_with_timestamp.nano_seconds_ = timestamps_to_use[i].second;
        nodes_with_timestamps.emplace_back(node_with_timestamp);
        nodes_by_timestamp[timestamps_to_use[i]] = node_with_timestamp.node_id_;
      }
    }
    if (FLAGS_merge_deduped_poses) {
      file_io::NodeIdAndTimestamp node_with_timestamp;
      node_with_timestamp.node_id_ = deduped_poses_rel_to_origin.size() - 1;
      node_with_timestamp.seconds_ = timestamps_to_use[i].first;
      node_with_timestamp.nano_seconds_ = timestamps_to_use[i].second;
      nodes_with_timestamps.emplace_back(node_with_timestamp);
      nodes_by_timestamp[timestamps_to_use[i]] = node_with_timestamp.node_id_;
    }
  }

  bag.close();

  // Write bounding boxes by node id to file
  std::vector<BbByNodeType> bounding_boxes_with_node_id;
  for (const BbByTimestampType &bounding_box : bounding_boxes_by_timestamp) {
    Timestamp bb_timestamp =
        std::make_pair(bounding_box.seconds, bounding_box.nano_seconds);
    if (nodes_by_timestamp.find(bb_timestamp) != nodes_by_timestamp.end()) {
      bounding_boxes_with_node_id.emplace_back(bb_by_node_creator(
          bounding_box, nodes_by_timestamp.at(bb_timestamp)));
    }
  }
  bb_out_writer(bb_by_node_file_name, bounding_boxes_with_node_id);

  LOG(INFO) << "Trajectory nodes size " << poses_rel_to_origin.size();
  std::vector<std::pair<uint64_t, pose::Pose3d>> pose_3ds_with_nodes;
  for (size_t node_num = 0; node_num < deduped_poses_rel_to_origin.size();
       node_num++) {
    pose_3ds_with_nodes.emplace_back(std::make_pair(
        node_num, pose::toPose3d(deduped_poses_rel_to_origin[node_num])));
  }
  LOG(INFO) << "Num nodes " << pose_3ds_with_nodes.size();

  // Output poses
  LOG(INFO) << "Outputting trajectory nodes to "
            << FLAGS_localization_est_out_file;
  file_io::writePose3dsWithNodeIdToFile(FLAGS_localization_est_out_file,
                                        pose_3ds_with_nodes);

  // Output timestamps
  file_io::writeNodeIdsAndTimestampsToFile(FLAGS_timestamp_by_node_id_file,
                                           nodes_with_timestamps);

  if (nodes_with_timestamps.size() != poses_rel_to_origin.size()) {
    LOG(INFO) << "Number of nodes doesn't match! Num timestamps: "
              << nodes_with_timestamps.size() << ", num poses "
              << poses_rel_to_origin.size();
  }
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }
  LOG(INFO) << "Prefix: " << param_prefix;

  ros::init(
      argc, argv, node_prefix + "wheel_odom_rosbag_extractor_semantic_points");
  ros::NodeHandle n;

  if (FLAGS_rosbag_file_name.empty()) {
    LOG(INFO) << "No rosbag file provided for localization estimates ";
    exit(1);
  }

  if (FLAGS_bb_by_timestamp_file_with_association.empty() &&
      FLAGS_bb_by_timestamp_file_no_association.empty()) {
    LOG(INFO) << "No bounding box by timestamp file provided";
    exit(1);
  }

  if (FLAGS_bb_by_node_out_file.empty()) {
    LOG(INFO) << "No bounding box by node file provided";
    exit(1);
  }

  if (FLAGS_localization_est_out_file.empty()) {
    LOG(INFO) << "No bounding box by node file provided";
    exit(1);
  }

  if (FLAGS_timestamp_by_node_id_file.empty()) {
    LOG(INFO) << "No bounding box by node file provided";
    exit(1);
  }

  if (!FLAGS_bb_by_timestamp_file_with_association.empty()) {
    LOG(INFO) << "Getting timestamps from file "
              << FLAGS_bb_by_timestamp_file_with_association;
    std::function<file_io::BoundingBoxWithNodeIdAndId(
        const file_io::BoundingBoxWithTimestampAndId &,
        const uint64_t &node_id)>
        bb_by_node_creator =
            [](const file_io::BoundingBoxWithTimestampAndId &bb_with_timestamp,
               const uint64_t &node_id) {
              file_io::BoundingBoxWithNodeIdAndId bb_with_node;
              bb_with_node.semantic_class = bb_with_timestamp.semantic_class;
              bb_with_node.min_pixel_x = bb_with_timestamp.min_pixel_x;
              bb_with_node.min_pixel_y = bb_with_timestamp.min_pixel_y;
              bb_with_node.max_pixel_x = bb_with_timestamp.max_pixel_x;
              bb_with_node.max_pixel_y = bb_with_timestamp.max_pixel_y;
              bb_with_node.ellipsoid_idx = bb_with_timestamp.ellipsoid_idx;
              bb_with_node.camera_id = bb_with_timestamp.camera_id;
              bb_with_node.detection_confidence =
                  bb_with_timestamp.detection_confidence;
              bb_with_node.node_id = node_id;
              return bb_with_node;
            };
    interpolateTimestamps<file_io::BoundingBoxWithTimestampAndId,
                          file_io::BoundingBoxWithNodeIdAndId>(
        FLAGS_bb_by_timestamp_file_with_association,
        file_io::readBoundingBoxWithTimestampAndIdsFromFile,
        FLAGS_bb_by_node_out_file,
        bb_by_node_creator,
        file_io::writeBoundingBoxWithNodeIdAndIdsToFile);
  } else {
    LOG(INFO) << "Getting timestamps from file "
              << FLAGS_bb_by_timestamp_file_no_association;
    std::function<file_io::BoundingBoxWithNodeId(
        const file_io::BoundingBoxWithTimestamp &, const uint64_t &node_id)>
        bb_by_node_creator =
            [](const file_io::BoundingBoxWithTimestamp &bb_with_timestamp,
               const uint64_t &node_id) {
              file_io::BoundingBoxWithNodeId bb_with_node;
              bb_with_node.semantic_class = bb_with_timestamp.semantic_class;
              bb_with_node.min_pixel_x = bb_with_timestamp.min_pixel_x;
              bb_with_node.min_pixel_y = bb_with_timestamp.min_pixel_y;
              bb_with_node.max_pixel_x = bb_with_timestamp.max_pixel_x;
              bb_with_node.max_pixel_y = bb_with_timestamp.max_pixel_y;
              bb_with_node.camera_id = bb_with_timestamp.camera_id;
              bb_with_node.detection_confidence =
                  bb_with_timestamp.detection_confidence;
              bb_with_node.node_id = node_id;
              return bb_with_node;
            };
    std::unordered_set<std::string> acceptable_classes = {"chair", "roadblock"};
    std::function<void(const std::string &,
                       std::vector<file_io::BoundingBoxWithTimestamp> &)>
        bb_reader = [&](const std::string &bb_file_name,
                        std::vector<file_io::BoundingBoxWithTimestamp>
                            &bb_out) {
          readBbsAndFilterBySemanticClass<file_io::BoundingBoxWithTimestamp>(
              bb_file_name,
              acceptable_classes,
              file_io::readBoundingBoxWithTimestampsFromFile,
              bb_out);
        };
    interpolateTimestamps<file_io::BoundingBoxWithTimestamp,
                          file_io::BoundingBoxWithNodeId>(
        FLAGS_bb_by_timestamp_file_no_association,
        bb_reader,
        FLAGS_bb_by_node_out_file,
        bb_by_node_creator,
        file_io::writeBoundingBoxesWithNodeIdToFile);
  }
  return 0;
}