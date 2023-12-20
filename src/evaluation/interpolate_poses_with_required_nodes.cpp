//
// Created by amanda on 1/27/23.
//

#include <base_lib/basic_utils.h>
#include <base_lib/pose_utils.h>
#include <evaluation/trajectory_interpolation_utils.h>
#include <file_io/pose_3d_io.h>
#include <file_io/pose_3d_with_double_timestamp_io.h>
#include <file_io/pose_3d_with_timestamp_io.h>
#include <file_io/timestamp_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <refactoring/types/vslam_basic_types_refactor.h>
#include <refactoring/visualization/ros_visualization.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace pose;
using namespace vslam_types_refactor;

DEFINE_string(required_timestamps_file,
              "",
              "File name containing timestamps that we must get estimate for.");
DEFINE_string(
    coarse_trajectory_file,
    "",
    "File with coarse 3d trajectory to get more detailed estimates for");
DEFINE_string(rosbag_file, "", "File with rosbag containing odometry messages");
DEFINE_string(odometry_topic,
              "/husky_velocity_controller/odom",
              "Odometry topic. ");
DEFINE_string(poses_for_required_timestamps_file,
              "",
              "File to which to output the interpolated poses for the required "
              "timestamps");
DEFINE_string(poses_for_all_timestamps_file,
              "",
              "File to which to output the interpolated poses "
              "for all timestamps (i.e. also contains smoothed odom)");
DEFINE_string(
    coarse_trajectory_frame_rel_bl_file,
    "",
    "File containing the extrinsics representing the frame of the coarse poses "
    "file (one to interpolate) relative to the base link");
DEFINE_string(odom_frame_rel_bl_file,
              "",
              "File containing the extrinsics representing the frame of the "
              "odom poses file (one to interpolate) relative to the base link");
DEFINE_string(param_prefix, "", "Prefix for published topics");

struct sort_pose3d_timestamp_pair {
  inline bool operator()(const std::pair<Timestamp, Pose3D<double>> &pose_1,
                         const std::pair<Timestamp, Pose3D<double>> &pose_2) {
    return timestamp_sort()(pose_1.first, pose_2.first);
  }
};

void convertToStampAndPose3D(
    const std::vector<file_io::Pose3DWithDoubleTimestamp> &coarse_fixed_poses,
    std::vector<std::pair<Timestamp, Pose3D<double>>>
        &output_poses_with_timestamps) {
  for (const file_io::Pose3DWithDoubleTimestamp &input_pose :
       coarse_fixed_poses) {
    ros::Time node_time(input_pose.timestamp_);
    output_poses_with_timestamps.emplace_back(std::make_pair(
        std::make_pair(node_time.sec, node_time.nsec),
        Pose3D<double>(
            Position3d<double>(input_pose.transl_x_,
                               input_pose.transl_y_,
                               input_pose.transl_z_),
            Orientation3D<double>(Eigen::Quaterniond(input_pose.quat_w_,
                                                     input_pose.quat_x_,
                                                     input_pose.quat_y_,
                                                     input_pose.quat_z_)))));
  }
  std::sort(output_poses_with_timestamps.begin(),
            output_poses_with_timestamps.end(),
            sort_pose3d_timestamp_pair());
}

void visualizePoseGraph(
    const std::shared_ptr<RosVisualization> &vis_manager,
    const util::BoostHashMap<pose::Timestamp, Pose3D<double>>
        &raw_poses_by_timestamp,
    const std::vector<RelativePoseFactorInfo> &residuals_info,
    ros::Publisher &pub) {
  if (vis_manager == nullptr) {
    return;
  }
  std_msgs::ColorRGBA color;
  color.b = 1;
  color.r = 1;
  color.a = 1;
  std::vector<std::pair<Position3d<double>, Position3d<double>>>
      residuals_lines;
  for (const RelativePoseFactorInfo &relative_pose_info : residuals_info) {
    Pose3D<double> raw_pose_1 =
        raw_poses_by_timestamp.at(relative_pose_info.before_pose_timestamp_);
    Pose3D<double> raw_pose_2 =
        raw_poses_by_timestamp.at(relative_pose_info.after_pose_timestamp_);
    residuals_lines.emplace_back(
        std::make_pair(raw_pose_1.transl_, raw_pose_2.transl_));
  }

  vis_manager->publishLines(pub, color, residuals_lines, 1);
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_rosbag_file.empty()) {
    LOG(ERROR) << "Rosbag file required but not specified";
    exit(1);
  }

  if (FLAGS_coarse_trajectory_file.empty()) {
    LOG(ERROR) << "Coarse trajectory file required but not specified";
    exit(1);
  }

  if (FLAGS_required_timestamps_file.empty()) {
    LOG(ERROR) << "Required timestamps file required but not specified";
    exit(1);
  }

  if (FLAGS_poses_for_required_timestamps_file.empty()) {
    LOG(ERROR) << "Output file name for poses for the required timestamps file "
                  "required but not specified";
    exit(1);
  }

  if (FLAGS_odom_frame_rel_bl_file.empty()) {
    LOG(ERROR) << "Odometry frame relative to base link file"
                  "required but not specified";
    exit(1);
  }

  if (FLAGS_odom_frame_rel_bl_file.empty()) {
    LOG(ERROR) << "Coarse trajectory frame relative to base link file"
                  "required but not specified";
    exit(1);
  }

  LOG(INFO) << "Reading extrinsics from " << FLAGS_odom_frame_rel_bl_file;
  Pose3D<double> odom_frame_rel_base_link;
  std::vector<Pose3D<double>> odom_extrinsics_contents;
  file_io::readPose3dsFromFile(FLAGS_odom_frame_rel_bl_file,
                               odom_extrinsics_contents);
  LOG(INFO) << "Got raw conent from file ";
  if (odom_extrinsics_contents.empty()) {
    LOG(ERROR) << "Odom extrinsics missing";
    exit(1);
  }
  if (odom_extrinsics_contents.size() > 1) {
    LOG(WARNING) << "Odom extrinsics file contained more than "
                    "one pose. Taking the first";
  }

  odom_frame_rel_base_link = odom_extrinsics_contents.front();
  LOG(INFO) << "Done reading odom extrincis";

  Pose3D<double> coarse_traj_frame_rel_base_link;
  std::vector<Pose3D<double>> coarse_traj_extrinsics_contents;
  file_io::readPose3dsFromFile(FLAGS_coarse_trajectory_frame_rel_bl_file,
                               coarse_traj_extrinsics_contents);
  if (coarse_traj_extrinsics_contents.empty()) {
    LOG(ERROR) << "Coarse trajectory extrinsics missing";
    exit(1);
  }
  if (coarse_traj_extrinsics_contents.size() > 1) {
    LOG(WARNING) << "Coarse trajectory extrinsics file contained more than "
                    "one pose. Taking the first";
  }

  coarse_traj_frame_rel_base_link = coarse_traj_extrinsics_contents.front();

  LOG(INFO) << "Done reading extrinsics";
  std::vector<std::pair<Timestamp, Pose2d>> odom_poses;
  // getOdomPoseEsts(FLAGS_rosbag_file, FLAGS_odometry_topic, odom_poses);
  getOdomPoseEstsFromFile(FLAGS_rosbag_file, odom_poses);

  std::vector<file_io::Pose3DWithDoubleTimestamp> coarse_fixed_poses_raw;
  readPose3DWithDoubleTimestampFromFile(FLAGS_coarse_trajectory_file,
                                        coarse_fixed_poses_raw);

  std::vector<std::pair<Timestamp, Pose3D<double>>> coarse_fixed_poses;
  convertToStampAndPose3D(coarse_fixed_poses_raw, coarse_fixed_poses);
  std::vector<Pose3D<double>> coarse_fixed_poses_no_stamp;
  for (const std::pair<Timestamp, Pose3D<double>> &fixed_pose :
       coarse_fixed_poses) {
    coarse_fixed_poses_no_stamp.emplace_back(fixed_pose.second);
  }
  LOG(INFO) << "Done getting fixed poses";

  Pose3D<double> lidar_rel_odom = combinePoses(
      poseInverse(odom_frame_rel_base_link), coarse_traj_frame_rel_base_link);
  std::vector<std::pair<Timestamp, Pose3D<double>>> coarse_fixed_poses_rel_odom;
  std::vector<Pose3D<double>> coarse_traj;

  for (const std::pair<Timestamp, Pose3D<double>> &coarse_pose :
       coarse_fixed_poses) {
    coarse_traj.emplace_back(coarse_pose.second);
  }
  std::vector<Pose3D<double>> coarse_traj_rel_odom_frame =
      adjustTrajectoryToStartAtOriginWithExtrinsics(
          coarse_traj, coarse_traj.front(), lidar_rel_odom);
  for (size_t pose_num = 0; pose_num < coarse_traj_rel_odom_frame.size();
       pose_num++) {
    coarse_fixed_poses_rel_odom.emplace_back(std::make_pair(
        coarse_fixed_poses.at(pose_num).first, coarse_traj.at(pose_num)));
  }

  LOG(INFO) << "Reading required timestamps";
  std::vector<Timestamp> required_timestamps;
  file_io::readTimestampsFromFile(FLAGS_required_timestamps_file,
                                  required_timestamps);
  std::sort(
      required_timestamps.begin(), required_timestamps.end(), timestamp_sort());

  std::vector<Timestamp> all_sorted_timestamps;
  all_sorted_timestamps.insert(all_sorted_timestamps.end(),
                               required_timestamps.begin(),
                               required_timestamps.end());
  for (const std::pair<Timestamp, Pose2d> &odom_stamp_and_pose : odom_poses) {
    all_sorted_timestamps.emplace_back(odom_stamp_and_pose.first);
  }
  for (const std::pair<Timestamp, Pose3D<double>> &coarse_fixed_pose :
       coarse_fixed_poses) {
    all_sorted_timestamps.emplace_back(coarse_fixed_pose.first);
  }
  std::sort(all_sorted_timestamps.begin(),
            all_sorted_timestamps.end(),
            timestamp_sort());

  LOG(INFO) << "Setting up ros stuff";

  std::string param_prefix = FLAGS_param_prefix;
  std::string node_prefix = FLAGS_param_prefix;
  if (!param_prefix.empty()) {
    param_prefix = "/" + param_prefix + "/";
    node_prefix += "_";
  }

  ros::init(argc, argv, "a_" + node_prefix + "interpolator");
  ros::NodeHandle node_handle;
  std::shared_ptr<RosVisualization> vis_manager =
      std::make_shared<RosVisualization>(node_handle, param_prefix, node_prefix);
  ros::Publisher pub = node_handle.advertise<visualization_msgs::Marker>(
      "/pose_graph_constraints", 5000);

  util::BoostHashMap<pose::Timestamp, Pose3D<double>> odom_poses_adjusted_3d;
  util::BoostHashMap<pose::Timestamp, Pose3D<double>>
      interpolated_poses_rel_odom;

  std::function<void(
      const util::BoostHashMap<pose::Timestamp, Pose3D<double>> &,
      const std::vector<RelativePoseFactorInfo> &)>
      vis_function =
          [&](const util::BoostHashMap<pose::Timestamp, Pose3D<double>> &poses,
              const std::vector<RelativePoseFactorInfo> &factors) {
            std::vector<Pose3D<double>> req_poses;
            for (const Timestamp &required_timestamp : required_timestamps) {
              req_poses.emplace_back(poses.at(required_timestamp));
            }
            vis_manager->visualizeTrajectory(coarse_fixed_poses_no_stamp,
                                             GROUND_TRUTH);
            vis_manager->visualizeTrajectory(req_poses, ESTIMATED);
            visualizePoseGraph(vis_manager, poses, factors, pub);
          };
  LOG(INFO) << "Starting interpolation";
  interpolate3dPosesUsingOdom(odom_poses,
                              coarse_fixed_poses_rel_odom,
                              required_timestamps,
                              vis_function,
                              interpolated_poses_rel_odom,
                              odom_poses_adjusted_3d);
  LOG(INFO) << "Finished interpoolation";

  std::vector<std::pair<Timestamp, Pose3D<double>>> all_poses_with_timestamps;
  std::vector<Pose3D<double>> all_poses;
  std::vector<std::pair<Timestamp, Pose3D<double>>>
      poses_for_required_timestamp;

  Pose3D<double> earliest_interp_pose =
      interpolated_poses_rel_odom[all_sorted_timestamps.front()];

  util::BoostHashMap<pose::Timestamp, Pose3D<double>> interpolated_poses =
      adjustTrajectoryToStartAtOriginWithExtrinsics(
          interpolated_poses_rel_odom,
          earliest_interp_pose,
          poseInverse(lidar_rel_odom));

  for (const Timestamp &timestamp : all_sorted_timestamps) {
    all_poses_with_timestamps.emplace_back(
        std::make_pair(timestamp, interpolated_poses.at(timestamp)));
    all_poses.emplace_back(interpolated_poses.at(timestamp));
  }
  for (const Timestamp &required_timestamp : required_timestamps) {
    poses_for_required_timestamp.emplace_back(std::make_pair(
        required_timestamp, interpolated_poses.at(required_timestamp)));
  }

  std::vector<Pose3D<double>> adjusted_odom_poses;
  for (const std::pair<Timestamp, Pose2d> &orig_odom_stamp_and_pose :
       odom_poses) {
    adjusted_odom_poses.emplace_back(
        odom_poses_adjusted_3d.at(orig_odom_stamp_and_pose.first));
  }

  file_io::writePose3dsWithTimestampToFile(
      FLAGS_poses_for_required_timestamps_file, poses_for_required_timestamp);

  if (!FLAGS_poses_for_all_timestamps_file.empty()) {
    file_io::writePose3dsWithTimestampToFile(
        FLAGS_poses_for_all_timestamps_file, all_poses_with_timestamps);
  }
}