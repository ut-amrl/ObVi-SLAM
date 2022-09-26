#ifndef IV_BBOX_H
#define IV_BBOX_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <amrl_msgs/BBox2DArrayMsg.h>
#include <cmath>
#include <iv_bbox/iv_bbox_types.h>

namespace iv_bbox {

using namespace vslam_types_refactor;
using std::string;
using std::vector;
using std::unordered_map;
using std::pair;
using std::cout;
using std::endl;

/**
 * @brief Debugging; TODO delete me
 * 
 */
template <typename T>
void to_csv(const string& filename, const Pose3DArr<T>& stampedPoses) {
  std::ofstream ofile;
  ofile.open(filename, std::ios::trunc);
  if (!ofile.is_open()) {
      LOG(ERROR) << "failed to open " << filename;
  }
  for (const auto& stampedPose : stampedPoses) {
    const auto& posePtr = stampedPose.second;
    ofile << posePtr->transl_[0] << "," << posePtr->transl_[1] << "," << posePtr->transl_[2] << endl;
  }
  ofile.close();
}

// http://planning.cs.uiuc.edu/node103.html
void R_to_roll_pitch_yaw(const Eigen::Matrix3f R, 
                         float& roll, float& pitch, float& yaw) {
  yaw  = atan2( R(1,0), R(0,0));
  pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  roll   = atan2( R(2,1), R(2,2));
}

template <typename T>
Eigen::Matrix<T, 4, 1> getMeasurementError(const Eigen::Matrix<T, 4, 1>& measurement, 
                                           const Eigen::Matrix<T, 4, 1>& gt) {
  return measurement - gt;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 4> getCovMat(const Eigen::Matrix<T, Eigen::Dynamic, 4>& X) {
  float n = X.rows();
  return 1/(n-1) * X.transpose() * X;
}

float getNNL() { // TODO
  return 0.0;
}

void openBagfile(const string& bagfile_path, rosbag::Bag& bag) {
  bag.open(bagfile_path, rosbag::bagmode::Read);
}

void closeBagfile(rosbag::Bag& bag) {bag.close();}

template <typename topic_type>
void loadTopic(const rosbag::Bag& bag, const string& topic_name,
                 vector<topic_type>& messages) {
  rosbag::View view(bag, rosbag::TopicQuery(topic_name));
  for (const rosbag::MessageInstance& m : view) {
    typename topic_type::ConstPtr ptr = m.instantiate<topic_type>();
    if (ptr != nullptr) {
        messages.push_back(*ptr);
    } else {
        LOG(INFO) << "get an empty message under " << topic_name << endl;
    }
  }
}

void parseCompressedImage(const rosbag::Bag& bag, const string& topic_name,
                          ImgArr& images) {
  vector<sensor_msgs::CompressedImage> rgbs;      
  loadTopic(bag, topic_name, rgbs);
  for (const sensor_msgs::CompressedImage& rgb : rgbs) {
    cv::Mat img = cv::imdecode(cv::Mat(rgb.data), cv::IMREAD_UNCHANGED);
    images.emplace_back(Timestamp(rgb.header.stamp.sec, rgb.header.stamp.nsec), 
                        std::make_shared<cv::Mat>(img));
  }
}

void parseCompressedImage(const string& bagfile, const string& topic_name, 
                          ImgArr& images) {
  rosbag::Bag bag;
  openBagfile(bagfile, bag);
  parseCompressedImage(bag, topic_name, images);
  closeBagfile(bag);
}

template <typename T>
void parseOdom(const rosbag::Bag& bag, const string& topic_name, Pose3DArr<T>& stampedOdoms2D) {
  vector<nav_msgs::Odometry> odoms;
  loadTopic(bag, topic_name, odoms);
  for (const auto& odom : odoms) {
    Timestamp time(odom.header.stamp.sec, odom.header.stamp.nsec);
    Pose3D<T> pose(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                   odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
                   odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    stampedOdoms2D.emplace_back(time, std::make_shared<Pose3D<T>>(pose));
  }
}

template <typename T>
void parseOdom(const string& bagfile, const string& topic_name, Pose3DArr<T>& stampedOdoms2D) {
  rosbag::Bag bag;
  openBagfile(bagfile, bag);
  parseOdom(bag, topic_name, stampedOdoms2D);
  closeBagfile(bag);
}

template <typename T>
void parseBBox(const rosbag::Bag& bag, const string& topic_name, YOLOBBoxVecArr<T>& stampedBboxes) {
  vector<amrl_msgs::BBox2DArrayMsg> bboxesArrMsg;
  loadTopic(bag, topic_name, bboxesArrMsg);
  for (const auto& bboxesMsg : bboxesArrMsg) {
    Timestamp time(bboxesMsg.header.stamp.sec, bboxesMsg.header.stamp.nsec);
    for (const auto& bbox : bboxesMsg.bboxes) {
      Eigen::Matrix<T, 1, 1> classes;
      classes << (float)1.0; // TODO hardcoded; need to add this feature to YOLO's ros message output
      Eigen::Matrix<T, 4, 1> bboxMeasurement; 
      bboxMeasurement << (T)bbox.xyxy[0], (T)bbox.xyxy[1], (T)bbox.xyxy[2], (T)bbox.xyxy[3]; 
      stampedBboxes.emplace_back(time, std::make_shared<YOLOBBoxVec<T>>(bboxMeasurement, bbox.conf, classes));
    }
  }
}

template <typename T>
void parseBBox(const string& bagfile, const string& topic_name, YOLOBBoxVecArr<T>& stampedBboxes) {
  rosbag::Bag bag;
  openBagfile(bagfile, bag);
  parseBBox(bag, topic_name, stampedBboxes);
  closeBagfile(bag);
}

template<typename T>
void parseAnnotation(const string& annotationFile, vector<Annotation<T>>& annotations) {
  cout << "inside parseAnnotation" << endl;
  YAML::Node anotation = YAML::LoadFile(annotationFile);
  cout << annotationFile << endl;
  for (size_t i = 0; i < anotation["tracks"].size(); ++i) {
    annotations.emplace_back(annotations.size(), 
                             anotation["tracks"][i]["track"][0]["label"].Scalar(),
                             std::stoi(anotation["tracks"][i]["track"][0]["header"]["stamp"]["secs"].Scalar()),
                             std::stoi(anotation["tracks"][i]["track"][0]["header"]["stamp"]["nsecs"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["translation"]["x"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["translation"]["y"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["translation"]["z"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["rotation"]["w"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["rotation"]["x"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["rotation"]["y"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["rotation"]["z"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["box"]["length"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["box"]["width"].Scalar()),
                             std::stof(anotation["tracks"][i]["track"][0]["box"]["height"].Scalar()));
  }
}

void parsePointCloud(const string& bagfile, const string& topic_name, PclArr stampedPointclouds) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;
  rosbag::Bag bag;
  openBagfile(bagfile, bag);
  rosbag::View view(bag, rosbag::TopicQuery(topic_name));
  for (const rosbag::MessageInstance& m : view) {
    sensor_msgs::PointCloud2::ConstPtr msgPtr = m.instantiate<sensor_msgs::PointCloud2>();
    if (msgPtr != nullptr) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msgPtr, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msgPtr, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msgPtr, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          cloudPtr->points.emplace_back(*iter_x, *iter_y, *iter_z);
      }
      stampedPointclouds.emplace_back(Timestamp(msgPtr->header.stamp.sec, msgPtr->header.stamp.nsec), cloudPtr);
    }
  }
  closeBagfile(bag);
}

template <typename T>
void parsePoseFile(const string& filepath, Pose3DArr<T>& stampedPoses3D) {
  std::ifstream ifile;
  ifile.open(filepath, std::ios::in);
  if (!ifile.is_open()) {
      LOG(FATAL) << "Failed to parse pose file: " << filepath << endl;
  }
  string line, word;
  long double time;
  T x, y, z, qw, qx, qy, qz;
  while (getline(ifile, line)) {
    std::stringstream ss_line(line);
    getline(ss_line, word, ',');
    time = std::stold(word);
    getline(ss_line, word, ',');
    x = std::stof(word);
    getline(ss_line, word, ',');
    y = std::stof(word);
    getline(ss_line, word, ',');
    z = std::stof(word);
    getline(ss_line, word, ',');
    qw = std::stof(word);
    getline(ss_line, word, ',');
    qx = std::stof(word);
    getline(ss_line, word, ',');
    qy = std::stof(word);
    getline(ss_line, word, ',');
    qz = std::stof(word);
    ros::Time rosTime(time);
    Timestamp time(rosTime.sec, rosTime.nsec);
    std::shared_ptr<Pose3D<T>> pose3d = std::make_shared<Pose3D<T>>(x, y, z, qw, qx, qy, qz);
    stampedPoses3D.emplace_back(time, pose3d);
  }
}

template <typename T>
void parsePoseFile(const string& filepath, Pose2DArr<T>& stampedPoses2D) {
  Pose3DArr<T> stampedPoses3D;
  parsePoseFile(filepath, stampedPoses3D);
  for (const auto& stampedPose3D : stampedPoses3D) {
    Pose2D<T> pose2d = toPose2D(stampedPose3D);
    stampedPoses2D.push_back(pose2d);
  }
}

// TODO move this some utils
/**
 * @brief 
 * 
 * @param t1 
 * @param t2 
 * @param tol in millisec (uint64_t)
 * @return 
 */
bool sameTime(const Timestamp& t1, const Timestamp& t2, const uint64_t tol=0) {
  if (tol == 0) {
    return (t1.first == t2.first) && (t1.second == t2.second);
  } else {
    uint64_t t1Milli = vslam_types_refactor::timestampToMillis(t1);
    uint64_t t2Milli = vslam_types_refactor::timestampToMillis(t2);
    bool res = (t2Milli > t1Milli) ? (t2Milli - t1Milli < tol) : (t1Milli - t2Milli < tol);
    return res;
  }
}

/**
 * @brief Bounding boxes timestamps \subseteq Image timestamps
 * Discard redundant images
 * 
 * @param stampedImagePtrs assume sorted by timestamps
 * @param stampedBBoxVecPtrs assume sorted by timestamps
 */
template <typename T>
void associateImgsAndBboxes(ImgArr& stampedImagePtrs, YOLOBBoxVecArr<T>& stampedBBoxVecPtrs) {
  Timestamp poseStartTime, poseEndTime;

}

/**
 * @brief 
 * 
 * @tparam T 
 * @param stampedOdom3DPtrs assume sorted by timestamps
 * @param stampedPose3DPtrs assume sorted by timestamps
 */
template <typename T>
void interpolatePosesByOdom(const Pose3DArr<T>& stampedOdom3DPtrs, Pose3DArr<T>& stampedPose3DPtrs) {
  if (stampedOdom3DPtrs.empty() || stampedPose3DPtrs.empty()) { return; }
  
  Pose3DArr<T> newStampedPose3DPtrs;
  size_t odomIdx, poseIdx;
  const auto& nOdom = stampedOdom3DPtrs.size();
  // const uint64_t kTimeTol = timestampToMillis(Timestamp(0, 1e3));

  odomIdx = 1; // avoid odomIdx-1 underflow
  for (poseIdx = 0; poseIdx < stampedPose3DPtrs.size()-1; ++poseIdx) {
    newStampedPose3DPtrs.push_back(stampedPose3DPtrs[poseIdx]);
    const Timestamp& currSLAMTime = stampedPose3DPtrs[poseIdx].first;
    const Timestamp& nextSLAMTIme = stampedPose3DPtrs[poseIdx+1].first;
    // Find the odometry whose timestamp is less than the one of currSLAMPosse
    while (odomIdx < nOdom && stampedOdom3DPtrs[odomIdx].first < currSLAMTime) { ++odomIdx; }
    // If odom interval is larger than the slam pose interval, no need to interpolate
    if (stampedOdom3DPtrs[odomIdx].first >= nextSLAMTIme) { continue; } 
    
    pair<Timestamp, std::shared_ptr<Pose3D<T>>> stampedCurrOdomPose, stampedNextOdomPose;
    Pose3D<T> currOdomPose, nextOdomPose;
    Pose3D<T> deltaPose;
    while (odomIdx < nOdom && stampedOdom3DPtrs[odomIdx].first < nextSLAMTIme) {
      stampedCurrOdomPose = stampedOdom3DPtrs[odomIdx-1];
      stampedNextOdomPose = stampedOdom3DPtrs[odomIdx];
      currOdomPose = *(stampedCurrOdomPose.second);
      nextOdomPose = *(stampedNextOdomPose.second);
      // At the first time of iteration
      if (stampedCurrOdomPose.first <= currSLAMTime) {
        *(stampedCurrOdomPose.second);
        currOdomPose = interpolatePosesIn2D(
          stampedCurrOdomPose.first, *(stampedCurrOdomPose.second), 
          stampedNextOdomPose.first, *(stampedNextOdomPose.second), currSLAMTime);
      } 
      deltaPose = getPose2RelativeToPose1(currOdomPose, nextOdomPose);
      const Pose3D<T>& lastPose = *(newStampedPose3DPtrs.back().second);
      Pose3D<T> newPose = combinePoses(lastPose, deltaPose);
      newStampedPose3DPtrs.emplace_back(stampedNextOdomPose.first, std::make_shared<Pose3D<T>>(newPose));
      ++odomIdx;
    }
  }
  newStampedPose3DPtrs.push_back(stampedPose3DPtrs.back());
  stampedPose3DPtrs = newStampedPose3DPtrs;
}

template <typename T>
void interpolateTimestamps(
  const ImgArr& stampedImagePtrs, 
  const Pose3DArr<T>& stampedPose3DPtrs,
  const Pose3DArr<T>& stampedOdom3DPtrs,
  const YOLOBBoxVecArr<T>& stampedBBoxVecPtrs,
  IVBoundingBoxArr<T>& ivbboxPtrs) {
  
}

template <typename T, typename BbByTimestampType, typename BbByNodeType>
void interpolateTimestamps(
  const YOLOBBoxVecArr<T>& stampedBBoxVecPtrs,
  const std::string &bb_by_node_file_name,
  const std::function<BbByNodeType(const BbByTimestampType &, const uint64_t &node_id)> bb_by_node_creator,
  const std::function<void(const std::string &, const std::vector<BbByNodeType> &)>& bb_out_writer) {
  
}

template <typename BbByTimestampType, typename BbByNodeType>
void interpolateTimestamps(
  const std::string &bb_by_timestamp_file_name,
  const std::function<void(const std::string &, std::vector<BbByTimestampType> &)>& bb_by_timestamp_reader,
  const std::string &bb_by_node_file_name,
  const std::function<BbByNodeType(const BbByTimestampType &, const uint64_t &node_id)> bb_by_node_creator,
  const std::function<void(const std::string &, const std::vector<BbByNodeType> &)>& bb_out_writer) {
  
}



}
#endif  // IV_BBOX_H