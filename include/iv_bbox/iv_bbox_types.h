#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <cmath>

#include <refactoring/types/ellipsoid_utils.h>

namespace iv_bbox {

using namespace vslam_types_refactor;
using std::string;
using std::vector;
using std::unordered_map;
using std::pair;
using std::cout;
using std::endl;

typedef uint16_t AnnotateId;
// typedef vector<pair<ros::Time, std::shared_ptr<cv::Mat>>> ImgArr;
typedef vector<pair<Timestamp, std::shared_ptr<cv::Mat>>> ImgArr;
typedef vector<pair<Timestamp, pcl::PointCloud<pcl::PointXYZ>::Ptr>> PclArr;

template <typename T>
using Pose3DArr = vector<pair<Timestamp, std::shared_ptr<vslam_types_refactor::Pose3D<T>>>>;

template <typename T>
using Pose2DArr = vector<pair<Timestamp, std::shared_ptr<vslam_types_refactor::Pose2D<T>>>>;

template <typename T>
using CubeState = vslam_types_refactor::EllipsoidState<T>;

template <typename T>
struct Annotation {
  AnnotateId annotateId_;
  Timestamp timestamp_;
  std::string label_;
  CubeState<T> state_;
  Annotation() {}
  Annotation(const AnnotateId annotateId, const std::string& label,
             const int32_t sec, const int32_t nsec,
             const T x, const T y, const T z,
             const T qw, const T qx, const T qy, const T qz,
             const T length, const T width, const T height) {
    annotateId_ = annotateId;
    timestamp_ = Timestamp(sec, nsec);
    label_ = label;
    state_.pose_.transl_ << x, y, z;
    state_.pose_.orientation_ = Eigen::AngleAxis<T>(Eigen::Quaternion<T>(qw, qx, qy, qz));
    state_.dimensions_ << length, width, height; // TODO need to verify length & width
  }

  friend std::ostream& operator<<(std::ostream& o, const Annotation& annot) {
    o << "ID: " << annot.annotateId_ << ", label: " << annot.label_ << endl
      << annot.state_.pose_.transl_[0] << "  " << annot.state_.pose_.transl_[1] << " " << annot.state_.pose_.transl_[2] << " "
      << (annot.state_.pose_.orientation_.angle() * annot.state_.pose_.orientation_.axis()).transpose() << " "
      << annot.state_.dimensions_[0] << ", " << annot.state_.dimensions_[1] << ", " << annot.state_.dimensions_[2];
    return o;
  }
};

template <typename T>
struct YOLOBBoxVec {
  BbCorners<T> measurement_;
  T conf_;
  Eigen::Matrix<T, Eigen::Dynamic, 1> classes_;
  YOLOBBoxVec() {}
  YOLOBBoxVec(const BbCorners<T>& measurement, const T& conf, const Eigen::Matrix<T, Eigen::Dynamic, 1>& classes) 
    : measurement_(measurement), conf_(conf), classes_(classes) {}

  Eigen::Matrix<T, Eigen::Dynamic, 1> serialize() {
    const size_t nrow = measurement_.rows() + 1 + classes_.rows();
    Eigen::Matrix<T, Eigen::Dynamic, 1> vec;
    vec.resize(nrow, 1);
    // TODO fix me to block operations
    cout << "nrow: " << nrow << endl;
    cout << "measurement_.rows(): " << measurement_.rows() << endl;
    for (int i = 0; i < measurement_.rows(); ++i) {
      vec[i] = measurement_[i];
    }
    vec[measurement_.rows()] = conf_;
    size_t idx = 0;
    for (int i = measurement_.rows()+1; i < nrow; ++i) {
      vec[i] = classes_[idx++];
    }
    return vec;
  }
};

template <typename T>
using YOLOBBoxVecArr = vector<pair<Timestamp, std::shared_ptr<YOLOBBoxVec<T>>>>;

/**
 * @brief struct for NN Input
 * @tparam T 
 */
template <typename T>
struct IVBoundingBox {
  Timestamp timestamp_; // TODO for debugging only; redundant & delete me
  YOLOBBoxVec<T> yoloVec_;
  BbCorners<T> gt_;
  std::shared_ptr<cv::Mat> imgPtr_; // TODO for debugging olny; redundant & delete me
  Eigen::Matrix<T, 4, 1> err_;
  std::shared_ptr<cv::Mat> imgPatchPtr_;
  IVBoundingBox() {}

  IVBoundingBox(const Timestamp& timestamp, const YOLOBBoxVec<T>& yoloVec, 
    const BbCorners<T>& gt, const std::shared_ptr<cv::Mat> imgPtr) 
    : timestamp_(timestamp), yoloVec_(yoloVec), gt_(gt), imgPtr_(imgPtr) {}

  IVBoundingBox(
    const Timestamp& timestamp, 
    const YOLOBBoxVec<T>& yoloVec, 
    const BbCorners<T>& gt, 
    const cv::Mat& img,
    const std::function<Eigen::Matrix<T, 4, 1>(const BbCorners<T>& measurement, const BbCorners<T>& gt)> errFunc,
    const std::function<void(const cv::Mat& inImg, cv::Mat& outImg)> patchExtractor) 
    : timestamp_(timestamp), yoloVec_(yoloVec), gt_(gt) {
    err_ = errFunc(yoloVec.measurement_, gt);
    imgPtr_ = std::make_shared<cv::Mat>(img);
    cv::Mat imgPatch;
    patchExtractor(*imgPtr_, imgPatch);
    imgPatchPtr_ = std::make_shared<cv::Mat>(imgPatch);
  }

  IVBoundingBox(
    const Timestamp& timestamp, 
    const YOLOBBoxVec<T>& yoloVec, 
    const BbCorners<T>& gt, 
    const std::shared_ptr<cv::Mat> imgPtr,
    const std::function<Eigen::Matrix<T, 4, 1>(const BbCorners<T>& measurement, const BbCorners<T>& gt)>& errFunc,
    const std::function<void(const cv::Mat& inImg, cv::Mat& outImg)>& patchExtractor) 
    : timestamp_(timestamp), yoloVec_(yoloVec), gt_(gt), imgPtr_(imgPtr) {
    err_ = errFunc(yoloVec.measurement_, gt);
    cv::Mat imgPatch;
    patchExtractor(*imgPtr_, imgPatch);
    imgPatchPtr_ = std::make_shared<cv::Mat>(imgPatch);
  }
};

typedef uint32_t ClusterId;

template <typename T>
struct PCLCluster {
  string label_;
  vector<std::shared_ptr<Eigen::Vector3f>> pointPtrs_;
  Eigen::Vector3f centroid_;
  CubeState<T> state_;
  PCLCluster() {}
  PCLCluster(const string& label, const vector<std::shared_ptr<Eigen::Vector3f>>& pointPtrs) 
    : label_(label), pointPtrs_(pointPtrs) {}
};

}