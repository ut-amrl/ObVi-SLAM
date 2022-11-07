#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <iomanip>

#include <iv_bbox/iv_bbox_utils.h>

namespace iv_bbox {
  
namespace fs = boost::filesystem;
using namespace vslam_types_refactor;
using std::unordered_map;
using std::string;
using std::vector;

template <typename T>
Eigen::Matrix<T, 4, 1>
bboxVecErrFunc(const BbCorners<T>& measurement, const BbCorners<T>& gt) {
  return (measurement - gt).cwiseAbs();
}

template <typename T>
bool patchExtractor(const BbCorners<T>& measurement, const BbCorners<T>& gt, const cv::Mat& inImg, cv::Mat& outImg) {
  const int padding = 5;
  int min_x, min_y, max_x, max_y;
  min_x = std::min((int)measurement[0], (int)gt[0]);
  min_y = std::min((int)measurement[2], (int)gt[2]);
  max_x = std::max((int)measurement[1], (int)gt[1]);
  max_y = std::max((int)measurement[3], (int)gt[3]);
  int halfSideLength = std::max(max_x-min_x, max_y-min_y) / 2;
  
  int center_x, center_y;
  center_x = (min_x + max_x) / 2;
  center_y = (min_y + max_y) / 2;
  min_x = center_x - halfSideLength;
  min_y = center_y - halfSideLength;
  max_x = center_x + halfSideLength;
  max_y = center_y + halfSideLength;

  min_x = std::max((int)0,          min_x-padding);
  min_y = std::max((int)0,          min_y-padding);
  max_x = std::min((int)inImg.cols, max_x+padding);
  max_y = std::min((int)inImg.rows, max_y+padding);

  int sideLength = std::min(max_x-min_x, max_y-min_y);
  if (sideLength < 2*(padding+halfSideLength)) {
    return false;
  } else {
    outImg = cv::Mat::zeros(sideLength, sideLength, 16);
    outImg = inImg.colRange(min_x, max_x).rowRange(min_y, max_y);
    return true;
  }

  // outImg = cv::Mat::zeros(inImg.rows, inImg.cols, 16);
  // cout << inImg.rows << "x" << inImg.cols << endl;
  // cout << "measurement: " <<  measurement[0] << ", " << measurement[2] << ", " << measurement[1] << ", " << measurement[3] << endl;
  // cout << "gt: " <<  gt[0] << ", " << gt[2] << ", " << gt[1] << ", " << gt[3] << endl;
  // cout << "after: " << min_x << ", " << min_y << ", " << max_x << ", " << max_y << endl << endl;;
  // cv::Mat inPatch  = inImg.colRange(min_x, max_x).rowRange(min_y, max_y);
  // cv::Mat outPatch = outImg.colRange(min_x, max_x).rowRange(min_y, max_y);
  // inPatch.copyTo(outPatch);
}

template <typename T>
bool patchExtractor2(const BbCorners<T>& measurement, const BbCorners<T>& gt, const cv::Mat& inImg, cv::Mat& outImg) {
  const int padding = 5;
  int min_x, min_y, max_x, max_y;
  min_x = std::min((int)measurement[0], (int)gt[0]);
  min_y = std::min((int)measurement[2], (int)gt[2]);
  max_x = std::max((int)measurement[1], (int)gt[1]);
  max_y = std::max((int)measurement[3], (int)gt[3]);
  
  min_x = std::max((int)0,          min_x-padding);
  min_y = std::max((int)0,          min_y-padding);
  max_x = std::min((int)inImg.cols, max_x+padding);
  max_y = std::min((int)inImg.rows, max_y+padding);
  outImg = cv::Mat::zeros(inImg.rows, inImg.cols, 16);
  // cout << inImg.rows << "x" << inImg.cols << endl;
  // cout << "measurement: " <<  measurement[0] << ", " << measurement[2] << ", " << measurement[1] << ", " << measurement[3] << endl;
  // cout << "gt: " <<  gt[0] << ", " << gt[2] << ", " << gt[1] << ", " << gt[3] << endl;
  // cout << "after: " << min_x << ", " << min_y << ", " << max_x << ", " << max_y << endl << endl;;
  cv::Mat inPatch  = inImg.colRange(min_x, max_x).rowRange(min_y, max_y);
  cv::Mat outPatch = outImg.colRange(min_x, max_x).rowRange(min_y, max_y);
  inPatch.copyTo(outPatch);
  return true;
}

template<typename T>
class IVBBox {
private:
  size_t uid_;

  CameraIntrinsics<T> intrinsics_;
  CameraExtrinsics<T> extrinsics_;
  
  string inDir_;
  string outDir_;
  string outImgDir_;
  string outBBoxDir_;
  string outGTDir_;
  /**
   * @brief ROS Topic Names
   * compressedImg: any compressed image topic
   * yoloBBox: 
   * odom:
   */
  unordered_map<string, string> names_;
  vector<string> bagnames_;

  ImgArr stampedImages_;
  YOLOBBoxVecArr<T> stampedYOLOBBoxVecs_;
  Pose3DArr<T> stampedOdoms_;
  Pose3DArr<T> stampedPoses_;
  vector<Annotation<T>> annotations_;
  vector<PCLCluster<T>> clusters_;
  vector<IVBoundingBox<T>> ivBoundingBoxes_;

  void parseConfig_(const string& configPath) {
    YAML::Node nodes = YAML::LoadFile(configPath);
    inDir_ = nodes["in_dir"].Scalar(); // TODO robustly handle filepath
    names_["datasetName"] = nodes["dataset_name"].Scalar();
    outDir_ = nodes["out_dir"].Scalar() + names_["datasetName"] + "/";
    for (size_t i = 0; i < nodes["bagnames"].size(); ++i) {
      bagnames_.emplace_back(nodes["bagnames"][i].Scalar());
    }
    names_["compressedImg"] = nodes["ros_topics"]["compressed_img"].Scalar();
    names_["yoloBBox"] = nodes["ros_topics"]["yolo_bbox"].Scalar();
    names_["odom"] = nodes["ros_topics"]["odom"].Scalar();
  }

  void saveBboxImg_(const string& path,  
                   const BbCornerPair<T>& bbCornerPair,
                   const cv::Mat& img) {
    T min_x, min_y, max_x, max_y;
    min_x = std::max((T)0,    bbCornerPair.first[0]);
    min_y = std::max((T)0,    bbCornerPair.first[1]);
    max_x = std::min((T)img.cols(), bbCornerPair.second[0]);
    max_y = std::min((T)img.rows(),  bbCornerPair.second[1]);
    cv::Mat bbox_img = img.clone();
    cv::rectangle(bbox_img, 
                  cv::Point(min_x,min_y),
                  cv::Point(max_x,max_y),
                  cv::Scalar(0, 255, 0), 2);
    cv::imwrite("test.png", bbox_img);
  }

  void preparePaths_(const size_t bagIdx, 
    string& bagpath, string& bboxBagPath, string& slamPath, string& annotPath, string& clusterPath) {
    // parse input directories
    const string& bagname = bagnames_[bagIdx];
    bagpath     = inDir_ + "bags/" + bagname + ".bag";
    bboxBagPath = inDir_ + "yolo/yolo_"  + bagname + ".bag";
    slamPath    = inDir_ + "LeGO-LOAM/"  + bagname + ".csv"; 
    annotPath   = inDir_ + "annotate/"   + bagname + ".yaml";
    clusterPath = inDir_ + "clustering/" + bagname + ".csv"; 
  }

  void prepareOutputDirs_() {
    // create output directory if not exists
    if (!fs::is_directory(outDir_) || !fs::exists(outDir_)) {
      if (!fs::create_directory(outDir_)) {
        LOG(FATAL) << "failed to create directory " << outDir_ << endl;
      }
    }
    outImgDir_  = outDir_ + "images/";
    outBBoxDir_ = outDir_ + "bboxes/";
    outGTDir_   = outDir_ + "gts/";
    if (!fs::is_directory(outImgDir_) || !fs::exists(outImgDir_)) {
      if (!fs::create_directory(outImgDir_)) {
        LOG(FATAL) << "failed to create directory " << outImgDir_ << endl;
      }
    }
    if (!fs::is_directory(outBBoxDir_) || !fs::exists(outBBoxDir_)) {
      if (!fs::create_directory(outBBoxDir_)) {
        LOG(FATAL) << "failed to create directory " << outBBoxDir_ << endl;
      }
    }
    if (!fs::is_directory(outGTDir_) || !fs::exists(outGTDir_)) {
      if (!fs::create_directory(outGTDir_)) {
        LOG(FATAL) << "failed to create directory " << outGTDir_ << endl;
      }
    }
    for (const auto& entry : fs::directory_iterator(outImgDir_)) {
      fs::remove_all(entry.path());
    }
    for (const auto& entry : fs::directory_iterator(outBBoxDir_)) {
      fs::remove_all(entry.path());
    }
    for (const auto& entry : fs::directory_iterator(outGTDir_)) {
      fs::remove_all(entry.path());
    }
  }

  void clear_() {
    stampedImages_.clear();
    stampedYOLOBBoxVecs_.clear();
    stampedOdoms_.clear();
    stampedPoses_.clear();
    annotations_.clear();
    ivBoundingBoxes_.clear();
  }

  int associateAnnotationWithClusters_(const Annotation<T>& annotation, const vector<PCLCluster<T>>& clusters) {
    float minDist = std::numeric_limits<float>::max();
    int argMinDist = -1;
    for (size_t i = 0; i < clusters.size(); ++i) {
      const auto& cluster = clusters[i];
      if (isPointInAnnotatedBBox3d(cluster.centroid_, annotation)) {
        if ((cluster.centroid_-annotation.state_.pose_.transl_).norm() < minDist) {
          argMinDist = i;
          minDist = (cluster.centroid_-annotation.state_.pose_.transl_).norm();
        }
      }
    }
    return argMinDist;
  }

  void labelClusters_() {
    vector<PCLCluster<T>> newClusters;
    for (const auto& annotation : annotations_) {
      int clusterIdx = associateAnnotationWithClusters_(annotation, clusters_);
      if (clusterIdx != -1) {
        clusters_[clusterIdx].label_ = annotation.label_;
        newClusters.push_back(clusters_[clusterIdx]);
      }
    }
    clusters_ = newClusters;
  }

  void prepare_(const size_t bagIdx) {
    clear_();
    string bagpath, bboxBagPath, slamPath, annotPath, clusterPath;
    IVBBox<T>::preparePaths_(bagIdx, bagpath, bboxBagPath, slamPath, annotPath, clusterPath);
    parseCompressedImage(bagpath, names_["compressedImg"], stampedImages_);
    parseOdom(bagpath, names_["odom"], stampedOdoms_);
    parseBBox(bboxBagPath, names_["yoloBBox"], stampedYOLOBBoxVecs_);
    parsePoseFile(slamPath, stampedPoses_);
    parseAnnotation(annotPath, annotations_);
    parsePointCloudCluster(clusterPath, clusters_);
    labelClusters_();
  }

  Eigen::Matrix<T, 2, 1>
  getBBoxCenter_(const BbCorners<T>& bbox) {
    const T& min_x = bbox[0];
    const T& min_y = bbox[2];
    const T& max_x = bbox[1];
    const T& max_y = bbox[3];
    return Eigen::Matrix<T, 2, 1>((min_x+max_x)/2.0, (min_y+max_y)/2.0);
  }

  int associateBbCorners_(const BbCorners<T>& bbox1, const vector<BbCorners<T>> bboxes2) {
    if (bboxes2.empty()) { return -1; }

    const float& kBBoxCenterTol = 20; 
    Eigen::Matrix<T, 2, 1> center1 = getBBoxCenter_(bbox1);
    float minDiff = std::numeric_limits<float>::max();
    int argminDiff = -1;
    for (size_t idx2 = 0; idx2 < bboxes2.size(); ++idx2) {
      const auto& bbox2  = bboxes2[idx2];
      Eigen::Matrix<T, 2, 1> center2 = getBBoxCenter_(bbox2);
      if ((center2 - center1).norm() < minDiff) {
        minDiff = (center2 - center1).norm();
        argminDiff = idx2;
      }
    }
    if (minDiff < kBBoxCenterTol) {
      return argminDiff;
    } else {
      return -1;
    }
  }

  void getIVBoundingBoxArr_(const vector<Timestamp>& timestamps) {
    size_t imgIdx, bboxIdx, poseIdx, timeIdx;
    imgIdx = bboxIdx = poseIdx = timeIdx = 0;
    const size_t& nImages   = stampedImages_.size();
    const size_t& nBBoxVecs = stampedYOLOBBoxVecs_.size();
    const size_t& nPoses    = stampedPoses_.size();
    bool found;
    for (const auto& currTime : timestamps) {
      found = true;
      while (imgIdx < nImages && stampedImages_[imgIdx].first < currTime) { ++imgIdx; }
      if (imgIdx >= nImages 
        || (imgIdx < nImages && stampedImages_[imgIdx].first != currTime)) {found &= false;}
      if (!found) {
        // cout << "imgIdx=" << imgIdx << endl;
        // cout << "image not found!" << endl;
        continue;
      }
      while (poseIdx < nPoses && stampedPoses_[poseIdx].first < currTime) { ++poseIdx; }
      if (poseIdx >= nPoses 
        || (poseIdx < nPoses && stampedPoses_[poseIdx].first != currTime)) {found &= false;}
      if (!found) {
        // cout << "poseIdx=" << poseIdx << endl;
        // cout << "pose not found!" << endl;
        continue;
      }
      // multiple bboxes can have the same timestamp, so it's find the starting idx for this iteration
      while (bboxIdx < nBBoxVecs && stampedYOLOBBoxVecs_[bboxIdx].first < currTime) { 
        ++bboxIdx; 
      }
      if (bboxIdx >= nBBoxVecs 
        || (bboxIdx < nBBoxVecs && stampedYOLOBBoxVecs_[bboxIdx].first != currTime)) {found &= false;}
      if (!found) {
        // cout << "bboxIdx=" << bboxIdx << endl;
        // cout << "bbox not found!" << endl;
        continue;
      }
      if (!found) { continue; }
      // cout << "found!" << endl;

      const auto& currImgPtr  = stampedImages_[imgIdx].second;
      const auto& currPosePtr = stampedPoses_[poseIdx].second;
      vector<BbCorners<T>> supervisedBBoxes;

      for (auto& cluster : clusters_) {
        ObjectDim<float> coneDim;
        coneDim << (T)0.3, (T)0.3, (T)0.5;
        cluster.state_.dimensions_ = coneDim;
        BbCornerPair<float> supervisedBBoxPair = getCornerLocationsPair(cluster.state_, *currPosePtr, extrinsics_, intrinsics_.camera_mat);
        supervisedBBoxes.push_back(cornerLocationsPairToVector(supervisedBBoxPair));
      }

      // Pose3D<T> conePose;
      // // conePose.transl_ << (T)2.699, (T)-0.32, (T)-0.475;
      // conePose.transl_ << (T)2.699, (T)-0.32, (T)-0.575;
      // conePose.orientation_ = Orientation3D<float>(0, Eigen::Vector3f(0, 0, 1));
      // ObjectDim<float> coneDim;
      // coneDim << (T)0.3, (T)0.3, (T)0.5;
      // CubeState<T> coneState(conePose, coneDim);
      // BbCornerPair<float> supervisedBBoxPair = getCornerLocationsPair(coneState, *currPosePtr, extrinsics_, intrinsics_.camera_mat);
      // supervisedBBoxes.push_back(cornerLocationsPairToVector(supervisedBBoxPair));

      while ( stampedYOLOBBoxVecs_[bboxIdx].first == currTime) {
        const auto& currYOLOBBoxVecPtr = stampedYOLOBBoxVecs_[bboxIdx].second;
        const auto& predBBox = currYOLOBBoxVecPtr->measurement_;
        int supervisedBBoxIdx = associateBbCorners_(predBBox, supervisedBBoxes);
        if (supervisedBBoxIdx != -1) { 
          // cv::Mat bboxImage = currImgPtr->clone();
          // bboxImage = drawBBoxes(bboxImage, supervisedBBoxes, cv::Scalar(0, 255, 0));
          // bboxImage = drawBBox(  bboxImage, predBBox,         cv::Scalar(255, 0, 0));
          // cv::imwrite("debug/" + std::to_string(uid) + ".png", bboxImage);
          IVBoundingBox ivBoundingBox(currTime, *currYOLOBBoxVecPtr, supervisedBBoxes[supervisedBBoxIdx], currImgPtr);
          ivBoundingBox.err_ = bboxVecErrFunc(ivBoundingBox.yoloVec_.measurement_, ivBoundingBox.gt_); // TODO fix the constructor
          cv::Mat imgPatch; 
          if (!patchExtractor(ivBoundingBox.yoloVec_.measurement_, ivBoundingBox.gt_, *(ivBoundingBox.imgPtr_), imgPatch)) { continue; }
          ivBoundingBox.imgPatchPtr_ = std::make_shared<cv::Mat>(imgPatch);
          ivBoundingBoxes_.push_back(ivBoundingBox);
        }
        ++bboxIdx;
      }
    }
  }

  void dumpIVBBoxes_() {
    for (const auto& ivBoundingBox : ivBoundingBoxes_) {
      if (uid_ > 200) { return; }
      cv::imwrite(outImgDir_ + std::to_string(uid_) + ".png", *(ivBoundingBox.imgPatchPtr_));
      std::ofstream ofileBBox;
      ofileBBox.open(outBBoxDir_ + std::to_string(uid_) + ".txt", std::ios::trunc);
      for (int i = 0; i < ivBoundingBox.yoloVec_.measurement_.rows(); ++i) {
        ofileBBox << ivBoundingBox.yoloVec_.measurement_[i] << " ";
      }
      ofileBBox << ivBoundingBox.yoloVec_.conf_;
      for (int i = 0; i < ivBoundingBox.yoloVec_.classes_.rows(); ++i) {
        ofileBBox << " " << ivBoundingBox.yoloVec_.classes_[i];
      }
      ofileBBox.close();
      std::ofstream ofileGT;
      ofileGT.open(outGTDir_ + std::to_string(uid_) + ".txt", std::ios::trunc);
      for (int i = 0; i < ivBoundingBox.gt_.rows(); ++i) {
        ofileGT << ivBoundingBox.gt_[i] << " ";
      }
      ofileGT.close();
      ++uid_;
    }
  }

  Eigen::Matrix<T, Eigen::Dynamic, 4>
  getErrMatrix_() {
    const size_t& nBBoxes = ivBoundingBoxes_.size();
    Eigen::Matrix<T, Eigen::Dynamic, 4> X;
    X.resize(nBBoxes, 4);
    for (size_t i = 0; i < nBBoxes; ++i) {
      const auto& ivBoundingBox = ivBoundingBoxes_[i];
      X.row(i) = ivBoundingBox.err_;
    }
    return X;
  }

  Eigen::Matrix<T, Eigen::Dynamic, 4>
  getNormalizedErrMatrix_() {
    const size_t& nBBoxes = ivBoundingBoxes_.size();
    Eigen::Matrix<T, Eigen::Dynamic, 4> X;
    X.resize(nBBoxes, 4);
    for (size_t i = 0; i < nBBoxes; ++i) {
      const auto& ivBoundingBox = ivBoundingBoxes_[i];
      T xDim = abs(ivBoundingBox.gt_[0]-ivBoundingBox.gt_[1]);
      T yDim = abs(ivBoundingBox.gt_[2]-ivBoundingBox.gt_[3]);
      X(i, 0) = ivBoundingBox.err_[0] / xDim;
      X(i, 1) = ivBoundingBox.err_[1] / xDim;
      X(i, 2) = ivBoundingBox.err_[2] / yDim;
      X(i, 3) = ivBoundingBox.err_[3] / yDim;
    }
    return X;
  }

public:
  IVBBox(const CameraIntrinsics<T>& intrinsics,
      const CameraExtrinsics<T>& extrinsics,
      const string& configPath) : intrinsics_(intrinsics), extrinsics_(extrinsics) {
    IVBBox<T>::parseConfig_(configPath);
  }

  ~IVBBox() = default;

  // TODO need to change to actual models
  static void cubeStateToEllipsoidState(
    const CubeState<T>& cubeState, EllipsoidState<T>& ellipsoidState) {
    ellipsoidState = cubeState;
  }

  /**
   * @brief the main function
   */
  void createDataset() {
    cout << "start creating dataset..." << endl;
    uid_ = 0;
    prepareOutputDirs_();
    for (size_t bagIdx = 0; bagIdx < bagnames_.size(); ++bagIdx) {
      IVBBox<T>::prepare_(bagIdx);
      cout << "stampedPoses_ size before interpolatePosesByOdom: " << stampedPoses_.size() << endl;
      interpolatePosesByOdom(stampedOdoms_, stampedPoses_);
      cout << "stampedPoses_ size after interpolatePosesByOdom: " << stampedPoses_.size() << endl;
      vector<Timestamp> targetTimestamps;
      for (const auto& stampedImage: stampedImages_) {
        targetTimestamps.push_back(stampedImage.first);
      }
      interpolatePosesByTime(targetTimestamps, stampedPoses_);
      getIVBoundingBoxArr_(targetTimestamps);
      dumpIVBBoxes_();
      cout << stampedImages_.size() << endl;;
      cout << stampedYOLOBBoxVecs_.size() << endl;
      cout << stampedOdoms_.size() << endl;
      cout << stampedPoses_.size() << endl;
      cout << clusters_.size() << endl;
      cout << annotations_.size() << endl;
      cout << ivBoundingBoxes_.size() << endl;

      Eigen::Matrix<T, Eigen::Dynamic, 4> X = getErrMatrix_();
      cout << "Error Matrix=" << endl;
      cout << X << endl << endl;
      cout << "Cov Matrix=" << endl;
      cout << getCovMat(X) << endl;

      X = getNormalizedErrMatrix_();
      cout << "Normalized Error Matrix=" << endl;
      cout << X << endl << endl;
      cout << "Normalized Cov Matrix=" << endl;
      cout << std::setprecision(2) << getCovMat(X) << endl;
    }
  }

};

}