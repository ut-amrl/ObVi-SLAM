#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

#include <iv_bbox/iv_bbox_utils.h>

namespace iv_bbox {
  
using namespace vslam_types_refactor;
using std::unordered_map;
using std::string;
using std::vector;

template <typename T>
Eigen::Matrix<T, 4, 1>
bboxVecErrFunc(const BbCorners<T>& measurement, const BbCorners<T>& gt) {
  return (measurement - gt).cwiseAbs();
}

void patchExtractor(const cv::Mat& inImg, cv::Mat& outImg) {

}

template<typename T>
class IVBBox {
private:
  CameraIntrinsics<T> intrinsics_;
  CameraExtrinsics<T> extrinsics_;
  
  string inDir_;
  string outDir_;
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

    const string& bagname = bagnames_[bagIdx];
    bagpath     = inDir_ + "bags/" + bagname + ".bag";
    bboxBagPath = inDir_ + "yolo/yolo_"  + bagname + ".bag";
    slamPath    = inDir_ + "LeGO-LOAM/"  + bagname + ".csv"; 
    annotPath   = inDir_ + "annotate/"   + bagname + ".yaml";
    clusterPath = inDir_ + "clustering/" + bagname + ".csv"; 
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
      clusters_[clusterIdx].label_ = annotation.label_;
      newClusters.push_back(clusters_[clusterIdx]);
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
    cout << "stampedPoses_ size in after parsePoseFile: " << stampedPoses_.size() << endl;
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

    const float& kBBoxCenterTol = 50; 
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
    size_t uid = 0;

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

      // cout << "start creating supervised bboxes for this frame" << endl;
      for (const auto& cluster : clusters_) {
        BbCornerPair<float> supervisedBBoxPair = getCornerLocationsPair(cluster.state_, *currPosePtr, extrinsics_, intrinsics_.camera_mat);
        supervisedBBoxes.push_back(cornerLocationsPairToVector(supervisedBBoxPair));
      }

      while ( stampedYOLOBBoxVecs_[bboxIdx].first == currTime) {
        const auto& currYOLOBBoxVecPtr = stampedYOLOBBoxVecs_[bboxIdx].second;
        const auto& predBBox = currYOLOBBoxVecPtr->measurement_;
        int supervisedBBoxIdx = associateBbCorners_(predBBox, supervisedBBoxes);
        if (supervisedBBoxIdx != -1) { 
          cv::Mat bboxImage = currImgPtr->clone();
          bboxImage = drawBBoxes(bboxImage, supervisedBBoxes, cv::Scalar(0, 255, 0));
          bboxImage = drawBBox(  bboxImage, predBBox,         cv::Scalar(255, 0, 0));
          cv::imwrite("debug/" + std::to_string(uid) + ".png", bboxImage);
          IVBoundingBox ivBoundingBox(currTime, *currYOLOBBoxVecPtr, supervisedBBoxes[supervisedBBoxIdx], currImgPtr);
          ivBoundingBox.err_ = bboxVecErrFunc(ivBoundingBox.yoloVec_.measurement_, ivBoundingBox.gt_); // TODO fix the constructor
          ivBoundingBoxes_.push_back(ivBoundingBox);
          ++uid;
        }
        ++bboxIdx;
      }
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
    }
  }

};

}