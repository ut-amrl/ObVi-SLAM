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
  string outBBoxImgDir_;
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
    string& bagpath, string& syncBagpath, string& bboxBagPath, string& slamPath, string& annotPath, string& clusterPath) {
    // parse input directories
    const string& bagname = bagnames_[bagIdx];
    bagpath     = inDir_ + "bags/" + bagname + ".bag";
    syncBagpath = inDir_ + "bags/sync/bags/sync_" + bagname + ".bag";
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
    outImgDir_     = outDir_ + "images/";
    outBBoxDir_    = outDir_ + "bboxes/";
    outGTDir_      = outDir_ + "gts/";
    outBBoxImgDir_ = outDir_ + "bboxImages/";
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
    if (!fs::is_directory(outBBoxImgDir_) || !fs::exists(outBBoxImgDir_)) {
      if (!fs::create_directory(outBBoxImgDir_)) {
        LOG(FATAL) << "failed to create directory " << outBBoxImgDir_ << endl;
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
    for (const auto& entry : fs::directory_iterator(outBBoxImgDir_)) {
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
      // if ((cluster.centroid_-annotation.state_.pose_.transl_).norm() < 2.0) {
      //   cout << "cluster centroid: " << cluster.centroid_.transpose() << endl;
      //   cout << "annote  centroid: " << annotation.state_.pose_.transl_.transpose() << endl;
      //   cout << "diff: " << (cluster.centroid_-annotation.state_.pose_.transl_).norm() << endl;
      //   Eigen::Matrix<T, 3, 1> point_in_box = 
      //       annotation.state_.pose_.orientation_.inverse() * (cluster.centroid_ - annotation.state_.pose_.transl_);
      //   cout << abs(point_in_box[0]) << ", " << annotation.state_.dimensions_[0]/2.0 << endl;
      //   cout << abs(point_in_box[1]) << ", " << annotation.state_.dimensions_[1]/2.0 << endl;
      //   cout << abs(point_in_box[2]) << ", " << annotation.state_.dimensions_[2]/2.0 << endl;
      //   cout << endl;
      // }
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
    // for (size_t i = 0; i < clusters_.size(); ++i) {
    //   toCSV("debug/clusters/"+std::to_string(i)+".csv", clusters_[i]);
    // }
    for (const auto& annotation : annotations_) {
      int clusterIdx = associateAnnotationWithClusters_(annotation, clusters_);
      if (clusterIdx != -1) {
        clusters_[clusterIdx].label_ = annotation.label_;
        newClusters.push_back(clusters_[clusterIdx]);
      }
    }
    clusters_ = newClusters;
  }

  void prepare_(const size_t bagIdx, bool sync) {
    cout << "preparing for bagfile " << bagIdx << endl;
    clear_();
    string bagpath, syncBagpath, bboxBagPath, slamPath, annotPath, clusterPath;
    IVBBox<T>::preparePaths_(bagIdx, bagpath, syncBagpath, bboxBagPath, slamPath, annotPath, clusterPath);
    cout << "finish preparing for the paths" << endl;
    // parseCompressedImage(bagpath, names_["compressedImg"], stampedImages_);
    parseCompressedImage(syncBagpath, names_["compressedImg"], stampedImages_);
    cout << "finish preparing for compressed images; stampedImages_ size = " << stampedImages_.size() << endl;
    parseOdom(bagpath, names_["odom"], stampedOdoms_);
    cout << "finish preparing for odoms; stampedOdoms_ size = " << stampedOdoms_.size() << endl;
    parseBBox(bboxBagPath, names_["yoloBBox"], stampedYOLOBBoxVecs_);
    cout << "finish preparing for bboxes; stampedYOLOBBoxVecs_ size = " << stampedYOLOBBoxVecs_.size() << endl;
    parsePoseFile(slamPath, stampedPoses_);
    cout << "finish preparing for poses; stampedPoses_ size = " << stampedPoses_.size() << endl;
    parseAnnotation(annotPath, annotations_);
    cout << "finish preparing for annotations; annotations_ size = " << annotations_.size() << endl;
    parsePointCloudCluster(clusterPath, clusters_);
    cout << "finish preparing for clusters_; clusters_ size = " << clusters_.size() << endl;
    labelClusters_();
    cout << "finish labelling clusters" << endl;
  }

  Eigen::Matrix<T, 2, 1>
  getBBoxCenter_(const BbCorners<T>& bbox) {
    const T& min_x = bbox[0];
    const T& min_y = bbox[2];
    const T& max_x = bbox[1];
    const T& max_y = bbox[3];
    return Eigen::Matrix<T, 2, 1>((min_x+max_x)/2.0, (min_y+max_y)/2.0);
  }

  float getOverlapArea_(const BbCorners<T>& bbox1, const BbCorners<T> bbox2) {
    float min_x, min_y, max_x, max_y;
    min_x = std::max(bbox1[0], bbox2[0]);
    min_y = std::max(bbox1[2], bbox2[2]);
    max_x = std::min(bbox1[1], bbox2[1]);
    max_y = std::min(bbox1[3], bbox2[3]);
    if (min_x >= max_x || min_y >= max_y ) { return 0.0; }
    float intersectionArea = (max_x - min_x) * (max_y - min_y);
    float bbox1Area = (bbox1[1] - bbox1[0]) * (bbox1[3] - bbox1[2]);
    return intersectionArea / bbox1Area;
  }

  int associateBbCorners_(const BbCorners<T>& bbox1, const vector<BbCorners<T>> bboxes2) {
    if (bboxes2.empty()) { return -1; }

    float maxOverlapAreaPercent = -1.0;
    int argmaxArea = -1;
    for (size_t idx2 = 0; idx2 < bboxes2.size(); ++idx2) {
      const auto& bbox2  = bboxes2[idx2];
      if (getOverlapArea_(bbox1, bbox2) > maxOverlapAreaPercent) {
        maxOverlapAreaPercent = getOverlapArea_(bbox1, bbox2);
        argmaxArea = idx2;
      }
    }
    // cout << "maxOverlapAreaPercent: " << maxOverlapAreaPercent << endl;
    if (maxOverlapAreaPercent > 0.95) { return argmaxArea; } else { return -1; }
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

      cv::Mat bboxImg = currImgPtr->clone();
      while ( stampedYOLOBBoxVecs_[bboxIdx].first == currTime) {
        const auto& currYOLOBBoxVecPtr = stampedYOLOBBoxVecs_[bboxIdx].second;
        const auto& predBBox = currYOLOBBoxVecPtr->measurement_;
        int supervisedBBoxIdx = associateBbCorners_(predBBox, supervisedBBoxes);
        if (supervisedBBoxIdx != -1) { 

          bboxImg = drawBBox(bboxImg, currYOLOBBoxVecPtr->measurement_, cv::Scalar(255, 0, 0));
          bboxImg = drawBBox(bboxImg, supervisedBBoxes[supervisedBBoxIdx], cv::Scalar(0, 255, 0));
          
          IVBoundingBox ivBoundingBox(currTime, *currYOLOBBoxVecPtr, supervisedBBoxes[supervisedBBoxIdx], currImgPtr);
          ivBoundingBox.err_ = bboxVecErrFunc(ivBoundingBox.yoloVec_.measurement_, ivBoundingBox.gt_); // TODO fix the constructor
          cv::Mat imgPatch; 
          if (patchExtractor(ivBoundingBox.yoloVec_.measurement_, ivBoundingBox.gt_, *(ivBoundingBox.imgPtr_), imgPatch)) { 
            ivBoundingBox.imgPatchPtr_ = std::make_shared<cv::Mat>(imgPatch);
            // cv::imwrite("test.png", *(ivBoundingBox.imgPatchPtr_));
            ivBoundingBoxes_.push_back(ivBoundingBox);
            // lazy at creating new variable; should be outside the loop
            cv::imwrite(outBBoxImgDir_ + std::to_string(bboxIdx) + ".png", bboxImg); 
          }
        }
        ++bboxIdx;
      }
    }
  }

  void dumpIVBBoxes_() {
    std::ofstream ofileNames;
    ofileNames.open(outDir_ + "names.txt", std::ios::trunc);
    for (const auto& ivBoundingBox : ivBoundingBoxes_) {
      ofileNames << std::to_string(uid_) << endl;
      cv::imwrite(outImgDir_ + std::to_string(uid_) + ".png", *(ivBoundingBox.imgPatchPtr_));
      cv::Mat imgDebug = ivBoundingBox.imgPtr_->clone();
      imgDebug = drawBBox(imgDebug, ivBoundingBox.yoloVec_.measurement_, cv::Scalar(255, 0, 0));
      imgDebug = drawBBox(imgDebug, ivBoundingBox.gt_,                   cv::Scalar(0, 255, 0));
      // cv::imwrite("/robodata/taijing/uncertainty-aware-perception/dataset/cones/bboxImages/" + std::to_string(uid_) + ".png", imgDebug);

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
    ofileNames.close();
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
  void createDataset(bool sync) {
    cout << "start creating dataset..." << endl;
    uid_ = 0;
    prepareOutputDirs_();
    cout << "prepared output directory..." << endl;
    for (size_t bagIdx = 0; bagIdx < bagnames_.size(); ++bagIdx) {
      IVBBox<T>::prepare_(bagIdx, sync);
      cout << "prepared for bagfile " << bagIdx << endl;
      interpolatePosesByOdom(stampedOdoms_, stampedPoses_);
      cout << "stampedPoses_ size after interpolatePosesByOdom: " << stampedPoses_.size() << endl;
      vector<Timestamp> targetTimestamps;
      for (const auto& stampedImage: stampedImages_) {
        targetTimestamps.push_back(stampedImage.first);
      }
      auto ret = interpolatePosesByTime(targetTimestamps, stampedPoses_);
      // auto ret0 = (size_t) std::get<0>(ret);
      // auto ret1 = (size_t) std::get<1>(ret);
      cout << "start time idx: " << std::get<0>(ret) << "; end time idx: " << std::get<1>(ret) << endl;
      toCSV("debug/uncertainty_husky_2022-11-15-16-08-18.csv", stampedPoses_);
      exit(0);
      cout << "start dataset generation and association...." << endl;
      getIVBoundingBoxArr_(targetTimestamps);
      cout << "start writing dataset...." << endl;
      dumpIVBBoxes_();
      cout << "finish writing dataset...." << endl;

      cout << endl << "----- final summary -----" << endl;
      cout << "images size:"     << stampedImages_.size() << endl;;
      cout << "yoloBBox size:"   << stampedYOLOBBoxVecs_.size() << endl;
      cout << "Odom size:"       << stampedOdoms_.size() << endl;
      cout << "Pose size:"       << stampedPoses_.size() << endl;
      cout << "cluster size:"    << clusters_.size() << endl;
      cout << "annotation size:" << annotations_.size() << endl;
      cout << "ivBoundingBoxes_ size:" << ivBoundingBoxes_.size() << endl << endl;;

      // Eigen::Matrix<T, Eigen::Dynamic, 4> X = getErrMatrix_();
      // cout << "Error Matrix=" << endl;
      // cout << X << endl << endl;
      // cout << "Cov Matrix=" << endl;
      // cout << getCovMat(X) << endl;

      // X = getNormalizedErrMatrix_();
      // cout << "Normalized Error Matrix=" << endl;
      // cout << X << endl << endl;
      // cout << "Normalized Cov Matrix=" << endl;
      // cout << std::setprecision(2) << getCovMat(X) << endl;
    }
  }

};

}