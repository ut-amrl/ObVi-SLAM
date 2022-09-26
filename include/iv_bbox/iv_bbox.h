#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>

#include <iv_bbox/iv_bbox_utils.h>

namespace iv_bbox {
  
using namespace vslam_types_refactor;
using std::unordered_map;
using std::string;
using std::vector;

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
    string& bagpath, string& bboxBagPath, string& slamPath) {

    const string& bagname = bagnames_[bagIdx];
    // bagpath = inDir_ + "bags/" + bagname + ".bag";
    bagpath = inDir_ + bagname + ".bag";
    bboxBagPath = inDir_ + "yolo/yolo_" + bagname + ".bag";
    slamPath = inDir_ + "LeGO-LOAM/" + bagname + ".csv"; 
    cout << bagpath << endl;
  }

  void clear_() {
    stampedImages_.clear();
    stampedYOLOBBoxVecs_.clear();
  }

  void prepare_(const size_t bagIdx) {
    clear_();
    string bagpath, bboxBagPath, slamPath;
    IVBBox<T>::preparePaths_(bagIdx, bagpath, bboxBagPath, slamPath);
    // parseCompressedImage(bagpath, names_["compressedImg"], stampedImages_);
    // parseBBox(bagpath, names_["yoloBBox"], stampedYOLOBBoxVecs_);
    parseOdom(bagpath, names_["odom"], stampedOdoms_);
    parsePoseFile(slamPath, stampedPoses_);
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
      cout << "stampedOdoms_ size: " << stampedOdoms_.size() << endl;
      cout << "stampedPoses_ size: " << stampedPoses_.size() << endl;
      interpolatePosesByOdom(stampedOdoms_, stampedPoses_);
    }
  }

};

}