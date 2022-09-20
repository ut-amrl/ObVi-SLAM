#include <yaml-cpp/yaml.h>

#include <unordered_map>
#include <vector>
#include <string>

#include <iv_bbox/iv_bbox_utils.h>
#include <refactoring/types/ellipsoid_utils.h>
#include <refactoring/types/vslam_obj_opt_types_refactor.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

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
  unordered_map<string, string> names_;
  vector<string> bagnames_;

  ImgArr stampedImages_;
  YOLOBBoxVecArr<float> stampedYOLOBBoxVecs_;

  void parseConfig_(const string& configPath) {
    YAML::Node nodes = YAML::LoadFile(configPath);
    string in_dir = nodes["in_dir"].Scalar();
    // TODO robustly handle filepath
    for (size_t i = 0; i < nodes["bagnames"].size(); ++i) {
      bagnames_.emplace_back(nodes["bagnames"][i].Scalar());
    }
    names_["compressedImg"] = nodes["ros_topics"]["compressed_img"].Scalar();
    names_["yoloBBox"] = nodes["ros_topics"]["yolo_bbox"].Scalar();
  }

  void saveBboxImg_(const string& path,  
                   const BbCornerPair<T>& bbCornerPair,
                   const cv::Mat& img) {
    float min_x, min_y, max_x, max_y;
    min_x = std::max((float)0,    bbCornerPair.first[0]);
    min_y = std::max((float)0,    bbCornerPair.first[1]);
    max_x = std::min((float)img.cols(), bbCornerPair.second[0]);
    max_y = std::min((float)img.rows(),  bbCornerPair.second[1]);
    cv::Mat bbox_img = img.clone();
    cv::rectangle(bbox_img, 
                 cv::Point(min_x,min_y),
                 cv::Point(max_x,max_y),
                 cv::Scalar(0, 255, 0), 2);
    cv::imwrite("test.png", bbox_img);
  }

  void clear_() {
    stampedImages_.clear();
  }

public:
  IVBBox(const CameraIntrinsics<T>& intrinsics,
         const CameraExtrinsics<T>& extrinsics,
         const string& configPath) : intrinsics_(intrinsics), extrinsics_(extrinsics) {
    parseConfig_(configPath);
  }

  ~IVBBox() = default;

  void getCornerLocationsPairList() {

  }


};

}