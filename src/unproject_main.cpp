#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "unproject.h"
#include "vslam_io.h"

using namespace std;
using namespace std::experimental::filesystem;
using namespace Eigen;
using namespace vslam_types;
using namespace vslam_unproject;
using namespace vslam_io;
using namespace google;

DEFINE_string(dataset_path,
              "",
              "\nPath to folder containing the dataset. Structured as - \n"
              "vslam_setX/\n\tcalibration/camera_matrix.txt\n\tfeatures/"
              "features.txt\n\t0000x.txt\n\n");

namespace {
const string calibration_path = "calibration/camera_matrix.txt";
const string features_path = "features/features.txt";
const string depths_path = "depths/";
}

struct FeatureProjector {
    size_t camera_id; // TODO: use frame id for now; need to change to actual camera id
    size_t feature_id;
    Vector2f measurement;
    RobotPose robot_pose;
    float depth;    

    FeatureProjector() {}
    FeatureProjector(const size_t camera_id, const size_t feature_id, const Vector2f& measurement, const RobotPose& robot_pose) {
        this->camera_id = camera_id;
        this->feature_id = feature_id;
        this->measurement = measurement;
        this->robot_pose = robot_pose;
    }
    FeatureProjector(const size_t camera_id, const size_t feature_id, const Vector2f& measurement, const RobotPose& robot_pose, const float depth) {
        this->camera_id = camera_id;
        this->feature_id = feature_id;
        this->measurement = measurement;
        this->depth = depth;
        this->robot_pose = robot_pose;
    }
};

void LoadFeatures(const string& dataset_path, 
                  vector<FeatureProjector>* features_ptr) {
    // use feature_id as key
    vector<FeatureProjector>& features = *features_ptr;
    for (const auto& entry : directory_iterator(path(dataset_path))) {
        const auto file_extension = entry.path().extension().string();
        if (!is_regular_file(entry) || file_extension != ".txt") { continue; }
        std::ifstream data_file_stream(entry.path());
        if (data_file_stream.fail()) {
          LOG(FATAL) << "Failed to load: " << entry.path()
                     << " are you sure this a valid data file? ";
          return;
        }
        string line;
        getline(data_file_stream, line); 
        uint64 frame_id;
        stringstream ss_frame_id(line);
        ss_frame_id >> frame_id;
        --frame_id;
        getline(data_file_stream, line);
        float x, y, z, qx, qy, qz, qw;
        stringstream ss_pose(line);
        ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
        Vector3f loc(x, y, z);
        Quaternionf angle_q(qw, qx, qy, qz);
        angle_q.normalize();
        AngleAxisf angle(angle_q);
        RobotPose robot_pose(frame_id, loc, angle);
       
        size_t feature_id;
        float measurement_x1, measurement_y1, measurement_x2, measurement_y2;
        // TODO: may need to assume another file format
        while ( getline(data_file_stream, line) ) {
            stringstream ss_feature(line);
            ss_feature >> feature_id >> measurement_x1 >> measurement_y1 >> measurement_x2 >> measurement_y2;
            if (feature_id + 1 > features.size()) { features.resize(feature_id + 1); }
            features[feature_id] = FeatureProjector(frame_id, feature_id, Vector2f(measurement_x1, measurement_y1), robot_pose);
        }
    }

    for (const auto& entry : directory_iterator(path(dataset_path + depths_path))) {
        const auto file_extension = entry.path().extension().string();
        if (!is_regular_file(entry) || file_extension != ".txt") { continue; }
        std::ifstream data_file_stream(entry.path());
        if (data_file_stream.fail()) {
          LOG(FATAL) << "Failed to load: " << entry.path()
                     << " are you sure this a valid data file? ";
          return;
        }
        string line;
        getline(data_file_stream, line); 
        uint64 frame_id;
        stringstream ss_frame_id(line);
        ss_frame_id >> frame_id;
        --frame_id;
        getline(data_file_stream, line); // skip the second line
        
        size_t feature_id;
        float depth;
        while ( getline(data_file_stream, line) ) {
            stringstream ss_depth(line);
            ss_depth >> feature_id >> depth;
            if (features[feature_id].camera_id == frame_id) {
                features[feature_id].depth = depth;
            }
        }
    }

}

void EstimatePoints3D(const string& dataset_path, const vector<FeatureProjector>& features, const CameraIntrinsics& intrinsics, const CameraExtrinsics& extrinsics, const bool useMean) {
    ofstream fp;
    fp.open(dataset_path + features_path);
    if (!fp.is_open()) {
        LOG(FATAL) << "Fail to load: " << dataset_path << features_path;
        return;
    }
    vector<pair<size_t,Vector3f>> points;
    for (size_t i = 0; i < features.size(); ++i) {
        // skip features with no matching
        // estimate from a single point or multiple points 
        // size_t N = useMean? features.size() : 1;
        Vector3f point = Unproject(features[i].measurement, intrinsics, extrinsics, features[i].robot_pose, features[i].depth);
        points.emplace_back(i, point);
    }
    for (const auto& point : points) {
        fp << point.first << " " 
           << point.second.x() << " " 
           << point.second.y() << " "
           << point.second.z() << endl;
    }
    fp.close();
}

int main(int argc, char** argv) {
    InitGoogleLogging(argv[0]);
        ParseCommandLineFlags(&argc, &argv, true);

    vector<FeatureProjector> features;
    LoadFeatures(FLAGS_dataset_path, &features);
    // intrinsics: project 3D point from camera's baselink frame to 2D measurement
    CameraIntrinsics intrinsics;
    cout << "before load camera calibration" << endl;
    LoadCameraCalibration(FLAGS_dataset_path + "calibration/camera_matrix.txt", intrinsics.camera_mat);
    // extrinsics: transform from the robot frame to the camera frame
    // TODO: double check this transformation (it takes an inverse in structured_main)
    CameraExtrinsics extrinsics{Vector3f(0, 0, 0), Quaternionf(0.5, 0.5, -0.5, 0.5)};
    // debug
    // features.resize(3);
    EstimatePoints3D(FLAGS_dataset_path, features, intrinsics, extrinsics, true);

}
