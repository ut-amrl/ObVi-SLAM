#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <unordered_map>
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
    CameraId camera_id; // TODO redundant, delete me & fix constructor
    FeatureId feature_id; // TODO redundant, delete me & fix constructor
    Vector2f measurement;
    RobotPose robot_pose;
    float depth;    

    FeatureProjector() {}
    FeatureProjector(const CameraId camera_id, const FeatureId feature_id, const Vector2f& measurement, const RobotPose& robot_pose) {
        this->camera_id = camera_id;
        this->feature_id = feature_id;
        this->measurement = measurement;
        this->robot_pose = robot_pose;
    }
    FeatureProjector(const CameraId camera_id, const FeatureId feature_id, const Vector2f& measurement, const RobotPose& robot_pose, const float depth) {
        this->camera_id = camera_id;
        this->feature_id = feature_id;
        this->measurement = measurement;
        this->depth = depth;
        this->robot_pose = robot_pose;
    }
};

/**
 * TODO only use one measurement for initial estimates; 
 * may need to use mean of multiple measurements instead
 * 
 * @param dataset_path 
 * @param features return
 */
void LoadFeatures(const string& dataset_path, 
                  unordered_map<FeatureId, FeatureProjector>& features) {
    
    // iterate through all measurements
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
        
        FeatureId feature_id;
        CameraId camera1_id, camera2_id;
        float measurement_x1, measurement_y1, measurement_x2, measurement_y2;
        while ( getline(data_file_stream, line) ) {
            stringstream ss_feature(line);
            ss_feature >> feature_id >> camera1_id >> measurement_x1 >> measurement_y1 >> camera2_id >> measurement_x2 >> measurement_y2;
            features[feature_id] = FeatureProjector(frame_id, feature_id, Vector2f(measurement_x1, measurement_y1), robot_pose);
        }
    }

    // iterate through all depths associated with the left camera measuremnts
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
        
        FeatureId feature_id;
        float depth;
        while ( getline(data_file_stream, line) ) {
            stringstream ss_depth(line);
            ss_depth >> feature_id >> depth;
            if (features[feature_id].feature_id == feature_id) {
                features[feature_id].depth = depth;
            }
        }
    }
}

void EstimatePoints3D(const string& dataset_path, 
                      const unordered_map<FeatureId, FeatureProjector>& features, 
                                            const CameraIntrinsics& intrinsics, 
                                            const CameraExtrinsics& extrinsics, 
                                            const bool dumpToFile) {
    vector<pair<FeatureId,Vector3f>> points;
    for (const auto& feature : features) {
        Vector3f point = Unproject(feature.second.measurement, intrinsics, extrinsics, feature.second.robot_pose, feature.second.depth);
        points.emplace_back(feature.first, point);
    }
    
    if (dumpToFile) {
        ofstream fp;
        fp.open(dataset_path + features_path, fstream::trunc);
        if (!fp.is_open()) {
            LOG(FATAL) << "[dataset_path + features_path] Fail to load: " << dataset_path + features_path;
            return;
        }
        for (const auto& point : points) {
            fp << point.first << " " 
               << point.second.x() << " " 
               << point.second.y() << " "
               << point.second.z() << endl;
        }
        fp.close();
    } else { // debug
        for (const auto& point : points) {
            printf("%ld: point - %.2f, %.2f, %.2f\n; depth - %.2f\n", 
                    point.first, point.second.x(), point.second.y(), point.second.x(), features.at(point.first).depth);
        }
    }
}

void LoadFeaturesByFrameId(const string& dataset_path, const size_t frame_id, unordered_map<FeatureId, FeatureProjector>& features) {
    std::string measurement_path = dataset_path + std::to_string(frame_id) + ".txt";
    std::ifstream measurement_file_stream(measurement_path);
    if (measurement_file_stream.fail()) {
        LOG(FATAL) << "[LoadFeaturesByFrameId] Failed to load measurement file: " << measurement_path;
        return;
    }

    string line;
    getline(measurement_file_stream, line); // skip frame_id
    
    getline(measurement_file_stream, line);
    float x, y, z, qx, qy, qz, qw;
    stringstream ss_pose(line);
    ss_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
    Vector3f loc(x, y, z);
    Quaternionf angle_q(qw, qx, qy, qz);
    angle_q.normalize();
    AngleAxisf angle(angle_q);
    RobotPose robot_pose(frame_id, loc, angle);

    FeatureId feature_id;
    CameraId camera1_id, camera2_id;
    float measurement_x1, measurement_y1, measurement_x2, measurement_y2;
    while (getline(measurement_file_stream, line)) {
        stringstream ss_feature(line);
        ss_feature >> feature_id >> camera1_id >> measurement_x1 >> measurement_y1 >> camera2_id >> measurement_x2 >> measurement_y2;
        features[feature_id] = FeatureProjector(frame_id, feature_id, Vector2f(measurement_x1, measurement_y1), robot_pose);
    }

    std::string depth_path = dataset_path + "depths/" + std::to_string(frame_id) + ".txt";
    std::ifstream depth_file_stream(depth_path);
    if (depth_file_stream.fail()) {
        LOG(FATAL) << "[LoadFeaturesByFrameId] Failed to load depth file: " << depth_path;
        return;
    }

    getline(depth_file_stream, line); // skip frame id
    getline(depth_file_stream, line); // skip camera pose
    float depth;
    while (getline(depth_file_stream, line)) {
        stringstream ss_depth(line);
        ss_depth >> feature_id >> depth;
        if (features[feature_id].feature_id == feature_id) {
            features[feature_id].depth = depth;
        }
    }
}

int main(int argc, char** argv) {
    InitGoogleLogging(argv[0]);
    ParseCommandLineFlags(&argc, &argv, true);

    unordered_map<FeatureId, FeatureProjector> features_map;
    LoadFeatures(FLAGS_dataset_path, features_map);
    // LoadFeaturesByFrameId(FLAGS_dataset_path, 1, features_map);
    // intrinsics: project 3D point from camera's baselink frame to 2D measurement
    unordered_map<CameraId, CameraIntrinsics> intrinsics_map;  
    LoadCameraIntrinsics(FLAGS_dataset_path + "calibration/camera_matrix.txt", intrinsics_map);
    // extrinsics: transform from the robot frame to the camera frame
    // TODO: double check this transformation (it takes an inverse in structured_main)
    unordered_map<CameraId, CameraExtrinsics> extrinsics_map;  
    LoadCameraExtrinsics(FLAGS_dataset_path + "calibration/extrinsics.txt", extrinsics_map);
    // debug
    EstimatePoints3D(FLAGS_dataset_path, features_map, intrinsics_map[1], extrinsics_map[1], true);
}