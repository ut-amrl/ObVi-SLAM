#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <experimental/filesystem>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "vslam_unproject.h"
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
const string velocities_path = "velocities/";
}

struct FeatureProjector {
    FeatureId feature_id; 
    FrameId frame_id;
    Vector2f measurement;
    RobotPose velocity; // TODO delete me
    RobotPose robot_pose;
    float depth;    

    FeatureProjector() {}
    FeatureProjector(const CameraId frame_id, const FeatureId feature_id, const Vector2f& measurement) 
        : frame_id(frame_id), feature_id(feature_id), measurement(measurement) {}
    FeatureProjector(const CameraId frame_id, const FeatureId feature_id, const Vector2f& measurement, const RobotPose& robot_pose)
        : frame_id(frame_id), feature_id(feature_id), measurement(measurement), robot_pose(robot_pose) {}
    FeatureProjector(const CameraId frame_id, const FeatureId feature_id, const Vector2f& measurement, const RobotPose& robot_pose, const float depth)
        : frame_id(frame_id), feature_id(feature_id), measurement(measurement), robot_pose(robot_pose), depth(depth) {}
    
    FeatureProjector(const CameraId frame_id, 
                     const FeatureId feature_id, 
                     const Vector2f& measurement, 
                     const RobotPose& prev_robot_pose,
                     const RobotPose& velocity) {
      this->frame_id = frame_id;
      this->feature_id = feature_id;
      this->measurement = measurement;
      Eigen::Affine3f robotToWorldTF = velocity.RobotToWorldTF() * prev_robot_pose.RobotToWorldTF();
      this->robot_pose = RobotPose(prev_robot_pose.frame_idx+1, 
                         robotToWorldTF.translation(),
                         AngleAxisf(robotToWorldTF.rotation()) );
    }
    FeatureProjector(const CameraId frame_id, 
                     const FeatureId feature_id, 
                     const Vector2f& measurement, 
                     const RobotPose& prev_robot_pose,
                     const RobotPose& velocity,
                     const float depth) {
      this->frame_id = frame_id;
      this->feature_id = feature_id;
      this->measurement = measurement;
      Eigen::Affine3f robotToWorldTF = velocity.RobotToWorldTF() * prev_robot_pose.RobotToWorldTF();
      this->robot_pose = RobotPose(prev_robot_pose.frame_idx+1, 
                         robotToWorldTF.translation(),
                         AngleAxisf(robotToWorldTF.rotation()) );
      this->depth = depth;
    }

    void SetRobotPose(const RobotPose& prev_robot_pose, const RobotPose& velocity) {
        Eigen::Affine3f robotToWorldTF = velocity.RobotToWorldTF() * prev_robot_pose.RobotToWorldTF();
        this->robot_pose = RobotPose(prev_robot_pose.frame_idx+1, 
                            robotToWorldTF.translation(),
                            AngleAxisf(robotToWorldTF.rotation()) );
    }

    friend std::ostream& operator<<(std::ostream& o,
                                  const FeatureProjector& p) {
        o << "frame_id: " << p.frame_id << "\trobopose: " << p.robot_pose 
          << "\tfeature_id: " << p.feature_id << " measurement: " << p.measurement.transpose();
        return o;
    }
    
};

// TODO need to figure out a cleanner transform
RobotPose getCurrentRobotPose(const RobotPose& prev_robot_pose, const RobotPose& velocity) {
    Eigen::Affine3f velocity_matrix = velocity.RobotToWorldTF();
    Eigen::Affine3f prev_pose_matrix = prev_robot_pose.RobotToWorldTF();
    velocity_matrix = velocity_matrix.inverse();
    prev_pose_matrix = prev_pose_matrix.inverse();
    Eigen::Affine3f curr_pose_matrix = velocity_matrix * prev_pose_matrix;
    curr_pose_matrix = curr_pose_matrix.inverse();
    RobotPose pose = RobotPose(prev_robot_pose.frame_idx+1, 
                     curr_pose_matrix.translation(),
                     AngleAxisf(curr_pose_matrix.rotation()) );
    return pose;
}

void LoadVelocityToAbsPose(const string& dataset_path, unordered_map<FeatureId, FeatureProjector>& features) {
    // load all relative poses (velocity)
    unordered_map<FrameId, RobotPose> velocities_by_frame_id;
    for (const auto& entry : directory_iterator(path(dataset_path + velocities_path))) {
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
        FrameId frame_id;
        stringstream ss_frame_id(line);
        ss_frame_id >> frame_id;
        getline(data_file_stream, line); 
        stringstream ss_robot_pose(line);
        float x, y, z, qx, qy, qz, qw;
        ss_robot_pose >> x >> y >> z >> qx >> qy >> qz >> qw;
        Vector3f translation(x, y, z);
        AngleAxisf rotation_a(Quaternionf(qw, qx, qy, qz));
        RobotPose velocity(frame_id, translation, rotation_a);
        velocities_by_frame_id[frame_id] = velocity;
    }
    // convert all relative poses to absolute ones
    size_t nframes = velocities_by_frame_id.size() + 1;
    unordered_map<FrameId, RobotPose> poses_by_frame_id;
    // NOTE hardcode first pose
    poses_by_frame_id[1] = RobotPose(1, Vector3f(-0.00654, -0.00277, 0.965), AngleAxisf(Quaternionf(1, 0.00226, 0.000729, -0.00036)));
    for (size_t curr_frame_id = 2; curr_frame_id <= nframes; ++curr_frame_id) {
        size_t prev_frame_id = curr_frame_id - 1;
        poses_by_frame_id[curr_frame_id] = getCurrentRobotPose(poses_by_frame_id[prev_frame_id], 
                                                               velocities_by_frame_id[curr_frame_id]);
    }
    // update absolute poses in features
    for (auto& feature : features) {
        size_t frame_id = feature.second.frame_id;
        feature.second.robot_pose = poses_by_frame_id[frame_id];
    }
}

void LoadDepths(const string& dataset_path, 
                unordered_map<FeatureId, FeatureProjector>& features) {
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
    FrameId frame_id;
    stringstream ss_frame_id(line);
    ss_frame_id >> frame_id;
    getline(data_file_stream, line); // skip the second line
    
    FeatureId feature_id;
    float depth;
    while ( getline(data_file_stream, line) ) {
        stringstream ss_depth(line);
        ss_depth >> feature_id >> depth;
        if (features[feature_id].frame_id == frame_id) {
            features[feature_id].depth = depth;
        }
    }
  }
}

/**
 * TODO only use one measurement for initial estimates; 
 * may need to use mean of multiple measurements instead
 * 
 * @param dataset_path 
 * @param features return
 */
void LoadFeaturesWithAbsPoses(const string& dataset_path, 
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
    LoadDepths(dataset_path, features);
}

void LoadFeaturesWithRelPoses(const string& dataset_path, 
                  unordered_map<FeatureId, FeatureProjector>& features) {
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
        getline(data_file_stream, line); // skip the absolute pose line
        
        FeatureId feature_id;
        CameraId camera1_id, camera2_id;
        float measurement_x1, measurement_y1, measurement_x2, measurement_y2;
        while ( getline(data_file_stream, line) ) {
            stringstream ss_feature(line);
            ss_feature >> feature_id >> camera1_id >> measurement_x1 >> measurement_y1 >> camera2_id >> measurement_x2 >> measurement_y2;
            features[feature_id] = FeatureProjector(frame_id, feature_id, Vector2f(measurement_x1, measurement_y1));
        }
    }

    // iterate through all depths associated with the left camera measuremnts
    LoadDepths(dataset_path, features);

  // iterate through all velocity to convert relative pose to the map frame
    LoadVelocityToAbsPose(dataset_path, features);
}

void EstimatePoints3D(const string& dataset_path, 
                      const unordered_map<FeatureId, FeatureProjector>& features, 
                                            const CameraIntrinsics& intrinsics, 
                                            const CameraExtrinsics& extrinsics, 
                                            const bool dumpToFile) {
    vector<pair<FeatureId,Vector3f>> points;
    for (const auto& feature : features) {
        FeatureProjector featureProjector = feature.second;
        Vector3f point = Unproject(feature.second.measurement, intrinsics, extrinsics, feature.second.robot_pose, feature.second.depth);
        points.emplace_back(feature.first, point);
        if (!dumpToFile) { // debug
            cout << feature.second << endl;
            cout << "point: " << point.transpose() << endl;
            cout << endl;
        }
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
    } 
}

void LoadDepthsByFrameId(const string& dataset_path, const FrameId frame_id,
                unordered_map<FeatureId, FeatureProjector>& features) {
    std::ifstream data_file_stream(dataset_path + depths_path + std::to_string(frame_id) + ".txt");
    if (data_file_stream.fail()) {
        LOG(FATAL) << "Failed to load: " << dataset_path + depths_path + std::to_string(frame_id) + ".txt"
                                << " are you sure this a valid data file? ";
        return;
    }
    string line;
    getline(data_file_stream, line); 
    getline(data_file_stream, line); // skip the second line
    
    FeatureId feature_id;
    float depth;
    while ( getline(data_file_stream, line) ) {
        stringstream ss_depth(line);
        ss_depth >> feature_id >> depth;
        if (features[feature_id].frame_id == frame_id) {
            features[feature_id].depth = depth;
        }
    }
}

void LoadFeaturesWithAbsPosesByFrameId(const string& dataset_path, const size_t frame_id, unordered_map<FeatureId, FeatureProjector>& features) {
    std::string measurement_path = dataset_path + std::to_string(frame_id) + ".txt";
    std::ifstream measurement_file_stream(measurement_path);
    if (measurement_file_stream.fail()) {
        LOG(FATAL) << "[LoadFeaturesWithAbsPosesByFrameId] Failed to load measurement file: " << measurement_path;
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

    // iterate through all depths associated with the left camera measuremnts
    LoadDepthsByFrameId(dataset_path, frame_id, features);
}

int main(int argc, char** argv) {
    InitGoogleLogging(argv[0]);
    ParseCommandLineFlags(&argc, &argv, true);

    unordered_map<FeatureId, FeatureProjector> features_map;
    LoadFeaturesWithRelPoses(FLAGS_dataset_path, features_map);
    // LoadFeaturesWithAbsPosesByFrameId(FLAGS_dataset_path, 500, features_map);
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