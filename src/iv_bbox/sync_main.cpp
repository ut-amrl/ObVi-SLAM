#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iv_bbox/iv_bbox_utils.h>

// e.g. /robodata/taijing/uncertainty-aware-perception/bags/test.bag
DEFINE_string(bagfile,   "", "");
// e.g. /robodata/taijing/uncertainty-aware-perception/LeGO-LOAM/uncertainty_husky_2022-11-07-12-02-34.csv
DEFINE_string(posefile,  "", "");
DEFINE_string(annotfile, "", "");
using namespace vslam_types_refactor;
using namespace iv_bbox;

void cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
             vector<PCLCluster<float>>& clusters) {
    static const float kGroundPlaneHeight = -0.95;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

    std::ofstream ofile;
    ofile.open("debug/map.csv", std::ios::trunc);
    for (auto p = cloud->points.begin(); p != cloud->points.end(); p++) {
        ofile << p->x << " " << p->y << " " << p->z << endl;
    }
    ofile.close();

    for (auto p = cloud->points.begin(); p != cloud->points.end(); p++) {
        if (fabs (p->z - kGroundPlaneHeight) < 0.25) { continue; }
        new_cloud->points.push_back(*p);
    }

    ofile.open("debug/filtered_map.csv", std::ios::trunc);
    for (auto p = new_cloud->points.begin(); p != new_cloud->points.end(); p++) {
        ofile << p->x << " " << p->y << " " << p->z << endl;
    }
    ofile.close();

    new_cloud->width = new_cloud->size();
    new_cloud->height = 1;
    new_cloud->is_dense = true;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (new_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); // in meters.
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (new_cloud);
    ec.extract (cluster_indices);

    int j = 0;
   
    // iterate through each clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
            it != cluster_indices.end (); ++it) {
        PCLCluster<float> cluster;

        // iterate though each point in cluster
        for (const auto& idx : it->indices) {
            Eigen::Vector3f point_eigen((*new_cloud)[idx].x, (*new_cloud)[idx].y, (*new_cloud)[idx].z);
            cluster.pointPtrs_.push_back(std::make_shared<Eigen::Vector3f>(point_eigen));
        }
        j++;
        clusters.push_back(cluster);
    }

    ofile.open("debug/colored_cloud.csv", std::ios::trunc);
    for (const auto& cluster : clusters) {
        for (const auto& p : cluster.pointPtrs_) {
            ofile << p->x() << " " << p->y() << " " << p->z() << endl;
        }
    }
    ofile.close();
}

void process_clusters(vector<PCLCluster<float>>& clusters, 
                      const string& annotfile, 
                      vslam_types_refactor::Pose3D<float> pose) {
    for (auto& cluster : clusters) {
        cluster.centroid_ = Eigen::Vector3f(0,0,0);
        for (const auto& pointPtr : cluster.pointPtrs_) {
            cluster.centroid_ += (*pointPtr);
        }
        cluster.centroid_ /= cluster.pointPtrs_.size();
        // cout << "cluster.centroid_: " << cluster.centroid_.transpose() << endl;
    }

    vector<Annotation<float>> annotations;
    parseAnnotation(annotfile, annotations);

    for (size_t i = 0; i < clusters.size(); ++i) { toCSV("debug/clusters/"+std::to_string(i)+".csv", clusters[i]); }

    vector<PCLCluster<float>> new_clusters;
    for (const auto& annotation : annotations) {

        PCLCluster<float>* bestClusterPtr;
        float minDist = std::numeric_limits<float>::max();
        bool found = false;

        for (auto& cluster : clusters) {
            const Eigen::Vector3f& point_in_baselink = cluster.centroid_;
            Eigen::Vector3f point_in_world = pose.orientation_ * point_in_baselink + pose.transl_;
            Eigen::Vector3f point_in_box   = annotation.state_.pose_.orientation_ * (point_in_world - annotation.state_.pose_.transl_);
            if (abs(point_in_box[0]) < annotation.state_.dimensions_[0]/2.0 + 0.5
             && abs(point_in_box[1]) < annotation.state_.dimensions_[1]/2.0 + 0.5
             && abs(point_in_box[2]) < annotation.state_.dimensions_[2]/2.0 + 0.5) {
                if (point_in_box.norm() < minDist) {
                    found = true;
                    minDist = point_in_box.norm();
                    bestClusterPtr = &cluster;
                }
            }
        }
        if (found) {
            const Eigen::Vector3f& point_in_baselink = bestClusterPtr->centroid_;
            Eigen::Vector3f point_in_world = pose.orientation_ * point_in_baselink + pose.transl_;
            Eigen::Vector3f point_in_box   = annotation.state_.pose_.orientation_ * (point_in_world - annotation.state_.pose_.transl_);
            cout << "best point_in_box: " << point_in_box.transpose() << endl;
            bestClusterPtr->label_ = annotation.label_;
            
            // bestClusterPtr->state_.dimensions_ << (float)0.3, (float)0.3, (float)0.5;
            float xmin, xmax, ymin, ymax, zmin, zmax;
            xmin = ymin = zmin =  1000;
            xmax = ymax = zmax = -1000;
            for (const auto& p : bestClusterPtr->pointPtrs_) {
                xmin = p->x() < xmin ? p->x() : xmin;
                ymin = p->y() < ymin ? p->y() : ymin;
                zmin = p->z() < zmin ? p->z() : zmin;
                xmax = p->x() > xmax ? p->x() : xmax;
                ymax = p->y() > ymax ? p->y() : ymax;
                zmax = p->z() > zmax ? p->z() : zmax;
            }
            bestClusterPtr->state_.dimensions_ << (float)(xmax-xmin), (float)(ymax-ymin), (float)(zmax-zmin);
            cout << "dimension: " << bestClusterPtr->state_.dimensions_.transpose() << endl;

            bestClusterPtr->state_.pose_.transl_ = bestClusterPtr->centroid_;
            bestClusterPtr->state_.pose_.orientation_ =  Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0));
            new_clusters.push_back(*bestClusterPtr);
        }
    }
    clusters = new_clusters;
}

void run(int argc, char **argv, 
         const string& bagfile, const string& posefile, const string& annotfile) {
    CameraIntrinsics<float> intrinsics(730.5822, 729.8109, 609.5004, 539.4311);
    CameraExtrinsics<float> extrinsics;
    extrinsics.transl_ = Eigen::Vector3f(0.0625, 0.05078125, 0);
    extrinsics.orientation_ = Eigen::Quaternionf(0.464909, -0.5409106, -0.5394464, -0.4475183);
    
    Eigen::Affine3f robot_to_cam_tf;
    robot_to_cam_tf = Eigen::Translation3f(Eigen::Vector3f(0.0625, 0.05078125, 0)) 
                    * Eigen::Quaternionf(0.464909, -0.5409106, -0.5394464, -0.4475183);
    
    robot_to_cam_tf = robot_to_cam_tf.inverse();
    float roll, pitch, yaw;
    R_to_roll_pitch_yaw(robot_to_cam_tf.linear(), roll, pitch, yaw);
    cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << endl;
    cout << "translation:\n" << robot_to_cam_tf.translation().transpose() << endl;
    cout << "roation:\n" << robot_to_cam_tf.linear() << endl;
    extrinsics.transl_ = robot_to_cam_tf.translation();
    extrinsics.orientation_ = Eigen::Quaternionf(robot_to_cam_tf.linear());

    extrinsics.transl_ = Eigen::Vector3f(0.01, 0.1, -0.1);
    // extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0)) 
    //                         * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
    extrinsics.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(0.99549089, 0.0, 0.09485717, 0.0)) 
                            * Eigen::Quaternionf(0.5, 0.5, -0.5, 0.5).inverse();
    R_to_roll_pitch_yaw(extrinsics.orientation_.matrix(), roll, pitch, yaw);
    cout << "hellooooo: roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << endl;
    Eigen::Matrix3f R;
    roll_pitch_yaw_to_R(0, 0.25, 0, R);
    cout << Eigen::Quaternionf(R).coeffs().transpose() << endl;

    Pose3D<float> robotPose;
    robotPose.transl_ << (float)0.0, (float)0.0, (float)0.0;
    robotPose.orientation_ = Eigen::AngleAxisf(Eigen::Quaternionf(1, 0, 0, 0));

    rosbag::Bag bag;
    openBagfile(bagfile, bag);
    
    ImgArr stampedImgPtrs;
    PclArr stampedPclPtrs;
    Pose3DArr<float> stampedPosePtrs, stampedOdomPtrs;
    parseCompressedImage(bag, "/stereo/left/image_raw/compressed", stampedImgPtrs);
    parsePointCloud     (bag, "/ouster/lidar_packets",             stampedPclPtrs);
    assertm(stampedImgPtrs.size() == stampedPclPtrs.size(), "[run] Expecting image topics and lidar topics have the same #frames");
    vector<Timestamp> targetTimes;
    for (const auto stampedImgPtr : stampedImgPtrs) {
        targetTimes.push_back(stampedImgPtr.first);
    }
    parseOdom           (bag, "/husky_velocity_controller/odom",  stampedOdomPtrs);
    parsePoseFile(posefile, stampedPosePtrs);
    interpolatePosesByOdom(stampedOdomPtrs, stampedPosePtrs);
    auto ret = interpolatePosesByTime(targetTimes, stampedPosePtrs);

    size_t startIdx = std::get<0>(ret);
    size_t endIdx   = std::get<1>(ret);
    cout << "startIdx: " << startIdx << ", endIdx: " << endIdx << endl;
    stampedImgPtrs = ImgArr(stampedImgPtrs.begin()+startIdx, stampedImgPtrs.begin()+endIdx+1);
    stampedPclPtrs = PclArr(stampedPclPtrs.begin()+startIdx, stampedPclPtrs.begin()+endIdx+1);

    ros::init(argc, argv, "input");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Publisher pub_img     = nh.advertise<sensor_msgs::Image>("/viz/image", 1);
    ros::Publisher pub_pose    = nh.advertise<geometry_msgs::PoseStamped>("/viz/pose", 1);
    ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/viz/cluster", 1);

    size_t idx = 0;
    cout << "start publishing" << endl;
    while (ros::ok()) {
        if (idx >= stampedPosePtrs.size()) { 
            closeBagfile(bag);
            exit(0); 
        }
        ros::Time timestamp(stampedPosePtrs[idx].first.first, stampedPosePtrs[idx].first.second);

        vector<BbCorners<float>> bboxes;
        vector<PCLCluster<float>> clusters;
        cluster(stampedPclPtrs[idx].second, clusters);
        process_clusters(clusters, annotfile, *(stampedPosePtrs[idx].second));

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        size_t color_idx = 0;
        size_t colored_cloud_width = 0;

        for (const auto& cluster : clusters) {
            uint8_t r = (uint8_t) (color_idx % 2) * 255;
            uint8_t b = (uint8_t) (color_idx % 4) * 63;
            uint8_t g = 0;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            colored_cloud_width += cluster.pointPtrs_.size();
            for (const auto& pointPtr : cluster.pointPtrs_) {
                pcl::PointXYZRGB colored_point;
                colored_point.x = pointPtr->x();
                colored_point.y = pointPtr->y();
                colored_point.z = pointPtr->z();
                colored_point.rgb = rgb;
                colored_cloud->push_back(colored_point);
            }
            ++color_idx;

            BbCornerPair<float> supervisedBBoxPair = getCornerLocationsPair(cluster.state_, robotPose, extrinsics, intrinsics.camera_mat);
            auto corner = cornerLocationsPairToVector(supervisedBBoxPair);
            cout << corner[0] << "," << corner[2] << "," << corner[1] << "," << corner[3] << endl;
            bboxes.push_back(cornerLocationsPairToVector(supervisedBBoxPair));
        }
        cout << "idx: " << idx << "; clusters size = " << clusters.size() << endl;
        cv::Mat bboxImg = drawBBoxes(*(stampedImgPtrs[idx].second), bboxes, cv::Scalar(0, 255, 0));
        cv::imwrite("debug/images/"+std::to_string(idx)+".png", bboxImg);

        // publish pointcloud clusters
        colored_cloud->width = colored_cloud_width;
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*colored_cloud, cluster_msg);
        cluster_msg.header.seq = idx;
        cluster_msg.header.stamp = timestamp;
        cluster_msg.header.frame_id = "baselink";
        pub_cluster.publish(cluster_msg);

        // publish image
        cv_bridge::CvImage cv_img;
        cv_img.header.seq = idx;
        cv_img.header.stamp = timestamp;
        cv_img.header.frame_id = "baseline";
        cv_img.encoding = "8UC3";
        cv_img.image = bboxImg;
        sensor_msgs::ImagePtr img_msg = cv_img.toImageMsg();
        pub_img.publish(*img_msg);

        // publish pose
        const vslam_types_refactor::Pose3D<float>& pose = *(stampedPosePtrs[idx].second);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.seq = idx;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = "baselink";
        // pose_msg.pose.position.x    = pose.transl_.x();
        // pose_msg.pose.position.y    = pose.transl_.y();
        // pose_msg.pose.position.z    = pose.transl_.z();
        pose_msg.pose.position.x    = 0.0;
        pose_msg.pose.position.y    = 0.0;
        pose_msg.pose.position.z    = 0.0;
        Eigen::Quaternionf quat = Eigen::Quaternionf(pose.orientation_);
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        pub_pose.publish(pose_msg);

        ++idx;
        ros::spinOnce();
        rate.sleep();
        cout << "ready for next iter" << endl;
    }
    
#if 0
    for (size_t i = 0; i < stampedPosePtrs.size(); ++i) {
        vector<BbCorners<float>> bboxes;
        vector<PCLCluster<float>> clusters;
        cluster(stampedPclPtrs[i].second, clusters);
        process_clusters(clusters, annotfile, *(stampedPosePtrs[i].second));
        for (const auto& cluster : clusters) {
            BbCornerPair<float> supervisedBBoxPair = getCornerLocationsPair(cluster.state_, robotPose, extrinsics, intrinsics.camera_mat);
            auto corner = cornerLocationsPairToVector(supervisedBBoxPair);
            cout << corner[0] << "," << corner[2] << "," << corner[1] << "," << corner[3] << endl;
            bboxes.push_back(cornerLocationsPairToVector(supervisedBBoxPair));
        }
        cout << "i: " << i << "; clusters size = " << clusters.size() << endl;
        drawBBoxes(*(stampedImgPtrs[i].second), bboxes, "debug/images/"+std::to_string(i)+".png");
    }
    closeBagfile(bag);
#endif

}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    run(argc, argv, FLAGS_bagfile, FLAGS_posefile, FLAGS_annotfile);
}