#ifndef IV_BBOX_H
#define IV_BBOX_H

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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>

using std::string;
using std::vector;
using std::unordered_map;
using std::pair;
using std::cout;
using std::endl;

namespace iv_bbox {

typedef vector<pair<ros::Time, std::shared_ptr<cv::Mat>>> ImgArr;
typedef vector<pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> PclArr;

// http://planning.cs.uiuc.edu/node103.html
void R_to_roll_pitch_yaw(const Eigen::Matrix3f R, 
                         float& roll, float& pitch, float& yaw) {
    yaw  = atan2( R(1,0), R(0,0));
    pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
    roll   = atan2( R(2,1), R(2,2));
}

template <typename T>
Eigen::Matrix<T, 4, 1> getMeasurementError(const Eigen::Matrix<T, 4, 1>& measurement, 
                                           const Eigen::Matrix<T, 4, 1>& gt) {
    return measurement - gt;
}

Eigen::MatrixXf getCovMat(const Eigen::MatrixXf& X) {
    float n = X.rows();
    return 1/(n-1) * X.transpose() * X;
}

float getNNL() { // TODO
    return 0.0;
}

void openBagfile(const string& bagfile_path, rosbag::Bag& bag) {
    bag.open(bagfile_path, rosbag::bagmode::Read);
}

void closeBagfile(rosbag::Bag& bag) {bag.close();}

template <typename topic_type>
void loadTopic(const rosbag::Bag& bag, const string& topic_name,
                 vector<topic_type>& messages) {
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    for (const rosbag::MessageInstance& m : view) {
        typename topic_type::ConstPtr ptr = m.instantiate<topic_type>();
        if (ptr != nullptr) {
            messages.push_back(*ptr);
        } else {
            LOG(INFO) << "get an empty message under " << topic_name << endl;
        }
    }
}

void parseCompressedImage(const string& bagfile, const string& topic_name, 
                          ImgArr& images) {
    vector<sensor_msgs::CompressedImage> rgbs;
    rosbag::Bag bag;
    openBagfile(bagfile, bag);
    loadTopic(bag, topic_name, rgbs);
    for (const sensor_msgs::CompressedImage& rgb : rgbs) {
        cv::Mat img = cv::imdecode(cv::Mat(rgb.data), cv::IMREAD_UNCHANGED);
        images.emplace_back(ros::Time(rgb.header.stamp.sec, rgb.header.stamp.nsec), 
                            std::make_shared<cv::Mat>(img));
    }
    closeBagfile(bag);
}

void parsePointCloud(const string& bagfile, const string& topic_name, PclArr stampedPointclouds) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;
    rosbag::Bag bag;
    openBagfile(bagfile, bag);
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::PointCloud2::ConstPtr msgPtr = m.instantiate<sensor_msgs::PointCloud2>();
        if (msgPtr != nullptr) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msgPtr, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msgPtr, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msgPtr, "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                cloudPtr->points.emplace_back(*iter_x, *iter_y, *iter_z);
            }
            stampedPointclouds.emplace_back(ros::Time(msgPtr->header.stamp.sec, msgPtr->header.stamp.nsec), cloudPtr);
        }
    }
    closeBagfile(bag);
}

}
#endif  // IV_BBOX_H