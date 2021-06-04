#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <iostream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/CompressedImage.h>
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/CameraSensor.h>
#include <hellocm_msgs/CameraSensorObj.h>
#include <visualization_msgs/MarkerArray.h>

#include <object_msgs/Objects.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/search/search.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/impl/point_types.hpp>


inline double deg2rad(double deg)
{
    return deg * M_PI / 180;
}

inline double rad2deg(double rad)
{
    return rad * 180 / M_PI;
}

// enum
enum OBJECT_TYPE
{
    CAR = 0,
    TRUCK,
    BICYCLE,
    PEDESTRIAN,
    TRAFFICSIGN,
    TRAFFICLIGHT,
    UNKNOWN

};
using namespace cv;

class ObjectDetection
{
public:
    ObjectDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:

    // ROS Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void CMNodeCallback(const hellocm_msgs::CM2Ext::ConstPtr& msg);

    // ROS Service
    ros::NodeHandle& nh;
    ros::NodeHandle& pnh;

    ros::Subscriber subPtCld;
    ros::Subscriber subCompImg;
    ros::Subscriber subCMNode;
    ros::Publisher pubObjects;

    ros::Publisher pubPcdClusters;
    ros::Publisher pubCenteroids;

    hellocm_msgs::CameraSensor camGT;
    object_msgs::Objects objectsPrev;
    object_msgs::Objects objectsCurr;

    // ROS Param
    Eigen::Matrix4d RTPointCloud;
    Eigen::Matrix4d RTCamGT;
    Eigen::Matrix4d RTCamRaw;
    Eigen::Matrix3d IntrinsicRawCam;

    Eigen::Matrix4d RTCamRaw2CamGT;
    Eigen::Matrix4d RTCamRaw2PointCloud;

    double clusterTolerance;
    int minClusterSize;
    int maxClusterSize;

    double segmentThreshold = 0;
    double roiRangeX = 0;
    double roiRangeY = 0;
    double roiRangeZ = 0;

    // Functions
    void setPointCloudRoi(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr dst);
    int clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters);
    void imageProjection(Eigen::MatrixXd& to, const Eigen::MatrixXd& RT, const Eigen::MatrixXd& from);

    // Variables
    bool centeroidUpdated = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> timePrevPcd;
    std::chrono::time_point<std::chrono::high_resolution_clock> timePrevImg;

    std::vector<Rect> boxes;
    std::vector<int> boxLabels;
    std::vector<int> boxID;
};

#endif // OBJECT_DETECTION_H
