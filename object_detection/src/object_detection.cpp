#include "object_detection/object_detection.h"

#include <string>

ObjectDetection::ObjectDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    :nh(nh)
    ,pnh(pnh)
{
    // set subscriber & publisher
    std::string inputTopicName = "";
    pnh.param<std::string>("point_cloud_topic_name", inputTopicName, "/pointcloud/os1");
    subPtCld = nh.subscribe(inputTopicName, 1, &ObjectDetection::pointCloudCallback, this);

    pnh.param<std::string>("camera_rsi_topic_name", inputTopicName, "/vds_node_localhost_2211/image_raw/compressed");
    subCompImg = nh.subscribe(inputTopicName, 1, &ObjectDetection::compressedImageCallback, this);

    pnh.param<std::string>("cmnode_topic_name", inputTopicName, "/hellocm/cm2ext");
    subCMNode = nh.subscribe(inputTopicName, 1, &ObjectDetection::CMNodeCallback, this);

    std::string outputTopicName = "";
    pnh.param<std::string>("object_topic_name", outputTopicName, "/object_detection/objects");
    pubObjects = nh.advertise<object_msgs::Objects>(outputTopicName, 1);

    pubTmpPcd = nh.advertise<sensor_msgs::PointCloud2>("/object_detection/tmp_cloud", 1);

    // set pointcloud RT matrix
    pcdRTMatrix = Eigen::Matrix4d::Identity();
    pnh.param<double>("lidar_rsi_position_m_x", pcdRTMatrix(0, 3), 2.4);
    pnh.param<double>("lidar_rsi_position_m_y", pcdRTMatrix(1, 3), 0.0);
    pnh.param<double>("lidar_rsi_position_m_z", pcdRTMatrix(2, 3), 2.2);

    double rotX = 0, rotY = 0, rotZ = 0;
    pnh.param<double>("lidar_rsi_rotation_deg_x", rotX, 0.0);
    pnh.param<double>("lidar_rsi_rotation_deg_x", rotY, 0.0);
    pnh.param<double>("lidar_rsi_rotation_deg_x", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    Eigen::Matrix3d RotX, RotY, RotZ;
    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    pcdRTMatrix.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set raw camera intrinsic matrix
    rawCamIntrinsicMatrix = Eigen::Matrix3d::Identity();
    pnh.param<double>("focal_length_x", rawCamIntrinsicMatrix(0, 0), 1814.0);
    pnh.param<double>("focal_length_y", rawCamIntrinsicMatrix(1, 1), 1814.0);
    pnh.param<double>("focal_center_x", rawCamIntrinsicMatrix(0, 2), 320.0);
    pnh.param<double>("focal_center_y", rawCamIntrinsicMatrix(1, 2), 240.0);

    // set raw camera RT matrix
    rawCamRTMatrix = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_rsi_position_m_x", rawCamRTMatrix(0, 3), 2.4);
    pnh.param<double>("camera_rsi_position_m_y", rawCamRTMatrix(1, 3), 0.0);
    pnh.param<double>("camera_rsi_position_m_z", rawCamRTMatrix(2, 3), 2.2);

    pnh.param<double>("camera_rsi_rotation_deg_x", rotX, 90.0);
    pnh.param<double>("camera_rsi_rotation_deg_y", rotY, -90.0);
    pnh.param<double>("camera_rsi_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    rawCamRTMatrix.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set GT camera RT matrix
    gtCamRTMatrix = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_gt_position_m_x", gtCamRTMatrix(0, 3), 2.5);
    pnh.param<double>("camera_gt_position_m_y", gtCamRTMatrix(1, 3), 0.0);
    pnh.param<double>("camera_gt_position_m_z", gtCamRTMatrix(2, 3), 1.3);

    pnh.param<double>("camera_gt_rotation_deg_x", rotX, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_y", rotY, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    gtCamRTMatrix.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    //    std::cout << "point cloud RT matrix : \n" << pcdRTMatrix << "\n"
    //              << "raw camera RT matrix : \n" << rawCamRTMatrix << "\n"
    //              << "raw camera Intrinsic matrix : \n" << rawCamIntrinsicMatrix << "\n"
    //              << "gt camera RT matrix : \n" << gtCamRTMatrix << "\n";

    pnh.param<double>("segment_distance_threshold", segmentThreshold, 0.3);
    pnh.param<double>("roi_range_x", roiRangeX, 40.0);
    pnh.param<double>("roi_range_y", roiRangeY, 10.0);
}

void ObjectDetection::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    // convert to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2::Ptr rosCloud2Input(new sensor_msgs::PointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(*msg, *rosCloud2Input);

    // convert to car reference frame from lidar reference frame
    rosCloud2Input->header.frame_id = "Fr1A";
    pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloudInput(new  pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*rosCloud2Input, *pclCloudInput);
    pcl::transformPointCloud(*pclCloudInput, *pclCloudInput, pcdRTMatrix);

    // set roi
    pcl::PointCloud<pcl::PointXYZI>::Ptr pclRoi(new pcl::PointCloud<pcl::PointXYZI>);
    setPointCloudRoi(pclCloudInput, pclRoi);


    sensor_msgs::PointCloud2::Ptr rosCloud2Output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*pclRoi, *rosCloud2Output);
    rosCloud2Output->header.frame_id = "Fr1A";
    pubTmpPcd.publish(*rosCloud2Output);

    ROS_INFO("converted!");
}

void ObjectDetection::compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{

}

void ObjectDetection::CMNodeCallback(const hellocm_msgs::CM2Ext::ConstPtr &msg)
{

}

void ObjectDetection::setPointCloudRoi(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr dst)
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-roiRangeY, roiRangeY);
    pass.setFilterLimitsNegative(false);

    pass.setInputCloud(src);
    pass.filter(*dst);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(-roiRangeX, roiRangeX);
    pass.setFilterLimitsNegative(false);

    pass.setInputCloud(dst);
    pass.filter(*dst);

    // floor segmentation
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (segmentThreshold);

    seg.setInputCloud(dst);
    seg.segment(*inliers, *coeff);

    pcl::ExtractIndices<pcl::PointXYZI> ext;
    ext.setNegative(true);
    ext.setIndices(inliers);
    ext.setInputCloud(dst);
    ext.filter(*dst);
}
