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

    pubPcdRoi = nh.advertise<sensor_msgs::PointCloud2>("/object_detection/cloud_roi", 1);
    pubPcdClusters = nh.advertise<sensor_msgs::PointCloud2>("/object_detection/cloud_clusters", 1);

    // set pointcloud RT matrix
    RTPointCloud = Eigen::Matrix4d::Identity();
    pnh.param<double>("lidar_rsi_position_m_x", RTPointCloud(0, 3), 2.4);
    pnh.param<double>("lidar_rsi_position_m_y", RTPointCloud(1, 3), 0.0);
    pnh.param<double>("lidar_rsi_position_m_z", RTPointCloud(2, 3), 2.2);

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
    RTPointCloud.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set raw camera intrinsic matrix
    IntrinsicRawCam = Eigen::Matrix3d::Identity();
    pnh.param<double>("focal_length_x", IntrinsicRawCam(0, 0), 1814.0);
    pnh.param<double>("focal_length_y", IntrinsicRawCam(1, 1), 1814.0);
    pnh.param<double>("focal_center_x", IntrinsicRawCam(0, 2), 320.0);
    pnh.param<double>("focal_center_y", IntrinsicRawCam(1, 2), 240.0);

    // set raw camera RT matrix
    RTRawCam = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_rsi_position_m_x", RTRawCam(0, 3), 2.4);
    pnh.param<double>("camera_rsi_position_m_y", RTRawCam(1, 3), 0.0);
    pnh.param<double>("camera_rsi_position_m_z", RTRawCam(2, 3), 2.2);

    pnh.param<double>("camera_rsi_rotation_deg_x", rotX, 90.0);
    pnh.param<double>("camera_rsi_rotation_deg_y", rotY, -90.0);
    pnh.param<double>("camera_rsi_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    RTRawCam.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set GT camera RT matrix
    RTGTCam = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_gt_position_m_x", RTGTCam(0, 3), 2.5);
    pnh.param<double>("camera_gt_position_m_y", RTGTCam(1, 3), 0.0);
    pnh.param<double>("camera_gt_position_m_z", RTGTCam(2, 3), 1.3);

    pnh.param<double>("camera_gt_rotation_deg_x", rotX, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_y", rotY, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    RTGTCam.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    //    std::cout << "point cloud RT matrix : \n" << RTPointCloud << "\n"
    //              << "raw camera RT matrix : \n" << RTRawCam << "\n"
    //              << "raw camera Intrinsic matrix : \n" << IntrinsicRawCam << "\n"
    //              << "gt camera RT matrix : \n" << RTGTCam << "\n";

    // set Lidar parameters
    pnh.param<double>("segment_distance_threshold", segmentThreshold, 0.3);
    pnh.param<double>("roi_range_x", roiRangeX, 40.0);
    pnh.param<double>("roi_range_y", roiRangeY, 10.0);
    pnh.param<double>("roi_range_z", roiRangeZ, 4);

    pnh.param<double>("cluster_tolerance", clusterTolerance, 0.1);
    pnh.param<int>("min_cluster_size", minClusterSize, 4);
    pnh.param<int>("max_cluster_size", maxClusterSize, 100);

    timePrev = std::chrono::high_resolution_clock::now();
}

void ObjectDetection::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    // convert to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2::Ptr pcdInput(new sensor_msgs::PointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(*msg, *pcdInput);

    // convert to car reference frame from lidar reference frame
    pcl::PointCloud<pcl::PointXYZI>::Ptr pclInput(new  pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pcdInput, *pclInput);
    pcl::transformPointCloud(*pclInput, *pclInput, RTPointCloud);

    // set roi
    pcl::PointCloud<pcl::PointXYZI>::Ptr pclRoi(new pcl::PointCloud<pcl::PointXYZI>);
    setPointCloudRoi(pclInput, pclRoi);

//    // visualization Roi of PointCloud
//    sensor_msgs::PointCloud2::Ptr pcdOutput(new sensor_msgs::PointCloud2);
//    pcl::toROSMsg(*pclRoi, *pcdOutput);
//    pcdOutput->header.frame_id = "Fr1A";
//    pubPcdRoi.publish(*pcdOutput);

    // clustering
    int numCluster = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pclClusters;
    if(pclRoi->size() > 0)
    {
        numCluster = clustering(pclRoi, pclClusters);
    }

    // merge clusters into single point cloud for visualization
    pcl::PointCloud<pcl::PointXYZI>::Ptr pclClustersMerged(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < pclClusters.size(); i++)
    {
        *pclClustersMerged += pclClusters[i];
    }

    // visualization Clusters
    sensor_msgs::PointCloud2::Ptr pcdClustersMerged(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*pclClustersMerged, *pcdClustersMerged);
    pcdClustersMerged->header.frame_id = "Fr1A";
    pubPcdClusters.publish(*pcdClustersMerged);

    // get elapsed time
    std::chrono::time_point<std::chrono::high_resolution_clock> timeCurr = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timeCurr - timePrev).count();
    timePrev = timeCurr;
    ROS_INFO("elpased %lf[ms]", elapsed / 1000);
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
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-roiRangeZ, roiRangeZ);
    pass.setFilterLimitsNegative(false);

    pass.setInputCloud(src);
    pass.filter(*dst);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-roiRangeY, roiRangeY);
    pass.setFilterLimitsNegative(false);

    pass.setInputCloud(dst);
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

int ObjectDetection::clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters)
{
    clusters.clear();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    std::vector<pcl::PointIndices> clusterIndices;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(clusterIndices);

    clusters.reserve(clusterIndices.size());
    int id = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it < clusterIndices.end(); it++)
    {
        pcl::PointCloud<pcl::PointXYZI> cluster;
        cluster.reserve(it->indices.size());
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit < it->indices.end(); pit++)
        {
            pcl::PointXYZI pt = input->at(static_cast<size_t>(*pit));
            pt.intensity = id;
            cluster.push_back(pt);
        }
        clusters.emplace_back(std::move(cluster));
        id++;
    }
    return id;
}
