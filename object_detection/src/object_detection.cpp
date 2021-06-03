#include "object_detection/object_detection.h"

#include <cv_bridge/cv_bridge.h>

std::string TABLE_MESSAGE[7] = {
    "CAR",
    "TRUCK",
    "BICYCLE",
    "PEDESTRIAN",
    "TRAFFIC_SIGN",
    "TRAFFIC_LIGHT",
    "UNKNOWN"
};

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

    // visualization
    pubPcdClusters = nh.advertise<sensor_msgs::PointCloud2>("/object_detection/cloud_clusters", 1);
    pubCenteroids = nh.advertise<visualization_msgs::MarkerArray>("/object_detection/centeroids", 1);

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
    RTCamRaw = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_rsi_position_m_x", RTCamRaw(0, 3), 2.4);
    pnh.param<double>("camera_rsi_position_m_y", RTCamRaw(1, 3), 0.0);
    pnh.param<double>("camera_rsi_position_m_z", RTCamRaw(2, 3), 2.2);

    pnh.param<double>("camera_rsi_rotation_deg_x", rotX, 90.0);
    pnh.param<double>("camera_rsi_rotation_deg_y", rotY, -90.0);
    pnh.param<double>("camera_rsi_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    RTCamRaw.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set GT camera RT matrix
    RTCamGT = Eigen::Matrix4d::Identity();
    pnh.param<double>("camera_gt_position_m_x", RTCamGT(0, 3), 2.5);
    pnh.param<double>("camera_gt_position_m_y", RTCamGT(1, 3), 0.0);
    pnh.param<double>("camera_gt_position_m_z", RTCamGT(2, 3), 1.3);

    pnh.param<double>("camera_gt_rotation_deg_x", rotX, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_y", rotY, 0.0);
    pnh.param<double>("camera_gt_rotation_deg_z", rotZ, 0.0);
    rotX = deg2rad(rotX);
    rotY = deg2rad(rotY);
    rotZ = deg2rad(rotZ);

    RotX << 1, 0, 0, 0, std::cos(rotX), -std::sin(rotX), 0, std::sin(rotX), std::cos(rotX);
    RotY << std::cos(rotY), 0, std::sin(rotY), 0, 1, 0, -std::sin(rotY), 0, std::cos(rotY);
    RotZ << std::cos(rotZ), -std::sin(rotZ), 0, std::sin(rotZ), std::cos(rotZ), 0, 0, 0, 1;
    RTCamGT.block(0, 0, 3, 3) = RotZ * RotY * RotX;

    // set other RT matrix
    RTCamRaw2CamGT = RTCamRaw.inverse() * RTCamGT;
    RTCamRaw2PointCloud = RTCamRaw.inverse() * RTPointCloud;

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

    // clustering
    int numCluster = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pclClusters;
    if(pclRoi->size() > 0)
    {
        numCluster = clustering(pclRoi, pclClusters);
    }

    // get centeroid of each cluster;
    objectsCurr.objects.clear();
    objectsCurr.objects.reserve(pclClusters.size());
    for (size_t c = 0; c < pclClusters.size(); c++)
    {
        object_msgs::Object object;
        pcl::PointCloud<pcl::PointXYZI>& cluster = pclClusters[c];
        double centeroid_x = 0;
        double centeroid_y = 0;
        double centeroid_z = 0;
        int cnt = 0;

        for (size_t p = 0; p < cluster.size(); p++)
        {
            cnt++;
            centeroid_x += cluster[p].x;
            centeroid_y += cluster[p].y;
            centeroid_z += cluster[p].z;
        }

        object.centeroid.x = centeroid_x / cnt;
        object.centeroid.y = centeroid_y / cnt;
        object.centeroid.z = centeroid_z / cnt;

        objectsCurr.objects.push_back(object);
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

    objectsPrev = objectsCurr;

    // get elapsed time
    std::chrono::time_point<std::chrono::high_resolution_clock> timeCurr = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timeCurr - timePrev).count();
    timePrev = timeCurr;
    ROS_INFO("elpased %lf[ms]", elapsed / 1000);
}

void ObjectDetection::compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat camRawImg = cvPtr->image;

    size_t nCamObj = camGT.cameraSensorObj.size();

    boxes.clear();
    boxes.reserve(nCamObj);
    boxLabels.clear();
    boxLabels.reserve(nCamObj);
    boxID.clear();
    boxID.reserve(nCamObj);

    // bounding box projection
    for( size_t i = 0; i < nCamObj; i++)
    {
        hellocm_msgs::CameraSensorObj& obj = camGT.cameraSensorObj[i];

        if(obj.classType < 4)
        {
            // bounding box
            Eigen::MatrixXd bl(4, 1), tr(4, 1);
            bl << obj.bottom_left_x, obj.bottom_left_y, obj.bottom_left_z, 1;
            tr << obj.top_right_x, obj.top_right_y, obj.top_right_z, 1;

            // projection to image
            Eigen::MatrixXd blImg(3, 1), trImg(3, 1);
            blImg = imageProjection(bl, RTCamRaw2CamGT);
            trImg = imageProjection(tr, RTCamRaw2CamGT);


            Rect box(Point(blImg(0, 0), trImg(1, 0)), Point(trImg(0, 0), blImg(1, 0)));
            if (obj.classType == static_cast<int>(PEDESTRIAN))
            {
                box.x -= 5;
                box.width += 10;
            }
            boxes.push_back(box);
            boxLabels.push_back(obj.classType);
            boxID.push_back(obj.ObjID);

//            // draw
//            double fontScale = 0.7;
//            int fontFace = CV_FONT_HERSHEY_PLAIN;
//            int thickness = 1;
//            int baseLine = 0;

//            std::string message = std::to_string(i) + " " + TABLE_MESSAGE[obj.classType];
//            Size backgroundSize = getTextSize(message, fontFace, fontScale, thickness, &baseLine);
//            Rect background(box.x, box.y - backgroundSize.height, backgroundSize.width, backgroundSize.height);

//            rectangle(camRawImg, background, Scalar(0, 0, 255), -1);
//            rectangle(camRawImg, box, Scalar(0, 0, 255), 2);
//            putText(camRawImg, message, box.tl(), fontFace, fontScale, Scalar(0,0,0), 1);
        }
    }

    //draw bounding boxes
    for (size_t i = 0; i < boxes.size(); i++)
    {
        const Rect& box = boxes[i];
        double fontScale = 0.7;
        int fontFace = CV_FONT_HERSHEY_PLAIN;
        int thickness = 1;
        int baseLine = 0;

        std::string message = std::to_string(i) + " " + TABLE_MESSAGE[boxLabels[i]];
        Size backgroundSize = getTextSize(message, fontFace, fontScale, thickness, &baseLine);
        Rect background(box.x, box.y - backgroundSize.height, backgroundSize.width, backgroundSize.height);

        rectangle(camRawImg, background, Scalar(0, 0, 255), -1);
        rectangle(camRawImg, box, Scalar(0, 0, 255), 2);
        putText(camRawImg, message, box.tl(), fontFace, fontScale, Scalar(0,0,0), 1);
    }

    // draw cluster centeroids
    for (size_t c = 0; c < objectsPrev.objects.size(); c++)
    {
        object_msgs::Object &obj = objectsPrev.objects[c];

        Eigen::MatrixXd centeroid(4, 1);
        centeroid << obj.centeroid.x, obj.centeroid.y, obj.centeroid.z, 1;

        // projection
        Eigen::MatrixXd centeroidImg(3, 1);
        centeroidImg = imageProjection(centeroid, RTCamRaw.inverse());

        // draw
        if(centeroid(0,0) > 0)
        {
            circle(camRawImg, Point(centeroidImg(0, 0), centeroidImg(1, 0)), 3, Scalar(0, 255, 0), -1);
        }
    }

    // labeling
    for (size_t i = 0; i < objectsCurr.objects.size(); i++)
    {
        object_msgs::Object& obj = objectsCurr.objects[i];

        Eigen::MatrixXd centeroid(4, 1);
        centeroid << obj.centeroid.x, obj.centeroid.y, obj.centeroid.z, 1;

        // projection
        Eigen::MatrixXd centeroidImg(3, 1);
        centeroidImg = imageProjection(centeroid, RTCamRaw.inverse());
        Point center(centeroidImg(0, 0), centeroidImg(1, 0));

        bool check = false;
        for (size_t b = 0; b < boxes.size(); b++)
        {
            Rect & box = boxes[b];
            if(box.contains(center))
            {
                obj.labels = boxLabels[b];
                obj.id = boxID[b];
                check = true;
            }
        }

        // no box contian centeroid than unknown
        if(!check)
        {
            obj.id = -1;
            obj.labels = 6;
        }
    }

    // visualization labels
    visualization_msgs::MarkerArray markerCenteroids;

    // delete all markers
    visualization_msgs::Marker markerD;
    markerD.header.frame_id = "Fr1A";
    markerD.action = visualization_msgs::Marker::DELETEALL;
    markerCenteroids.markers.push_back(markerD);
    pubCenteroids.publish(markerCenteroids);

    for (size_t i = 0; i < objectsCurr.objects.size(); i++)
    {
        object_msgs::Object& obj = objectsCurr.objects[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "Fr1A";
        marker.text = TABLE_MESSAGE[obj.labels];
        marker.color.a = 1.0;
        marker.scale.z = 1.0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = obj.centeroid.x;
        marker.pose.position.y = obj.centeroid.y;
        marker.pose.position.z = obj.centeroid.z + 2;

        markerCenteroids.markers.push_back(marker);
    }
    pubCenteroids.publish(markerCenteroids);

    imshow("raw image", camRawImg);
    waitKey(1);
}

void ObjectDetection::CMNodeCallback(const hellocm_msgs::CM2Ext::ConstPtr &msg)
{
    camGT = msg->camearaSensor;
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

Eigen::MatrixXd ObjectDetection::imageProjection(Eigen::MatrixXd from, Eigen::MatrixXd RT)
{
    Eigen::MatrixXd targetCoordinate(4, 1);
    targetCoordinate = RT * from;
    targetCoordinate /= targetCoordinate(2, 0);
    return IntrinsicRawCam * targetCoordinate.block(0, 0, 3, 1);
}

