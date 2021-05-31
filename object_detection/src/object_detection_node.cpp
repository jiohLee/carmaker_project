#include <ros/ros.h>

#include "object_detection/object_detection.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ObjectDetection obj(nh, pnh);

    ros::spin();

    return 0;
}
