#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <chrono>

#include <ros/ros.h>
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/Ext2CM.h>
#include <object_msgs/Objects.h>
#include <object_msgs/Object.h>

#include "pid_controller.h"

enum ON_LANE_CHECK
{
    ON_LANE_LEFT = 1,
    ON_LANE_CURRENT = 0,
    ON_LANE_RIGHT = -1
};

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

class DecisionMaker
{
public:

    DecisionMaker(ros::NodeHandle& nh, ros::NodeHandle& pnh);
private:

    // ROS Callbacks
    void objectsCallback(const object_msgs::Objects::ConstPtr& msg);
    void cm2extCallback(const hellocm_msgs::CM2Ext::ConstPtr& msg);

    // ROS Service
    ros::NodeHandle& nh;
    ros::NodeHandle& pnh;

    ros::Subscriber subCM2Ext;
    ros::Subscriber subObjects;

    ros::Publisher pubExt2CM;

    object_msgs::Objects objectsCurr;
    hellocm_msgs::CM2Ext cm2ext;
    hellocm_msgs::Ext2CM ext2cm;

    // ROS param
    double velocityMaxKPH = 0;

    // Functions
    bool onLaneCheck(const ON_LANE_CHECK onLane, double x, double y);

    // Variables
    PIDController latSteer;
    PIDController longVelocity;
    PIDController longDistance;

    std::chrono::time_point<std::chrono::high_resolution_clock> timePrevCM2Ext;

    double elapsedTime = 0;
    bool changeing = false;
};

#endif // DECISION_MAKER_H
