#include "decision_maker/decision_maker.h"

#include <string>

std::string TABLE_MESSAGE[7] = {
    "CAR",
    "TRUCK",
    "BICYCLE",
    "PEDESTRIAN",
    "TRAFFIC_SIGN",
    "TRAFFIC_LIGHT",
    "UNKNOWN"
};

DecisionMaker::DecisionMaker(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    :nh(nh)
    ,pnh(pnh)
{
    // set topic name
    std::string topic_name;
    pnh.param<std::string>("cm2ext_topic_name", topic_name, "/hellocm/cm2ext");
    subCM2Ext = nh.subscribe(topic_name, 1, &DecisionMaker::cm2extCallback, this);

    pnh.param<std::string>("objects_topic_name", topic_name, "/object_detection/objects");
    subObjects = nh.subscribe(topic_name, 1, &DecisionMaker::objectsCallback, this);

    pnh.param<std::string>("ext2cm_topic_name", topic_name, "/hellocm/ext2cm");
    pubExt2CM = nh.advertise<hellocm_msgs::Ext2CM>(topic_name, 1);

    // set PID gains
    double Kp, Ki, Kd;
    pnh.param<double>("latitude_steer_kp", Kp, 5);
    pnh.param<double>("latitude_steer_ki", Ki, 0);
    pnh.param<double>("latitude_steer_kd", Kd, 0);
    latSteer.setGains(Kp, Ki, Kd);
    latSteer.setQueueSize(10);

    pnh.param<double>("longitude_velocity_kp", Kp, 5);
    pnh.param<double>("longitude_velocity_ki", Ki, 0);
    pnh.param<double>("longitude_velocity_kd", Kd, 0);
    longVelocity.setGains(Kp, Ki, Kd);
    longVelocity.setQueueSize(10);

    pnh.param<double>("longitude_distance_kp", Kp, 5);
    pnh.param<double>("longitude_distance_ki", Ki, 0);
    pnh.param<double>("longitude_distance_kd", Kd, 0);
    longDistance.setGains(Kp, Ki, Kd);
    longDistance.setQueueSize(10);

    pnh.param<double>("velocity_max_kph", velocityMaxKPH, 40.0);

    timePrevCM2Ext = std::chrono::high_resolution_clock::now();
}

void DecisionMaker::objectsCallback(const object_msgs::Objects::ConstPtr &msg)
{
    objectsCurr = *msg;
}

void DecisionMaker::cm2extCallback(const hellocm_msgs::CM2Ext::ConstPtr &msg)
{
    cm2ext = *msg;

    std::chrono::time_point<std::chrono::high_resolution_clock> timeCurr = std::chrono::high_resolution_clock::now();
    float durationSec = std::chrono::duration_cast<std::chrono::microseconds>(timeCurr - timePrevCM2Ext).count() / 1000000.0;
    timePrevCM2Ext = timeCurr;

    // steer pid control
    float steerCurrent = cm2ext.CMOut.Car_Yaw_rad;
    float steerTarget = std::atan2(cm2ext.RoadPath[9].y - cm2ext.ego_GK_y, cm2ext.RoadPath[9].x - cm2ext.ego_GK_x);

    // velocity pid control
    float velCurrent = cm2ext.CMOut.Car_vx_mps;
    float velTarget = velocityMaxKPH / 3.6;

    if(latSteer.getError() > 0.1)
    {
        velTarget = 30 / 3.6;
    }
    else if(latSteer.getError() < -0.1)
    {
        velTarget = 20 / 3.6;
    }

    bool brake = false;
    float relativeVelocity = 0;
    float distance = 0;
    for (size_t i = 0; i < objectsCurr.objects.size(); i++)
    {
        const object_msgs::Object& obj = objectsCurr.objects[i];

        if(onLaneCheck(ON_LANE_CURRENT, obj.centeroid.x, obj.centeroid.y))
        {
            ACCon = true;
            ACCID = obj.id;

            relativeVelocity = obj.velocity.x;
            distance = obj.centeroid.x;

            velTarget = velCurrent + relativeVelocity;

            if(obj.labels == PEDESTRIAN)
            {
                brake = true;
            }

            std::cout << TABLE_MESSAGE[obj.labels] << " ahead!!\n";
        }
    }

    float carLength = 4.3;
    float safeMargin = 3;
    float distTarget = velTarget* 3.6 - 15.0 + carLength + safeMargin;
    float distCurrent = distance;

    float steerInput = latSteer.getControlInput(steerTarget, steerCurrent, durationSec);
    float velInput = longVelocity.getControlInput(velTarget, velCurrent, durationSec);
    float distInput = longDistance.getControlInput(distTarget, distCurrent, durationSec);

    if(distance != 0)
    {
        double alpha = 0.8;
        velInput = velInput - distInput;
        printf("dist : %f\n", distance - carLength);
    }

    if(brake)
    {
        velInput = -10000.0;
        ROS_INFO("BRAKE!!!!!!!!!!!!");
    }

    printf("steer : %f\nvel : %f\ndur : %f[sec]\n", steerCurrent, velCurrent * 3.6, durationSec);

    ext2cm.VC_SwitchOn = 1;
    ext2cm.GearNo = 1;
    ext2cm.Ax = velInput;

    ext2cm.SteeringWheel = steerInput;
    pubExt2CM.publish(ext2cm);

    ACCon = false;
    printf("\n");
}

bool DecisionMaker::onLaneCheck(const ON_LANE_CHECK onLane, double x, double y)
{
    double yRangeLeft= 1.5 + 3.0 * static_cast<int>(onLane);
    double yRangeRight = -1.5 + 3.0 * static_cast<int>(onLane);;

    double xRangeFront = 35.0;
    double xRangeRear = -35.0;

    if(onLane == ON_LANE_CURRENT) xRangeRear = 0;

    return (yRangeRight < y && y < yRangeLeft) && (xRangeRear < x && x < xRangeFront);
}
