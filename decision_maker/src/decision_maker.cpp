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

    double x = cm2ext.RoadPath[9].x - cm2ext.ego_GK_x;
    double y = cm2ext.RoadPath[9].y - cm2ext.ego_GK_y;

    double a = std::cos(cm2ext.CMOut.Car_Yaw_rad + M_PI / 2);
    double b = std::sin(cm2ext.CMOut.Car_Yaw_rad + M_PI / 2);

    double c = std::cos(cm2ext.CMOut.Car_Yaw_rad);
    double d = std::sin(cm2ext.CMOut.Car_Yaw_rad);

    geometry_msgs::Point lookAhead[3];

    lookAhead[1].x = cm2ext.RoadPath[9].x - cm2ext.ego_GK_x;
    lookAhead[1].y = cm2ext.RoadPath[9].y - cm2ext.ego_GK_y;

    lookAhead[0].x = x + 3 * a;
//    lookAhead[0].x += 7 * c;
    lookAhead[0].y = y + 3 * b;
//    lookAhead[0].y += 7 * d;

    lookAhead[2].x = x - 3 * a;
//    lookAhead[2].x += 7 * c;
    lookAhead[2].y = y - 3 * b;
//    lookAhead[2].y += 7 * d;

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

    float carLength = 4.3;
    float safeMargin = 3;

    std::vector<object_msgs::Object> objVec;
    for (size_t i = 0; i < objectsCurr.objects.size(); i++)
    {
        const object_msgs::Object& obj = objectsCurr.objects[i];

        if(onLaneCheck(ON_LANE_CURRENT, obj.centeroid.x, obj.centeroid.y))
        {
            objVec.push_back(obj);
        }
    }

    int index = 0;
    if(objVec.size() > 0)
    {
        distance = objVec[0].centeroid.x;
        for (size_t i = 0; i < objVec.size(); i++)
        {
            if(distance > objVec[i].centeroid.x)
            {
                relativeVelocity = objVec[i].velocity.x;
                distance = objVec[i].centeroid.x;
                index = i;
            }
        }

        velTarget = velCurrent + relativeVelocity;

        if(objVec[index].labels == PEDESTRIAN)
        {
            brake = true;
        }

        if(distance - carLength < 9 && velCurrent < 21 / 3.6 && !brake)
        {
            elapsedTime += durationSec;
            printf("CHANGE prepare\n");
        }
        else
        {
            elapsedTime = 0;
        }

        if(elapsedTime > 3)
        {
           changeing = true;
        }
    }

    // steer pid control
    float steerCurrent = cm2ext.CMOut.Car_Yaw_rad;
    float steerTarget = std::atan2(lookAhead[1].y, lookAhead[1].x);

    if(changeing)
    {
        steerTarget = std::atan2(lookAhead[0].y, lookAhead[0].x);
        printf("CHANGING\n");

        bool check = false;
        for (size_t i = 0; i < objectsCurr.objects.size(); i++)
        {
            const object_msgs::Object& obj = objectsCurr.objects[i];

            if(onLaneCheck(ON_LANE_RIGHT, obj.centeroid.x, obj.centeroid.y))
            {
                check = true;
            }
        }

        if(check) changeing = true;
        else  changeing = false;
    }

    float distTarget = velTarget* 3.6 - 15.0 + carLength + safeMargin;
    float distCurrent = distance;

    float steerInput = latSteer.getControlInput(steerTarget, steerCurrent, durationSec);
    float velInput = longVelocity.getControlInput(velTarget, velCurrent, durationSec);
    float distInput = longDistance.getControlInput(distTarget, distCurrent, durationSec);

    if(steerInput)

    if(distance != 0)
    {
        velInput = velInput - distInput;
        printf("dist : %f, FOLLOW : %s\n", distance - carLength, TABLE_MESSAGE[objVec[index].labels].c_str());
    }

    if(brake)
    {
        velInput = -10000.0;
        printf("BRAKE!!!!!!!!!!!!\n");
    }

    printf("steer : %f\nvel : %f\ndur : %f[sec]\n", steerCurrent, velCurrent * 3.6, durationSec);

    ext2cm.VC_SwitchOn = 1;
    ext2cm.GearNo = 1;
    ext2cm.Ax = velInput;

    ext2cm.SteeringWheel = steerInput;
    pubExt2CM.publish(ext2cm);

    printf("\n");
}

bool DecisionMaker::onLaneCheck(const ON_LANE_CHECK onLane, double x, double y)
{
    double yRangeLeft= 1.5 + 3.0 * static_cast<int>(onLane);
    double yRangeRight = -1.5 + 3.0 * static_cast<int>(onLane);

    if(onLane == ON_LANE_RIGHT)
    {
        yRangeLeft = 0;
    }
    else if(onLane == ON_LANE_LEFT)
    {
        yRangeRight = 0;
    }

    double xRangeFront = 35.0;
    double xRangeRear = -35.0;

    if(onLane == ON_LANE_CURRENT) xRangeRear = 0;

    return (yRangeRight < y && y < yRangeLeft) && (xRangeRear < x && x < xRangeFront);
}
