#include "ros/ros.h"
#include "std_msgs/String.h"
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/Ext2CM.h>
#include <hellocm_msgs/CarMaker_Out.h>
#include <hellocm_msgs/CarMaker_Out.h>

#include <vector>

ros::Publisher pub;

int cnt = 0;

double targetVelocityKPH = 0;

void ControlCallback(const hellocm_msgs::CM2Ext::ConstPtr& msg)
{
    float Present_v = msg->CMOut.Car_vx_mps;
    float Kp = 50; // kp > 0
    float Target_v = 20 / 3.6;
    float Error_v = Target_v - Present_v;

    ///p제어기
    float p_control = Kp * Error_v;

    hellocm_msgs::Ext2CM exmsg;
    exmsg.VC_SwitchOn = 1;
    exmsg.GearNo = 1;
    exmsg.Ax = p_control;
    pub.publish(exmsg);
}

void ControlCallbackwithSteeringwheel(const hellocm_msgs::CM2Ext::ConstPtr& msg)
{
    float Current_v = msg->CMOut.Car_vx_mps;
    float Long_Kp = 5; // kp > 0
    float Target_v = targetVelocityKPH / 3.6;
    float Error_v = Target_v - Current_v;

    ///p제어기
    float Long_p_control = Long_Kp * Error_v;

    ///////////////////////////////////////////////

    float Lat_Kp = 5;
    float Current_angle = msg->CMOut.Car_Yaw_rad;
    float LAP_world_x = msg->RoadPath[7].x - msg->ego_GK_x;
    float LAP_world_y = msg->RoadPath[7].y - msg->ego_GK_y;
    float Target_angle = std::atan2(LAP_world_y,LAP_world_x);
    float Error_angle = Target_angle - Current_angle;

    float Lat_p_control = Lat_Kp * Error_angle;

    //  std::cout << Target_angle*180/3.141592<<"\t\t"<< Current_angle *180/3.141592<<"\t\t"<< Error_angle*180/3.141592<<"\t\t" << Lat_p_control*180/3.141592<<std::endl;
    //std::cout << LAP_world_x << "\t\t" << LAP_world_y << std::endl;
    //std::cout << std::endl;

    // e he ra di ya~
    float nCamObj = msg->camearaSensor.nObj;
    std::cout << "#Obj : " << nCamObj << " " << " cnt : " << cnt++ << "\n";

    for( size_t i = 0; i < nCamObj; i++)
    {
        std::cout << "#type : " << msg->camearaSensor.cameraSensorObj[i].classType
                  << " #id" << msg->camearaSensor.cameraSensorObj[i].ObjID << "\n";

        std::cout << "MBR bottom left x, y, z | "
                  << msg->camearaSensor.cameraSensorObj[i].bottom_left_x << ", "
                  << msg->camearaSensor.cameraSensorObj[i].bottom_left_y << ", "
                  << msg->camearaSensor.cameraSensorObj[i].bottom_left_z << "\n"
                  << "MBR top right x, y, z | "
                  << msg->camearaSensor.cameraSensorObj[i].top_right_x << ", "
                  << msg->camearaSensor.cameraSensorObj[i].top_right_y << ", "
                  << msg->camearaSensor.cameraSensorObj[i].top_right_z << "\n";

    }

        hellocm_msgs::Ext2CM exmsg;
        exmsg.VC_SwitchOn = 1;
        exmsg.GearNo = 1;
        exmsg.Ax = Long_p_control;
        exmsg.SteeringWheel = Lat_p_control;
        pub.publish(exmsg);
    }

    int main(int argc, char **argv)
    {

        ros::init(argc, argv, "user_pkg");

        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Subscriber sub = nh.subscribe("/hellocm/cm2ext", 1, ControlCallbackwithSteeringwheel);

        pub = nh.advertise<hellocm_msgs::Ext2CM>("/hellocm/ext2cm",1);

        pnh.param<double>("target_velocity_kph", targetVelocityKPH, 20.0);

        ros::spin();

        return 0;
    }
