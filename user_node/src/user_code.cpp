#include "ros/ros.h"
#include "std_msgs/String.h"
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/Ext2CM.h>
#include <hellocm_msgs/Carmaker_Out.h>

ros::Publisher pub;

void ControlCallback(const hellocm_msgs::CM2Ext::ConstPtr& msg)
{
  float present_ax = msg->CMOut.Car_ax_mpss;
  float kp = 3; // kp > 0
  float want_ax = 5;
  float error_ax = want_ax - present_ax;
  
///p제어기  
  float p_control = kp * error_ax;

  hellocm_msgs::Ext2CM exmsg;
  exmsg.VC_SwitchOn = 1; //////////TURN ON Autonomous control code
  exmsg.Ax = p_control;
  pub.publish(exmsg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "user_pkg");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/hellocm/cm2ext", 1, ControlCallback);

  pub = n.advertise<hellocm_msgs::Ext2CM>("/hellocm/ext2cm",1);

  ros::spin();

  return 0;
}
