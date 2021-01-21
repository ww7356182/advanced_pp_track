#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 geometry_msgs::Twist msg_twist;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::NodeHandle m;
  ros::Publisher chatter_pub =m.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  double x,y,z;
  x=msg->pose.pose.position.x;
  y=msg->pose.pose.position.y;
  z=msg->pose.pose.position.z;
  ROS_INFO("x:%f,y:%f,z:%f", x,y,z);
  double d,theta;
  d=sqrt((50-x)*(50-x)+(50-y)*(50-y));
  theta=atan((50-y)/(50-x))*57.29578;
  ROS_INFO("d:%f,theta:%f/n",d,theta);
  if(d<8)
  {
    if(d<0.05)
    {
      msg_twist.linear.x=0;
    }
    else
    {
      msg_twist.linear.x=d/2;
    }
  }
  else
  {
    msg_twist.linear.x=4;
  }
  if(theta<20)
 {
    msg_twist.angular.z=theta/40;
 }
 else
 {
   msg_twist.angular.z=0.5;
 }
 chatter_pub.publish(msg_twist);
 
  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate loop_rate(10);
  int count = 0;
  double t;
  geometry_msgs::Twist msg;
  while (ros::ok())
  {
     msg.linear.x=4;
     msg.angular.z=pow(-1,count)*0.5;
    ros::spinOnce();
    count++;
    chatter_pub.publish(msg);
    loop_rate.sleep();
  }


  return 0;
}