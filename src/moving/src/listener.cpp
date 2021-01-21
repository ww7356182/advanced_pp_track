#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
geometry_msgs::Twist msg;
int count = 0;
double t;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  /*ros::NodeHandle m;
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
 ROS_INFO("send msg");*/
 /*ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      msg.linear.x=4;
     msg.angular.z=pow(-1,count)*0.5;
    ros::spinOnce();
    count++;
    chatter_pub.publish(msg);
    ROS_INFO("send msg");*/
      tf::Quaternion quat;
  geometry_msgs::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("yaw:%f",yaw*57.29578);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 1, chatterCallback);
  ros::spin();
  return 0;
}
