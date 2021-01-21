#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "demo");
  ros::NodeHandle n_; 
  ros::Publisher pub_path_demo; 
  geometry_msgs::PoseStamped pose_demo;
  nav_msgs::Path path_demo;
  ros::Time time_demo;
  time_demo=ros::Time::now();
  pose_demo.header.stamp=time_demo;
  pose_demo.header.frame_id="odom";
  path_demo.header.stamp=time_demo;
  path_demo.header.frame_id="odom";
  int angle,num;
  num=0;
  double demo_x,demo_y;
  double x[30];
  double y[30];
  for(angle=30;angle<=120;angle=angle+5)
  {
    x[num]=50*sin(angle/57.29578);
    y[num]=50*cos(angle/57.29578);
    pose_demo.pose.position.x=x[num];
    pose_demo.pose.position.y=y[num];
    path_demo.poses.push_back(pose_demo);
    num++;
  }
  pub_path_demo= n_.advertise<nav_msgs::Path>("/path_demo", 1,true);
  ROS_INFO("msg");
  while (ros::ok())
{
  pub_path_demo.publish(path_demo);
  ros::spin();
}
  return 0;
}