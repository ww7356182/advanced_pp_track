#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include<canlib.h>
#include<stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <endian.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <stack>
#include<algorithm>


//声明变量

using namespace std;
using namespace Eigen;
  int count_path;
  int angle;
  double cur_angle;
  double target_x;
  double target_y;
  int num;
  int num_;
  int count;
  int count_tar;
  double r;
  double theta1,theta2,theta3,theta4,theta_temp;
  double v1,v2,v3,v4;
  double d_min;
  int count_d;
  double d_temp;
  double x_init,y_init;
  double x,y,z;
  unsigned int flag;
  long id;
  long id_1;
  uint8_t buffer[8];
  long int msg1,msg_2,msg_3;
  int int_theta1,int_theta2,int_v1,int_v2;
  unsigned int dlc;
  int count_2;
  double d;
  double theta,target_angle;
  double roll,pitch,yaw;
  int count_init;
  double d_ym;
  double abs_theta;
  int min_ind;
  double d_min_init;
  double v_pp;
  int error_count;
  double error,sum_er;
  double sum_angle_delta,angle_delta;
  double rt_angle;
  double d_tar;
  double delta_angle_l,delta_angle_r;
  double last_angle_r,last_angle_l,angle_delta_l,angle_delta_r;
  int sec_min_ind;
  double error_ins;
  double d_curv;
  Eigen::Vector3d v_result; 
  int ins_count;
  int p_size;
   int re_count;
   stack<double> plan_x;
   stack<double> plan_y;
   std::vector<double> tar_x;
   std::vector<double> tar_y;
   bool agv_is_clear;
   int len_path;


int main(int argc, char **argv)
{
//ros节点初始化
  ros::init(argc, argv, "track_carto_curv");
  ros::NodeHandle nh; 
  ros::Publisher pub_path;  
  ros::Publisher agv_pub;
  ros::Publisher pub_path_target;
  
  ros::Time current_time;
  tf::TransformListener listener;
  ros::Time current_time_target;
  tf::Quaternion q;
  tf::StampedTransform transform;
  ros::Publisher pub_start = nh.advertise<geometry_msgs::PoseStamped>("start", 1000);
  
  x=transform.getOrigin().x();
  y=transform.getOrigin().y(); 
  q=transform.getRotation(); 
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

  geometry_msgs::PoseStamped start_msg;
  start_msg.header.stamp=ros::Time::now();
  start_msg.pose.position.x=x;
  start_msg.pose.position.y=y;
  start_msg.pose.position.z=0;
  start_msg.pose.orientation.x=q.getX();
  start_msg.pose.orientation.y=q.getY();
  start_msg.pose.orientation.z=q.getZ();
  start_msg.pose.orientation.w=q.getW();
  pub_start.publish(start_msg);
  return 0;

}