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

void laser_Callback(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    int size=scan_msg->ranges.size();
    std::vector<double> r;
    for(int i=0;i<size;i++)
    {
        r.push_back(scan_msg->ranges[i]);
    }
    std::sort(&r[0],&r[size-1]);
    if(r[0]>1.5)
    {
        agv_is_clear=true;
    }
    else
    {
        agv_is_clear=false;
    }
}

void path_Callback(const nav_msgs::PathConstPtr &path_msg)
{
  tar_x.clear();
  tar_y.clear();
  len_path=path_msg->poses.size();
  for(int i=len_path-1;i>=0;i--)
  {
    tar_x.push_back(path_msg->poses[i].pose.position.x);
    tar_y.push_back(path_msg->poses[i].pose.position.y);
  }
}
    

void find_min_ind(const double& x_init,const double& y_init)
 {
  d_min=sqrt((tar_x[0]-x_init)*(tar_x[0]-x_init)+(tar_y[0]-y_init)*(tar_y[0]-y_init));
  min_ind=0;
 for(count_d=1;count_d<len_path;count_d++)
  {
    d_temp=sqrt((tar_x[count_d]-x_init)*(tar_x[count_d]-x_init)+(tar_y[count_d]-y_init)*(tar_y[count_d]-y_init));
    if(d_temp<d_min)
    {
      d_min=d_temp;
      min_ind=count_d;
    }
  }
    return;
  }

int main(int argc, char **argv)
{
//ros节点初始化
  ros::init(argc, argv, "track_carto_curv");
  ros::NodeHandle n_; 
  ros::Publisher pub_path;  
  ros::Publisher agv_pub;
  ros::Publisher pub_path_target;
  ros::Subscriber laser = n_.subscribe("scan", 10, laser_Callback);
  ros::Subscriber plan_path = n_.subscribe("path", 1, path_Callback);
  
  visualization_msgs::Marker marker;
  visualization_msgs::Marker marker2;
  uint32_t shape;
  visualization_msgs::MarkerArray array;
  geometry_msgs::PoseStamped pose_sta;
  geometry_msgs::PoseStamped pose_target;
  geometry_msgs::Twist msg_twist;
  nav_msgs::Path msg_path;
  nav_msgs::Path msg_target;
  ros::Time current_time;
  tf::TransformListener listener;
  ros::Time current_time_target;
  tf::Quaternion q;
//设置控制频率
  ros::Rate loop_rate(10);

//定义消息发布器
  pub_path= n_.advertise<nav_msgs::Path>("/path_real", 1,true);
  pub_path_target=n_.advertise<nav_msgs::Path>("/path_target",1,true);
  agv_pub=n_.advertise<visualization_msgs::MarkerArray>("target_point", 1);

//变量赋初始值
  agv_is_clear=false;
  num_=0;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  count_init=0;
  error_ins=0;
  p_size=len_path-1;
  re_count=0;
  sum_er=0;
  error_count=0;

//目标轨迹消息赋值
 for(count_path=0;count_path<p_size+1;count_path++)
 {
   pose_target.pose.position.x=tar_x[count_path];
   pose_target.pose.position.y=tar_y[count_path];
   pose_target.pose.position.z=0;
   pose_target.pose.orientation.x=0;
   pose_target.pose.orientation.y=0;
   pose_target.pose.orientation.z=0;
   pose_target.pose.orientation.w=1;
   pose_target.header.stamp=ros::Time::now();
   pose_target.header.frame_id="map";
   msg_target.poses.push_back(pose_target);
   //printf("pose:%f",pose_target.pose.position.x);
   msg_target.header.frame_id="map";
   msg_target.header.stamp=ros::Time::now();
 }

while (ros::ok())
  {
    agv_is_clear=false;
    ros::spinOnce();    
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/laser",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)   
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  
//获取AGV实时位姿
  x=transform.getOrigin().x();
  y=transform.getOrigin().y(); 
  q=transform.getRotation(); 
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

//目标点初始化
  find_min_ind(x,y);
  num_=min_ind;
  
  v_pp=0.6;
  d_ym=1;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  d_tar=sqrt((target_x-tar_x[min_ind])*(target_x-tar_x[min_ind])+(target_y-tar_y[min_ind])*(target_y-tar_y[min_ind]));
    while ((d<d_ym)&& ros::ok())
  {
     num_=num_+1;
     if(num_>p_size)
     {
        num_=p_size;
        break;
     }
      target_x=tar_x[num_];
      target_y=tar_y[num_];
     d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  }
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  //将目标点转换到车体坐标系
  Eigen::Vector3d v(target_x-x,target_y-y,0);
  Eigen::AngleAxisd r_v(-yaw,Vector3d(0,0,1));
  Eigen::Matrix3d R=r_v.matrix();
  v_result=R*v;
  
  msg_twist.linear.x=v_pp;
  msg_twist.angular.z=2*v_pp*v_result[1]/(d*d);
 

//轨迹终点停车
  if(num_==p_size && d<0.5)
  {
    msg_twist.linear.x=0;
    msg_twist.linear.y=0;
    msg_twist.linear.z=0;
    msg_twist.angular.x=0;
    msg_twist.angular.y=0;
    msg_twist.angular.z=0;
    //ROS_INFO("arrive");     
  }
//计算AGV前轮转角
 if(msg_twist.linear.x!=0 && agv_is_clear)
 {
 r=fabs(msg_twist.linear.x/msg_twist.angular.z);
 theta2=atan(1.42/(2*r-1.16))*57.29578;
 theta1=atan(1.42/(2*r+1.16))*57.29578;
 if(msg_twist.angular.z<0)
 {
   theta1=-theta1;
   theta2=-theta2;
 }
 if(msg_twist.angular.z>0)
 {
    theta_temp=theta1;
    theta1=theta2;
    theta2=theta_temp;
 }
  /*if(msg_twist.angular.z==0)
 {
    theta1=0;
    theta2=0;
 }*/

//计算AGV前轮转速
 v2=fabs(msg_twist.angular.z*sqrt(pow(r-0.58,2)+pow(0.71,2)));
 v1=fabs(msg_twist.angular.z*sqrt(pow(r+0.58,2)+pow(0.71,2)));
 }
 else
 {
   theta1=0;
   theta2=0;
   v1=0;
   v2=0;
 }


 angle_delta_l=fabs(theta1-last_angle_l);
 angle_delta_r=fabs(theta2-last_angle_r);
 sum_angle_delta=sum_angle_delta+angle_delta_l;
 angle_delta=sum_angle_delta/error_count;
 last_angle_l=theta1;
 last_angle_r=theta2;


//设置CAN通信
  canHandle h;
 canInitializeLibrary();
  h=canOpenChannel(0,canWANT_EXCLUSIVE);
  if(h != canOK)
    {
        char msg[64];
        canGetErrorText((canStatus)h,msg,sizeof(msg));
        fprintf(stderr,"canopenchannel fialied(%s)\n",msg);
        exit(1);

    }
  canSetBusParams(h,BAUD_500K,0,0,0,0,0);
  canBusOn(h);

//将转速转角存入buffer中
 int_theta1=round(theta1*100);
 int_theta2=round(theta2*100);
 int_v1=round(v1*100*52.9);
 int_v2=round(v2*100*52.9);
 int16_t can_angle;
 int16_t can_v;
 //can_angle=htobe16(can_angle);

 if(msg_twist.angular.z<=0)
 {
    can_angle=int_theta2;
    can_angle=htobe16(can_angle);
    *((int16_t *)&(buffer[2]))=can_angle;
    can_v=int_v1;
    //can_v=htobe16(can_v);
    //ROS_INFO("%d", can_v);
    //*((int16_t *)&(buffer[2])) = can_v;
    buffer[0]=can_v/256;
    buffer[1]=can_v%256;
 }
 else
 {
    can_angle=int_theta1;
    can_angle=htobe16(can_angle);
    *(int16_t*)&buffer[2]=can_angle;
    can_v=int_v2;
    buffer[0]=can_v/256;
    buffer[1]=can_v%256;
 }
 //ROS_INFO("%d, %d", buffer[2], buffer[3]);
if(msg_twist.linear.x==0)
{
  buffer[0]=0;
  buffer[1]=0;
  buffer[2]=0;
  buffer[3]=0;
}

 buffer[4]=1;
 buffer[5]=0; 
 buffer[6]=0;
 buffer[7]=0;
 id=0x311;
 dlc=8;
 flag=0;

//发布AGV实时轨迹
 pose_sta.pose.position.x=x;
 pose_sta.pose.position.y=y;
 pose_sta.pose.position.z=0;
 pose_sta.pose.orientation.x=q.getX();
 pose_sta.pose.orientation.y=q.getY();
 pose_sta.pose.orientation.z=q.getZ();
 pose_sta.pose.orientation.w=q.getW();
 pose_sta.header.stamp=ros::Time::now();
 pose_sta.header.frame_id="map";
 msg_path.poses.push_back(pose_sta);
 msg_path.header.frame_id="map";
 msg_path.header.stamp=ros::Time::now();

//发布AGV车身姿态，目标点
  array.markers.clear();
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker2.header.frame_id = "map";
  marker2.header.stamp = ros::Time::now();
  marker.ns = "b";
  marker2.ns = "b";
  marker2.id = 0;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker2.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker2.action = visualization_msgs::Marker::ADD;

  marker2.pose.position.x = x;
  marker2.pose.position.y = y;
  marker2.pose.position.z = 0;
  marker2.pose.orientation.x = q.getX();
  marker2.pose.orientation.y = q.getY();
  marker2.pose.orientation.z =q.getZ();
  marker2.pose.orientation.w = q.getW();
 
  marker.pose.position.x = target_x;
  marker.pose.position.y = target_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;
  marker2.scale.x = 0.85;
  marker2.scale.y = 0.7;
  marker2.scale.z = 0.4;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker2.color.r = 0.0f;
  marker2.color.g = 1.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker2.lifetime = ros::Duration();
  array.markers.push_back(marker);
  array.markers.push_back(marker2);
//发布消息
 pub_path.publish(msg_path);
 pub_path_target.publish(msg_target);
 agv_pub.publish(array);

//发送can报文
 canWrite(h,id,&buffer,dlc,flag);
 canWriteSync(h,500);
 canBusOff(h);   
 canClose(h);

//循环控制
 loop_rate.sleep();
  }
  return 0;

}