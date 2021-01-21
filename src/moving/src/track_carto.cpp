#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include<canlib.h>
#include<stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
//#include "LinearMath/btMatrix3x3.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel");
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub_path;  
  ros::Subscriber sub_;
  ros::Subscriber sub_tra;
  geometry_msgs::PoseStamped pose_sta;
  geometry_msgs::PoseStamped pose_target;
  geometry_msgs::Twist msg_twist;
  nav_msgs::Path msg_path;
  nav_msgs::Path msg_target;
  ros::Time current_time;
  int angle;
  double cur_angle;
  double target_x;
  double target_y;
  double tar_x[1000];
  double tar_y[1000];
  int jiaodu;
  int num;
  int num_;
  int count;
  int count_tar;
  double r;
  double theta1,theta2,theta3,theta4,theta_temp;
  double v1,v2,v3,v4;
  
  unsigned int flag;
  long id;
  long id_1;
  unsigned char buffer[8];
  long int msg1,msg_2,msg_3;
  int int_theta1,int_theta2,int_v1,int_v2;
  unsigned int dlc;
  int count_2;
  double d;
  double x,y,z;
  double theta,target_angle;
  tf::TransformListener listener;
  tf::Quaternion q;
  double roll,pitch,yaw;

  
  ros::Rate loop_rate(10);
  pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pub_path= n_.advertise<nav_msgs::Path>("/path", 1,true);
    num_=0;
    num=0;
    tar_x[0]=4.1746;
    tar_y[0]=-0.1800;
    tar_x[1]=4.1746;
    tar_y[1]=-0.1800;
    tar_x[2]=8.2569;
    tar_y[2]=-0.6285;
    tar_x[3]=10.5530;
    tar_y[3]=-1.3364;
    tar_x[4]=12.1167;
    tar_y[4]=-2.5964;
    tar_x[5]=13.1040;
    tar_y[5]=-4.3114;
    tar_x[6]=13.7216;
    tar_y[6]=-7.0246;
    tar_x[7]=15.3134;//14.4249;
    tar_y[7]=-9.062;//-8.3153;
    tar_x[8]=15.3134;
    tar_y[8]=-9.062;
    tar_x[9]=16.3062;
    tar_y[9]=-9.3782;
    tar_x[10]=18.3101;
    tar_y[10]=-8.8536;
    tar_x[11]=18.3101;
    tar_y[11]=-8.8536;
    tar_x[12]=19.0298;
    tar_y[12]=-8.0619;
    //tar_x[13]=14.3179;
   // tar_y[13]=-2.5115;
    tar_x[13]=19.5058;
    tar_y[13]=-6.5782;
    tar_x[14]=19.4585;
    tar_y[14]=-4.1405;
    tar_x[15]=18.5350;
    tar_y[15]=-1.874387;
    tar_x[16]=11.7446;//16.5373;//17.5365;
    tar_y[16]=-0.1417;//-0.7367;
    tar_x[17]=11.7446;//16.5373;
    tar_y[17]=-0.1417;//-0.2976;
    //tar_x[18]=15.4814;
    //tar_y[18]=-0.2741;
    //tar_x[18]=13.9382;
   // tar_y[18]=-0.4636;
    tar_x[18]=11.7446;
    tar_y[18]=-0.1417;
    tar_x[19]=7.8864;
    tar_y[19]=-0.1629;
    tar_x[20]=3.7829;
    tar_y[20]=0.0663;
    tar_x[21]=0;
    tar_y[21]=0;
    tar_x[22]=0;
    tar_y[22]=0;
 
  while (ros::ok())
  {
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
  x=transform.getOrigin().x();
  y=transform.getOrigin().y(); 
  q=transform.getRotation();
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
    if(target_y-y<0.01 && target_x-x>0)
  {
     target_angle=0;
  }
  if(target_x-x>0 && target_y-y>0)
  {
    target_angle=atan((target_y-y)/(target_x-x))*57.29578;
  }
    if(target_x-x>0 && target_y-y<0)
  {
    target_angle=atan((target_y-y)/(target_x-x))*57.29578;
  }
  if(target_x-x<0 && target_y-y>0)
  {
    target_angle=180+atan((target_y-y)/(target_x-x))*57.29578;
  }
   if(target_x-x<0 && target_y-y<0)
  {
    target_angle=-180+atan((target_y-y)/(target_x-x))*57.29578;
  } 
  /*cur_angle=yaw*57.29578;
  if(cur_angle<0)
  {
    cur_angle=360+cur_angle;
  }*/
  theta=target_angle-yaw*57.29578;
  
  if(d<4)
  {
    if(d<0.2)
    {

      num_=num_+1;
      if(num_>22)
      {
        num_=22;
        msg_twist.linear.x=0;
        msg_twist.linear.y=0;
        msg_twist.linear.z=0;
        msg_twist.angular.x=0;
        msg_twist.angular.y=0;
        msg_twist.angular.z=0;
        ROS_INFO("arrive");
      }
      else
      {
      target_x=tar_x[num_];
      target_y=tar_y[num_];
      }
    }
    else
      {msg_twist.linear.x=0.4;}
  }
  else
  {
    msg_twist.linear.x=0.4;
  }
/*if(theta>180)
{
  msg_twist.angular.z=-0.5;
}
if(theta<-180)
{
  msg_twist.angular.z=0.5;
}*/
if(theta<40 && theta>20)
 {
    msg_twist.angular.z=3*theta/400;
 }
 if(theta<20&&theta>0)
 {
   msg_twist.angular.z=3*theta/400;
 }
 if(theta==40 || theta>40)
 {
   msg_twist.angular.z=0.3;
 }
 if(theta<0 && theta>-20)
 {
   msg_twist.angular.z=3*theta/400;
 }
 if(theta<-20 && theta>-40)
 {
    msg_twist.angular.z=3*theta/400;//-(fabs(theta)-20)/100;
 }
 if(theta==20)
 {
   msg_twist.angular.z=0.15;
 }
 if(theta==-20)
 {
   msg_twist.angular.z=-0.15;
 }
 if(theta<-40 || theta==-40)
 {
    msg_twist.angular.z=-0.3;
 }
 if(msg_twist.linear.x!=0)
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
  if(msg_twist.angular.z==0)
 {
    theta1=0;
    theta2=0;
 }
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
 int_theta1=round(theta1*100)+6000;
 int_theta2=round(theta2*100)+6000;
 int_v1=round(v1*1000);
 int_v2=round(v2*1000);
 buffer[0]=int_theta1/256;
 buffer[1]=int_theta1%256;
 buffer[2]=int_theta2/256;
 buffer[3]=int_theta2%256;
 buffer[4]=int_v1/256;
 buffer[5]=int_v1%256; 
 buffer[6]=int_v2/256;
 buffer[7]=int_v2%256;
 id=0x320;
 dlc=8;
 flag=0;
 /*pose_sta.pose.position.x=msg->pose.pose.position.x;
 pose_sta.pose.position.y=msg->pose.pose.position.y;
 pose_sta.pose.position.z=msg->pose.pose.position.z;
 pose_sta.pose.orientation.x=msg->pose.pose.orientation.x;
 pose_sta.pose.orientation.y=msg->pose.pose.orientation.y;
 pose_sta.pose.orientation.z=msg->pose.pose.orientation.z;
 pose_sta.pose.orientation.w=msg->pose.pose.orientation.w;*/
 pose_sta.header.stamp=current_time;
 pose_sta.header.frame_id="map";
 msg_path.poses.push_back(pose_sta);
 msg_path.header.frame_id="map";
 msg_path.header.stamp=current_time;
 ROS_INFO("d:%f,theta:%f,target_x:%f,target_y:%f,target_angle:%f,X:%f,Y:%f,theta1:%f,theta2:%f,v1:%f,v2:%f,num:%d",d,theta,target_x,target_y,target_angle,x,y,theta1,theta2,v1,v2,num_);
 //ROS_INFO("theta1:%f,theta2:%f,v1:%f,v2:%f",theta1,theta2,v1,v2);
 pub_.publish(msg_twist);
 //pub_path.publish(msg_path);

 canWrite(h,id,&buffer,dlc,flag);
 canWriteSync(h,500);
 //ROS_INFO("send");
 //rate.sleep();
 canBusOff(h);   
  canClose(h);
 loop_rate.sleep();
  }
  
  return 0;

}