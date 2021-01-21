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
//#include "LinearMath/btMatrix3x3.h"
double d;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_path= n_.advertise<nav_msgs::Path>("/path", 1,true);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/odom", 1, &SubscribeAndPublish::callback, this);
    sub_tra=n_.subscribe("/trajectory_node_list",1,&SubscribeAndPublish::callback_tra,this);
    num_=0;
    num=0;
  /*num=0;
  num_=0;
  for(jiaodu=30;jiaodu<=120;jiaodu=jiaodu+10)
  {
    tar_x[num]=50*sin(jiaodu/57.29578);
    tar_y[num]=50*cos(jiaodu/57.29578);
    num++;
  }*/
  target_x=0;
  target_y=0;
  //ROS_INFO("num:%d",num);
  }
  void callback_tra(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    for(count=0,count_tar=0;count<1455;count=count+5,count_tar++)
    {
     tar_x[count_tar]=msg->markers[1].points[count].x;
     tar_y[count_tar]=msg->markers[1].points[count].y;
     //ROS_INFO("receive tra");
     //ROS_INFO("x:%f,y:%f,count_tar:%d",tar_x[count_tar],tar_y[count_tar],count_tar);
    }
    //ROS_INFO("count_tar:%d",count_tar);
  }
 
  void callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
  current_time=ros::Time::now();
  double x,y,z;
  tf::Quaternion quat;
  geometry_msgs::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  x=msg->pose.pose.position.x;
  y=msg->pose.pose.position.y;
  z=msg->pose.pose.position.z;
  //ROS_INFO("x:%f,y:%f,yaw:%f", x,y,yaw);
  double theta,target_angle;
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

      num_++;
      if(num_>148)
      {
        num_=148;
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
if(theta<10 && theta>0)
 {
    msg_twist.angular.z=theta/50;
 }
 if(theta==10 || theta>10)
 {
   msg_twist.angular.z=0.2;
 }
 if(theta<0 && theta>-10)
 {
   msg_twist.angular.z=theta/50;
 }
 if(theta<-10 || theta==-10)
 {
    msg_twist.angular.z=-0.2;
 }
  if(theta>180 || theta==180)
 {
    msg_twist.angular.z=-0.2;
 }
  if(theta<-180 || theta==-180)
 {
    msg_twist.angular.z=0.2;
 }
 r=fabs(msg_twist.linear.x/msg_twist.angular.z);
 theta1=atan(1.42/(2*r-1.16))*57.29578;
 theta3=atan(1.42/(2*r+1.16))*57.29578;
 v1=msg_twist.angular.z*sqrt(pow(r-0.58,2)+pow(0.71,2));
 v3=msg_twist.angular.z*sqrt(pow(r+0.58,2)+pow(0.71,2));
 pose_sta.pose.position.x=msg->pose.pose.position.x;
 pose_sta.pose.position.y=msg->pose.pose.position.y;
 pose_sta.pose.position.z=msg->pose.pose.position.z;
 pose_sta.pose.orientation.x=msg->pose.pose.orientation.x;
 pose_sta.pose.orientation.y=msg->pose.pose.orientation.y;
 pose_sta.pose.orientation.z=msg->pose.pose.orientation.z;
 pose_sta.pose.orientation.w=msg->pose.pose.orientation.w;
 pose_sta.header.stamp=current_time;
 pose_sta.header.frame_id="map";
 msg_path.poses.push_back(pose_sta);
 msg_path.header.frame_id="map";
 msg_path.header.stamp=current_time;
 //ROS_INFO("d:%f,theta:%f,target_x:%f,target_y:%f,target_angle:%f,r:%f,num:%d,theta1:%f,theta3:%f,v1:%f,v3:%f",d,theta,target_x,target_y,target_angle,r,num_,theta1,theta3,v1,v3);
 ROS_INFO("theta1:%f,theta3:%f,v1:%f,v3:%f",theta1,theta3,v1,v3);
 pub_.publish(msg_twist);
 pub_path.publish(msg_path);
     /*msg_twist.linear.x=1;
     msg_twist.angular.z=pow(-1,count)*0.3;
     count++;
     pub_.publish(msg_twist);
     ROS_INFO("send msg");*/

}
 
private:
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
  double theta1,theta2,theta3,theta4;
  double v1,v2,v3,v4;

};//End of class SubscribeAndPublish
 
int main(int argc, char **argv)
{
  //static double d;
  //Initiate ROS
  ros::init(argc, argv, "cmd_vel");
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject=SubscribeAndPublish();
  ROS_INFO("callback\n");
  ros::spin();
  return 0;
}