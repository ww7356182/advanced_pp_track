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
    //sub_tra=n_.subscribe("/trajectory_node_list",1,&SubscribeAndPublish::callback_tra,this);
    num_=0;
    num=0;
    canInitializeLibrary();
    h=canOpenChannel(0,canWANT_EXCLUSIVE);
    tar_x[0]=0;
    tar_y[0]=0;
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
    tar_x[7]=14.4249;
    tar_y[7]=-8.3153;
    tar_x[8]=15.3134;
    tar_y[8]=-9.062;
    tar_x[9]=16.3062;
    tar_y[9]=-9.3782;
    tar_x[10]=17.3609;
    tar_y[10]=-9.3171;
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
    tar_x[16]=17.5365;
    tar_y[16]=-0.7367;
    tar_x[17]=16.5373;
    tar_y[17]=-0.2976;
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


   /* tar_x[18]=10.9627;
    tar_y[18]=-7.1277;
    tar_x[19]=12.7397;
    tar_y[19]=-6.5221;
    tar_x[20]=13.4057;
    tar_y[20]=-6.0152;
    tar_x[21]=13.9134;
    tar_y[21]=-5.4331;
    tar_x[22]=14.2952;
    tar_y[22]=-4.7837;
    tar_x[23]=14.5364;
    tar_y[23]=-4.1090;
    tar_x[24]=14.5701;
    tar_y[24]=-3.5538;
    tar_x[25]=14.4895;
    tar_y[25]=-3.0244;
    tar_x[26]=14.3179;
    tar_y[26]=-2.5115;
    tar_x[27]=14.0332;
    tar_y[27]=-1.9555;
    tar_x[28]=13.6436;
    tar_y[28]=-1.4549;
    tar_x[29]=13.1595;
    tar_y[29]=-1.0525;
    tar_x[30]=12.5614;
    tar_y[30]=-0.6909;
    tar_x[31]=11.6203;
    tar_y[31]=-0.2274;
    tar_x[32]=10.5806;
    tar_y[32]=0.0758;
    tar_x[33]=9.4757;
    tar_y[33]=0.1753;
    tar_x[34]=0;
    tar_y[34]=0;
    */
    if(h != canOK)
    {
        char msg[64];
        canGetErrorText((canStatus)h,msg,sizeof(msg));
        fprintf(stderr,"canopenchannel fialied(%s)\n",msg);
        exit(1);

    }
    canSetBusParams(h,BAUD_500K,0,0,0,0,0);
    canBusOn(h);
    msg_1=0x00;
    id=0x320;
    dlc=8;
    
    flag=0;
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
 /*void callback_tra(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    ROS_INFO("start");
    for(count=0,count_tar=0;count<336;count=count+20,count_tar++)
    {
     tar_x[count_tar]=msg->markers[1].points[count].x;
     tar_y[count_tar]=msg->markers[1].points[count].y;
     ROS_INFO("x:%f,Y:%f\n",tar_x[count_tar],tar_y[count_tar]);
     //ROS_INFO("receive tra");
     //ROS_INFO("x:%f,y:%f,count_tar:%d",tar_x[count_tar],tar_y[count_tar],count_tar);
    }
        for(count_2=0;count_2<66;count_2=count_2+10,count_tar++)
    {
     tar_x[count_tar]=msg->markers[2].points[count].x;
     tar_y[count_tar]=msg->markers[2].points[count].y;
    }

    ROS_INFO("count_tar:%d",count_tar);
  }*/
 
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
    msg_twist.angular.z=(theta+20)/200;
 }
 if(theta<20&&theta>0)
 {
   msg_twist.angular.z=theta/100;
 }
 if(theta==40 || theta>40)
 {
   msg_twist.angular.z=0.3;
 }
 if(theta<0 && theta>-20)
 {
   msg_twist.angular.z=theta/100;
 }
 if(theta<-20 && theta>-40)
 {
    msg_twist.angular.z=-(fabs(theta)+20)/200;
 }
 if(theta==20)
 {
   msg_twist.angular.z=0.2;
 }
 if(theta==-20)
 {
   msg_twist.angular.z=-0.2;
 }
 if(theta<-40 || theta==-40)
 {
    msg_twist.angular.z=-0.3;
 }
  if(theta>180 || theta==180)
 {
    msg_twist.angular.z=-0.15;
 }
  if(theta<-180 || theta==-180)
 {
    msg_twist.angular.z=0.15;
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
 ROS_INFO("d:%f,theta:%f,target_x:%f,target_y:%f,target_angle:%f,r:%f,num:%d,theta1:%f,theta2:%f,v1:%f,v2:%f",d,theta,target_x,target_y,target_angle,r,num_,theta1,theta2,v1,v2);
 //ROS_INFO("theta1:%f,theta3:%f,v1:%f,v3:%f",theta1,theta3,v1,v3);
 pub_.publish(msg_twist);
 pub_path.publish(msg_path);
 canWrite(h,id,&buffer,dlc,flag);
 canWriteSync(h,500);
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
  double theta1,theta2,theta3,theta4,theta_temp;
  double v1,v2,v3,v4;
  canHandle h;
  unsigned int flag;
  long id;
  long id_1;
  unsigned char buffer[8];
  long int msg_1,msg_2,msg_3;
  int int_theta1,int_theta2,int_v1,int_v2;
  unsigned int dlc;
  int count_2;

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
  //canBusOff(h);   
  //canClose(h);
  return 0;
}