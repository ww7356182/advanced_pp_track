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
#include <iostream>
#include <endian.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <Eigen/Dense>
//声明变量

using namespace std;
using namespace Eigen;
  double tar_x[500];
  double tar_y[500];
  double tar_theta[500];
  double curv[500];
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
  double fun_a,fun_b;
  double d_curv;
  Eigen::Vector3d v_result;
  int ins_count;
  int p_size;
   double re_x[2000];
   double re_y[2000];
   double re_d[2000];
   int re_count;
   double re_w[2000];
   double yaw_rate[2000];
  

void show_x(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",re_x[i]);
    }
}

void show_yaw(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",yaw_rate[i]);
    }
}

void show_y(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",re_y[i]);
    }
}

void show_d(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",re_d[i]);
    }
}
void show_w(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",re_w[i]);
    }
}
void dec_min(const double& x_,const double& y_,double* tar_x,double* tar_y)
{
    double dec_d_min,dec_d_temp;
    double d_fro,d_rear;
    int dec_min_ind;
    int find_ind;
    dec_min_ind=0;
    dec_d_min=sqrt((x_-tar_x[0])*(x_-tar_x[0])+(y_-tar_y[0])*(y_-tar_y[0]));
    for(int dec_i=1;dec_i<p_size+1;dec_i++)
    {
      dec_d_temp=sqrt((x_-tar_x[dec_i])*(x_-tar_x[dec_i])+(y_-tar_y[dec_i])*(y_-tar_y[dec_i]));
      if(dec_d_temp<dec_d_min)
      {
          dec_d_min=dec_d_temp;
          dec_min_ind=dec_i;
      }
    }
    d_min=dec_d_min;
    find_ind=dec_min_ind;

    if(find_ind>0 && find_ind<p_size)
  {
  d_fro=sqrt((tar_x[find_ind+1]-x_init)*(tar_x[find_ind+1]-x_init)+(tar_y[find_ind+1]-y_init)*(tar_y[find_ind+1]-y_init));
  d_rear=sqrt((tar_x[find_ind-1]-x_init)*(tar_x[find_ind-1]-x_init)+(tar_y[find_ind-1]-y_init)*(tar_y[find_ind-1]-y_init));
  min_ind=find_ind;
  if(d_fro<d_rear || d_fro==d_rear)
 {
   sec_min_ind=find_ind+1;
 }
 else
 {
   sec_min_ind=find_ind-1;
 }
}
if(find_ind==0)
{
  min_ind=0;
  sec_min_ind=1;
}
    return;
}

/*int find_min_ind(const double& x_init,const double& y_init,const double& yaw)
{
  int ind_temp[300];
  int find_j,find_ind;
  double find_d,find_d_temp;
  find_j=0;
  for(int find_i=0;find_i<258;find_i++)
  {
    double find_theta_temp;
    find_theta_temp=yaw*57.29578-tar_theta[find_i];
    if(find_theta_temp>180)
    {
      find_theta_temp=find_theta_temp-360;
    }
    if(find_theta_temp<-180)
    {
      find_theta_temp=find_theta_temp+360;
    }
    if(fabs(find_theta_temp)<90)
    {
        ind_temp[find_j]=find_i;
        find_j++;
    }
  }
  find_d=sqrt((tar_x[ind_temp[0]]-x_init)*(tar_x[ind_temp[0]]-x_init)+(tar_y[ind_temp[0]]-y_init)*(tar_y[ind_temp[0]]-y_init));
  find_ind=ind_temp[0];
  for(int find_k=1;find_k<find_j;find_k++)
  {
     find_d_temp=sqrt((tar_x[ind_temp[find_k]]-x_init)*(tar_x[ind_temp[find_k]]-x_init)+(tar_y[ind_temp[find_k]]-y_init)*(tar_y[ind_temp[find_k]]-y_init));
     if(find_d>find_d_temp)
     {
       find_d=find_d_temp;
       find_ind=ind_temp[find_k];
     }
  }
  d_min=find_d;
  return find_ind;
}*/

void find_min_ind(const double& x_init,const double& y_init,const double& yaw)
{
  int ind_temp[300];
  int find_j,find_ind;
  double d_fro,d_rear;
  double find_d,find_d_temp;
  find_j=0;
  for(int find_i=0;find_i<p_size+1;find_i++)
  {
    double find_theta_temp;
    find_theta_temp=yaw*57.29578-tar_theta[find_i];
    if(find_theta_temp>180)
    {
      find_theta_temp=find_theta_temp-360;
    }
    if(find_theta_temp<-180)
    {
      find_theta_temp=find_theta_temp+360;
    }
    if(fabs(find_theta_temp)<150)
    {
        ind_temp[find_j]=find_i;
        find_j++;
    }
  }
  find_d=sqrt((tar_x[ind_temp[0]]-x_init)*(tar_x[ind_temp[0]]-x_init)+(tar_y[ind_temp[0]]-y_init)*(tar_y[ind_temp[0]]-y_init));
  find_ind=ind_temp[0];
  for(int find_k=1;find_k<find_j;find_k++)
  {
     find_d_temp=sqrt((tar_x[ind_temp[find_k]]-x_init)*(tar_x[ind_temp[find_k]]-x_init)+(tar_y[ind_temp[find_k]]-y_init)*(tar_y[ind_temp[find_k]]-y_init));
     if(find_d>find_d_temp)
     {
       find_d=find_d_temp;
       find_ind=ind_temp[find_k];
     }
  }
  d_min=find_d;
  min_ind=find_ind;
  if(find_ind>0 && find_ind<p_size)
  {
  d_fro=sqrt((tar_x[find_ind+1]-x_init)*(tar_x[find_ind+1]-x_init)+(tar_y[find_ind+1]-y_init)*(tar_y[find_ind+1]-y_init));
  d_rear=sqrt((tar_x[find_ind-1]-x_init)*(tar_x[find_ind-1]-x_init)+(tar_y[find_ind-1]-y_init)*(tar_y[find_ind-1]-y_init));
  min_ind=find_ind;
  if(d_fro<d_rear || d_fro==d_rear)
 {
   sec_min_ind=find_ind+1;
 }
 else
 {
   sec_min_ind=find_ind-1;
 }
}
if(find_ind==0)
{
  min_ind=0;
  sec_min_ind=1;
}
  return ;
}

int main(int argc, char **argv)
{
//赋值
  tar_x[0]=-0.403013;
 tar_y[0]=0.662679;
 tar_x[1]=-0.257130;
 tar_y[1]=0.663731;
 tar_x[2]=-0.099851;
 tar_y[2]=0.665784;
 tar_x[3]=0.057797;
 tar_y[3]=0.670408;
 tar_x[4]=0.231189;
 tar_y[4]=0.669850;
 tar_x[5]=0.408150;
 tar_y[5]=0.680405;
 tar_x[6]=0.592929;
 tar_y[6]=0.686742;
 tar_x[7]=0.773572;
 tar_y[7]=0.687298;
 tar_x[8]=0.962881;
 tar_y[8]=0.688841;
 tar_x[9]=1.133022;
 tar_y[9]=0.705572;
 tar_x[10]=1.305391;
 tar_y[10]=0.708161;
 tar_x[11]=1.474511;
 tar_y[11]=0.716468;
 tar_x[12]=1.667221;
 tar_y[12]=0.722875;
 tar_x[13]=1.865691;
 tar_y[13]=0.732260;
 tar_x[14]=2.035944;
 tar_y[14]=0.730902;
 tar_x[15]=2.214018;
 tar_y[15]=0.739525;
 tar_x[16]=2.389056;
 tar_y[16]=0.743178;
 tar_x[17]=2.557046;
 tar_y[17]=0.743096;
 tar_x[18]=2.738898;
 tar_y[18]=0.748071;
 tar_x[19]=2.915167;
 tar_y[19]=0.755507;
 tar_x[20]=3.101898;
 tar_y[20]=0.761413;
 tar_x[21]=3.293979;
 tar_y[21]=0.773565;
 tar_x[22]=3.470162;
 tar_y[22]=0.781138;
 tar_x[23]=3.645944;
 tar_y[23]=0.784947;
 tar_x[24]=3.830467;
 tar_y[24]=0.796187;
 tar_x[25]=4.004238;
 tar_y[25]=0.795468;
 tar_x[26]=4.195579;
 tar_y[26]=0.805220;
 tar_x[27]=4.377448;
 tar_y[27]=0.804372;
 tar_x[28]=4.540061;
 tar_y[28]=0.816331;
 tar_x[29]=4.714608;
 tar_y[29]=0.819272;
 tar_x[30]=4.891709;
 tar_y[30]=0.823658;
 tar_x[31]=5.059236;
 tar_y[31]=0.825030;
 tar_x[32]=5.245308;
 tar_y[32]=0.829396;
 tar_x[33]=5.427906;
 tar_y[33]=0.835259;
 tar_x[34]=5.591568;
 tar_y[34]=0.840535;
 tar_x[35]=5.774208;
 tar_y[35]=0.844253;
 tar_x[36]=5.952010;
 tar_y[36]=0.847562;
 tar_x[37]=6.132385;
 tar_y[37]=0.851293;
 tar_x[38]=6.291048;
 tar_y[38]=0.860023;
 tar_x[39]=6.469086;
 tar_y[39]=0.871845;
 tar_x[40]=6.631442;
 tar_y[40]=0.871838;
 tar_x[41]=6.811171;
 tar_y[41]=0.874715;
 tar_x[42]=6.992294;
 tar_y[42]=0.880542;
 tar_x[43]=7.172993;
 tar_y[43]=0.901253;
 tar_x[44]=7.358367;
 tar_y[44]=0.904208;
 tar_x[45]=7.536639;
 tar_y[45]=0.900958;
 tar_x[46]=7.714903;
 tar_y[46]=0.917663;
 tar_x[47]=7.886115;
 tar_y[47]=0.931984;
 tar_x[48]=8.077129;
 tar_y[48]=0.937027;
 tar_x[49]=8.230913;
 tar_y[49]=0.948207;
 tar_x[50]=8.416774;
 tar_y[50]=0.951790;
 tar_x[51]=8.594564;
 tar_y[51]=0.966786;
 tar_x[52]=8.776319;
 tar_y[52]=0.970303;
 tar_x[53]=8.948560;
 tar_y[53]=0.983076;
 tar_x[54]=9.140111;
 tar_y[54]=0.975178;
 tar_x[55]=9.318749;
 tar_y[55]=0.984575;
 tar_x[56]=9.503468;
 tar_y[56]=1.000970;
 tar_x[57]=9.678533;
 tar_y[57]=1.008410;
 tar_x[58]=9.864983;
 tar_y[58]=1.012924;
 tar_x[59]=10.027548;
 tar_y[59]=1.030041;
 tar_x[60]=10.216831;
 tar_y[60]=1.032773;
 tar_x[61]=10.398067;
 tar_y[61]=1.038531;
 tar_x[62]=10.574274;
 tar_y[62]=1.046036;
 tar_x[63]=10.741410;
 tar_y[63]=1.057780;
 tar_x[64]=10.907556;
 tar_y[64]=1.059542;
 tar_x[65]=11.084243;
 tar_y[65]=1.069033;
 tar_x[66]=11.260258;
 tar_y[66]=1.071800;
 tar_x[67]=11.437934;
 tar_y[67]=1.074118;
 tar_x[68]=11.612846;
 tar_y[68]=1.082410;
 tar_x[69]=11.788068;
 tar_y[69]=1.093434;
 tar_x[70]=11.963247;
 tar_y[70]=1.103751;
 tar_x[71]=12.145856;
 tar_y[71]=1.102093;
 tar_x[72]=12.320907;
 tar_y[72]=1.109347;
 tar_x[73]=12.501088;
 tar_y[73]=1.119329;
 tar_x[74]=12.681838;
 tar_y[74]=1.128503;
 tar_x[75]=12.859385;
 tar_y[75]=1.138311;
 tar_x[76]=13.046118;
 tar_y[76]=1.144225;
 tar_x[77]=13.231708;
 tar_y[77]=1.149557;
 tar_x[78]=13.399007;
 tar_y[78]=1.163199;
 tar_x[79]=13.597321;
 tar_y[79]=1.163837;
 tar_x[80]=13.778486;
 tar_y[80]=1.165680;
 tar_x[81]=13.957455;
 tar_y[81]=1.169461;
 tar_x[82]=14.133793;
 tar_y[82]=1.178326;
 tar_x[83]=14.294108;
 tar_y[83]=1.188397;
 tar_x[84]=14.491358;
 tar_y[84]=1.196738;
 tar_x[85]=14.680949;
 tar_y[85]=1.205281;
 tar_x[86]=14.860152;
 tar_y[86]=1.216919;
 tar_x[87]=15.043762;
 tar_y[87]=1.221546;
 tar_x[88]=15.230777;
 tar_y[88]=1.220380;
 tar_x[89]=15.381966;
 tar_y[89]=1.231882;
 tar_x[90]=15.582178;
 tar_y[90]=1.239317;
 tar_x[91]=15.756495;
 tar_y[91]=1.248830;
 tar_x[92]=15.937383;
 tar_y[92]=1.255247;
 tar_x[93]=16.111457;
 tar_y[93]=1.269332;
 tar_x[94]=16.292299;
 tar_y[94]=1.271278;
 tar_x[95]=16.423841;
 tar_y[95]=1.278529;

for(int x_count=0;x_count<96;x_count++)
{
  tar_theta[x_count]=0;
  curv[x_count]=0;
}

  



//ros节点初始化
  ros::init(argc, argv, "track_carto_curv");
  ros::NodeHandle n_; 
  ros::Publisher pub_path;  
  ros::Publisher agv_pub;
  ros::Publisher pub_path_target;
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
  num_=0;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  count_init=0;
  error_ins=0;
  p_size=95;
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
  find_min_ind(x,y,yaw);
  //dec_min(x,y,tar_x,tar_y);
  num_=min_ind;

  //dec_min(x,y,tar_x,tar_y);
  //num_=min_ind;

  fun_a=(tar_y[sec_min_ind]-tar_y[min_ind])/(tar_x[sec_min_ind]-tar_x[min_ind]);
  fun_b=tar_y[min_ind]-fun_a*tar_x[min_ind];
  error_ins=fabs(fun_a*x-y+fun_b)/sqrt(fun_a*fun_a+1);
  
 
  sum_er=sum_er+error_ins;
  error_count++;
  error=sum_er/error_count;

  re_x[re_count]=x;
  re_y[re_count]=y;
  re_d[re_count]=error_ins;
  
  
  //设定线速度
  if(curv[min_ind]<0.1 || curv[min_ind]==0.1)
  {
    d_curv=1;
    v_pp=1;
  }
  if(curv[min_ind]>0.1 && curv[min_ind]<0.3)
  {
    d_curv=0.8;
    v_pp=0.8;
  }

  if(curv[min_ind]>0.3 || curv[min_ind]==0.3)
  {
    d_curv=0.8;
    v_pp=0.6;
  }
  v_pp=0.6;
  d_ym=1.5;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  d_tar=sqrt((target_x-tar_x[min_ind])*(target_x-tar_x[min_ind])+(target_y-tar_y[min_ind])*(target_y-tar_y[min_ind]));
    while ((d<d_ym || d_tar<0.6*d_min)&& ros::ok())
    //while ((d<d_ym)&& ros::ok())
  {
     num_=num_+1;
     if(num_>p_size)
     {
        num_=p_size;
        break;
     }
      target_x=tar_x[num_];
      target_y=tar_y[num_];
      //d_tar=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
     d_tar=sqrt((target_x-tar_x[min_ind])*(target_x-tar_x[min_ind])+(target_y-tar_y[min_ind])*(target_y-tar_y[min_ind]));
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
  re_w[re_count]=msg_twist.angular.z;

//轨迹终点停车
  if(num_==p_size && d<0.8)
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
   re_w[re_count]=theta2;
   re_count++;
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
   re_w[re_count]=theta2;
   re_count++;
    can_angle=int_theta1;
    can_angle=htobe16(can_angle);
    *(int16_t*)&buffer[2]=can_angle;
    can_v=int_v2;
    //can_v=htobe16(can_v);
    //ROS_INFO("%d", can_v);
    //*(int16_t*)&buffer[0]=can_v;
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


 //id=320接口
 /*id=0x320;
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
 buffer[7]=int_v2%256;*/

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

//ROS_INFO("d:%f\n num:%d\n,target_x:%f\n target_y:%f\n",d,num_,target_x,target_y);
 //printf("num:%d,err_ins:%f,error:%f,v:%f,d_ym:%f\n",num_,error_ins,error,msg_twist.linear.x,d_ym);


//printf("%f ",error_ins);
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
  show_x(re_count);
  printf("end\n");
  show_y(re_count);
  printf("end\n");
  show_d(re_count);
  printf("end\n");
  show_w(re_count);
  printf("end\n");
  show_yaw(re_count);
  return 0;

}