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
  int p_size;
   double re_x[2000];
   double re_y[2000];
   double re_d[2000];
   int re_count;
   double re_w[2000];
  

void show_x(int re_count)
{
    for(int i=0;i<re_count;i++)
    {
        printf("%f\n",re_x[i]);
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
    for(int dec_i=1;dec_i<257;dec_i++)
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

    if(find_ind>0 && find_ind<257)
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
  tar_x[0]=-1.061029;
 tar_y[0]=0.317620;
 tar_x[1]=-1.061029;
 tar_y[1]=0.317620;
 tar_x[2]=-0.794035;
 tar_y[2]=0.310700;
 tar_x[3]=-0.490538;
 tar_y[3]=0.296988;
 tar_x[4]=-0.196903;
 tar_y[4]=0.287905;
 tar_x[5]=0.114804;
 tar_y[5]=0.274085;
 tar_x[6]=0.406471;
 tar_y[6]=0.270075;
 tar_x[7]=0.727845;
 tar_y[7]=0.261036;
 tar_x[8]=1.015271;
 tar_y[8]=0.254851;
 tar_x[9]=1.307214;
 tar_y[9]=0.240351;
 tar_x[10]=1.618827;
 tar_y[10]=0.237704;
 tar_x[11]=1.915371;
 tar_y[11]=0.229897;
 tar_x[12]=2.212254;
 tar_y[12]=0.218212;
 tar_x[13]=2.507549;
 tar_y[13]=0.209292;
 tar_x[14]=2.800754;
 tar_y[14]=0.197025;
 tar_x[15]=3.105732;
 tar_y[15]=0.184117;
 tar_x[16]=3.401068;
 tar_y[16]=0.173276;
 tar_x[17]=3.698710;
 tar_y[17]=0.170825;
 tar_x[18]=3.970841;
 tar_y[18]=0.170728;
 tar_x[19]=4.278123;
 tar_y[19]=0.155088;
 tar_x[20]=4.576994;
 tar_y[20]=0.144044;
 tar_x[21]=4.852536;
 tar_y[21]=0.137828;
 tar_x[22]=5.154432;
 tar_y[22]=0.134858;
 tar_x[23]=5.467905;
 tar_y[23]=0.113391;
 tar_x[24]=5.777325;
 tar_y[24]=0.094452;
 tar_x[25]=6.074193;
 tar_y[25]=0.095567;
 tar_x[26]=6.347364;
 tar_y[26]=0.084916;
 tar_x[27]=6.615652;
 tar_y[27]=0.076074;
 tar_x[28]=6.937102;
 tar_y[28]=0.071551;
 tar_x[29]=7.250721;
 tar_y[29]=0.063840;
 tar_x[30]=7.540776;
 tar_y[30]=0.054757;
 tar_x[31]=7.863197;
 tar_y[31]=0.051367;
 tar_x[32]=8.161951;
 tar_y[32]=0.038016;
 tar_x[33]=8.465751;
 tar_y[33]=0.030626;
 tar_x[34]=8.768705;
 tar_y[34]=0.022995;
 tar_x[35]=9.069237;
 tar_y[35]=0.020898;
 tar_x[36]=9.350450;
 tar_y[36]=0.018809;
 tar_x[37]=9.657750;
 tar_y[37]=0.007666;
 tar_x[38]=9.959717;
 tar_y[38]=0.011768;
 tar_x[39]=10.254385;
 tar_y[39]=0.002910;
 tar_x[40]=10.543323;
 tar_y[40]=-0.005377;
 tar_x[41]=10.838631;
 tar_y[41]=-0.011643;
 tar_x[42]=11.144452;
 tar_y[42]=-0.022582;
 tar_x[43]=11.448743;
 tar_y[43]=-0.032804;
 tar_x[44]=11.747583;
 tar_y[44]=-0.034005;
 tar_x[45]=12.042491;
 tar_y[45]=-0.042436;
 tar_x[46]=12.338401;
 tar_y[46]=-0.048365;
 tar_x[47]=12.638403;
 tar_y[47]=-0.046424;
 tar_x[48]=12.911328;
 tar_y[48]=-0.044658;
 tar_x[49]=13.239677;
 tar_y[49]=-0.054809;
 tar_x[50]=13.556530;
 tar_y[50]=-0.061496;
 tar_x[51]=13.856381;
 tar_y[51]=-0.073687;
 tar_x[52]=14.161575;
 tar_y[52]=-0.078571;
 tar_x[53]=14.460567;
 tar_y[53]=-0.107690;
 tar_x[54]=14.744143;
 tar_y[54]=-0.113740;
 tar_x[55]=15.079856;
 tar_y[55]=-0.128101;
 tar_x[56]=15.388589;
 tar_y[56]=-0.133424;
 tar_x[57]=15.668643;
 tar_y[57]=-0.140023;
 tar_x[58]=15.971031;
 tar_y[58]=-0.144600;
 tar_x[59]=16.272260;
 tar_y[59]=-0.147623;
 tar_x[60]=16.569228;
 tar_y[60]=-0.149471;
 tar_x[61]=16.871804;
 tar_y[61]=-0.146504;
 tar_x[62]=17.179922;
 tar_y[62]=-0.147119;
 tar_x[63]=17.488643;
 tar_y[63]=-0.156164;
 tar_x[64]=17.772104;
 tar_y[64]=-0.156451;
 tar_x[65]=18.064261;
 tar_y[65]=-0.162939;
 tar_x[66]=18.343111;
 tar_y[66]=-0.164988;
 tar_x[67]=18.660182;
 tar_y[67]=-0.176366;
  tar_theta[0]=0.000000;
 tar_theta[1]=-1.484670;
 tar_theta[2]=-2.586865;
 tar_theta[3]=-1.771763;
 tar_theta[4]=-2.538632;
 tar_theta[5]=-0.787685;
 tar_theta[6]=-1.611083;
 tar_theta[7]=-1.232734;
 tar_theta[8]=-2.843386;
 tar_theta[9]=-0.486688;
 tar_theta[10]=-1.508056;
 tar_theta[11]=-2.253938;
 tar_theta[12]=-1.730212;
 tar_theta[13]=-2.395722;
 tar_theta[14]=-2.423561;
 tar_theta[15]=-2.102232;
 tar_theta[16]=-0.471804;
 tar_theta[17]=-0.020423;
 tar_theta[18]=-2.913719;
 tar_theta[19]=-2.116254;
 tar_theta[20]=-1.292326;
 tar_theta[21]=-0.563648;
 tar_theta[22]=-3.917566;
 tar_theta[23]=-3.502594;
 tar_theta[24]=0.215195;
 tar_theta[25]=-2.232845;
 tar_theta[26]=-1.887621;
 tar_theta[27]=-0.806134;
 tar_theta[28]=-1.408457;
 tar_theta[29]=-1.793617;
 tar_theta[30]=-0.602397;
 tar_theta[31]=-2.558785;
 tar_theta[32]=-1.393457;
 tar_theta[33]=-1.442898;
 tar_theta[34]=-0.399782;
 tar_theta[35]=-0.425616;
 tar_theta[36]=-2.076691;
 tar_theta[37]=0.778273;
 tar_theta[38]=-1.721847;
 tar_theta[39]=-1.642844;
 tar_theta[40]=-1.215550;
 tar_theta[41]=-2.048556;
 tar_theta[42]=-1.924005;
 tar_theta[43]=-0.230263;
 tar_theta[44]=-1.637559;
 tar_theta[45]=-1.147853;
 tar_theta[46]=0.370696;
 tar_theta[47]=0.370735;
 tar_theta[48]=-1.770751;
 tar_theta[49]=-1.209015;
 tar_theta[50]=-2.328184;
 tar_theta[51]=-0.916822;
 tar_theta[52]=-5.562526;
 tar_theta[53]=-1.222201;
 tar_theta[54]=-2.449483;
 tar_theta[55]=-0.987764;
 tar_theta[56]=-1.349829;
 tar_theta[57]=-0.867173;
 tar_theta[58]=-0.574976;
 tar_theta[59]=-0.356541;
 tar_theta[60]=0.561813;
 tar_theta[61]=-0.114362;
 tar_theta[62]=-1.678189;
 tar_theta[63]=-0.058011;
 tar_theta[64]=-1.272172;
 tar_theta[65]=-0.421004;
 tar_theta[66]=-2.055160;
 tar_theta[67]=-2.055160;

curv[0]=0.000000;
curv[1]=0.000000;
curv[2]=0.000000;
curv[3]=0.000000;
curv[4]=0.000000;
curv[5]=0.000000;
curv[6]=0.000000;
curv[7]=0.000000;
curv[8]=0.000000;
curv[9]=0.000000;
curv[10]=0.000000;
curv[11]=0.000000;
curv[12]=0.000000;
curv[13]=0.000000;
curv[14]=0.000000;
curv[15]=0.000000;
curv[16]=0.000000;
curv[17]=0.000000;
curv[18]=0.000000;
curv[19]=0.000000;
curv[20]=0.000000;
curv[21]=0.000000;
curv[22]=0.000000;
curv[23]=0.000000;
curv[24]=0.000000;
curv[25]=0.000000;
curv[26]=0.000000;
curv[27]=0.000000;
curv[28]=0.000000;
curv[29]=0.000000;
curv[30]=0.000000;
curv[31]=0.000000;
curv[32]=0.000000;
curv[33]=0.000000;
curv[34]=0.000000;
curv[35]=0.000000;
curv[36]=0.000000;
curv[37]=0.000000;
curv[38]=0.000000;
curv[39]=0.000000;
curv[40]=0.000000;
curv[41]=0.000000;
curv[42]=0.000000;
curv[43]=0.000000;
curv[44]=0.000000;
curv[45]=0.000000;
curv[46]=0.000000;
curv[47]=0.000000;
curv[48]=0.000000;
curv[49]=0.000000;
curv[50]=0.000000;
curv[51]=0.000000;
curv[52]=0.000000;
curv[53]=0.000000;
curv[54]=0.000000;
curv[55]=0.000000;
curv[56]=0.000000;
curv[57]=0.000000;
curv[58]=0.000000;
curv[59]=0.000000;
curv[60]=0.000000;
curv[61]=0.000000;
curv[62]=0.000000;
curv[63]=0.000000;
curv[64]=0.000000;
curv[65]=0.000000;
curv[66]=0.000000;
curv[67]=0.000000;

//ros节点初始化
  ros::init(argc, argv, "track_carto_pp");
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
  p_size=67;
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

  v_pp=0.6;
  d_ym=0.8;

  target_x=tar_x[num_];
  target_y=tar_y[num_];
  d_tar=sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y));
    while (d_tar<d_ym && ros::ok())
  {
     num_=num_+1;
     if(num_>p_size)
     {
        num_=p_size;
        break;
     }
      target_x=tar_x[num_];
      target_y=tar_y[num_];
     d_tar=sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y));
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
  if(num_==p_size && d<0.8)
  {
    msg_twist.linear.x=0;
    msg_twist.linear.y=0;
    msg_twist.linear.z=0;
    msg_twist.angular.x=0;
    msg_twist.angular.y=0;
    msg_twist.angular.z=0;
    ROS_INFO("arrive");     
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
 {  re_w[re_count]=theta1;
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
  return 0;

}