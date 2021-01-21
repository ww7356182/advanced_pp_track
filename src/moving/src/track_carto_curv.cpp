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
#include<iostream>
#include<string>


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
   int index_five;
   int index_three;
   double d_five;
   double d_three;
  

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

double points_dis(double x1,double y1,double x2,double y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
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

class Fuzzy_controller
{
public:
	const static int N = 2;//定义量化论域模糊子集的个数
private:
	string mf_t_e;   //e的隶属度函数类型
	string mf_t_de;  //de的隶属度函数类型
	string mf_t_u;   
	string mf_zero;  
	string mf_five; 
	string mf_three;
	string mf_d;
	double* e_mf_paras;
	double* de_mf_paras;
	double* u_mf_paras;

	double* zero_mf_paras;
	double* three_mf_paras;
	double* five_mf_paras;
	double* d_mf_paras;
	double zero;
	double five;
	double three;
	double d;
	double zero_max;
	double five_max;
	double three_max;
	double l_max;
	double temp_zero[3], temp_five[3], temp_three[3];

public:
	Fuzzy_controller();
	~Fuzzy_controller();
	double trimf(double x, double a, double b, double c);          //三角隶属度函数
	double gaussmf(double x, double ave, double sigma);          //正态隶属度函数
	double trapmf(double x, double a, double b, double c, double d); //梯形隶属度函数
	//设置模糊隶属度函数的参数
	void setMf(const string& mf_type_zero, double* zero_mf, const string& mf_type_five, double* five_mf, const string& mf_type_three, double* three_mf);
	double realize(double zero, double five, double three);
	double rule(int i, int j, int k);
};


Fuzzy_controller::Fuzzy_controller()
{
	mf_t_e = "trimf";
	mf_t_de = "trimf";
	mf_t_u = "trimf";
	mf_zero= "trapmf";   
	mf_five= "trapmf";
	mf_three= "trapmf";
	mf_d= " trimf";
}

double Fuzzy_controller::rule(int i, int j, int k)
{
	if (i == 0 && j == 0 && k == 0)
		return 1.5;
	if (i == 0 && j == 0 && k == 1)
		return 1.2;
	if (i == 0 && j == 0 && k == 2)
		return 1.2;
	if (i == 0 && j == 1)
		return 1.2;
	if (i == 0 && j == 2)
		return 0.8;
	if (i == 2)
		return 0.8;
	if (i == 1 && j == 0)
		return 1.2;
	if (i == 1 && j == 1)
		return 1.2;
	if (i == 1 && j == 2)
		return 1.2;
    }

Fuzzy_controller::~Fuzzy_controller()
{
	delete[] e_mf_paras;
	delete[] de_mf_paras;
	delete[] u_mf_paras;
	delete[] zero_mf_paras;
	delete[] five_mf_paras;
	delete[] three_mf_paras;
	delete[] d_mf_paras;
}
//三角隶属度函数
double Fuzzy_controller::trimf(double x, double a, double b, double c)
{
	double u;
	if (x >= a && x <= b)
		u = (x - a) / (b - a);
	else if (x > b && x <= c)
		u = (c - x) / (c - b);
	else
		u = 0.0;
	return u;

}
//正态隶属度函数
double Fuzzy_controller::gaussmf(double x, double ave, double sigma)
{
	double u;
	if (sigma < 0)
	{
		cout << "In gaussmf, sigma must larger than 0" << endl;
	}
	u = exp(-pow(((x - ave) / sigma), 2));
	return u;
}
//梯形隶属度函数
double Fuzzy_controller::trapmf(double x, double a, double b, double c, double d)
{
	double u;
	if (x >= a && x < b)
		u = (x - a) / (b - a);
	else if (x >= b && x < c)
		u = 1;
	else if (x >= c && x <= d)
		u = (d - x) / (d - c);
	else
		u = 0;
	return u;
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller::setMf(const string & mf_type_zero, double* zero_mf, const string & mf_type_five, double* five_mf, const string & mf_type_three, double* three_mf)
{
	if (mf_type_zero == "trimf" || mf_type_zero == "gaussmf" || mf_type_zero == "trapmf")
		mf_zero = mf_type_zero;
	else
		std::cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;

	if (mf_type_five == "trimf" || mf_type_five == "gaussmf" || mf_type_five == "trapmf")
		mf_five = mf_type_five;
	else
		std::cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;

	if (mf_type_three == "trimf" || mf_type_three == "gaussmf" || mf_type_three == "trapmf")
		mf_three = mf_type_three;
	else
		std::cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;

	zero_mf_paras = new double[3* 4];
	five_mf_paras = new double[3 * 4];
	three_mf_paras = new double[3* 4];
	for(int i = 0; i < 3 * 4; i++)
		zero_mf_paras[i] = zero_mf[i];
	for(int i = 0; i < 3 * 4; i++)
		five_mf_paras[i] = five_mf[i];
	for(int i = 0; i < 3 * 4; i++)
		three_mf_paras[i] = three_mf[i];
}
//实现模糊控制
double Fuzzy_controller::realize(double zero, double five, double three)
{
	if (zero > 0.4)
		zero = 0.4; 
	if (five > 0.4)
		five = 0.4;
	if (three > 0.4)
		three= 0.4;
	double u;
	int M;
	if (mf_zero == "trimf")
		M = 3;               //三角函数有三个参数
	else if (mf_zero == "gaussmf")
		M = 2;              //正态函数有两个参数
	else if (mf_zero == "trapmf")
		M = 4;              //梯形函数有四个参数
	for (int i = 0; i < 3; i++)
	{
		temp_zero[i] = trapmf(zero, zero_mf_paras[i * 4], zero_mf_paras[i * 4 + 1], zero_mf_paras[i * 4 + 2], zero_mf_paras[i * 4 + 3]);//e模糊化，计算它的隶
	}

	if (mf_five == "trimf")
		M = 3;               //三角函数有三个参数
	else if (mf_five == "gaussmf")
		M = 2;              //正态函数有两个参数
	else if (mf_five == "trapmf")
		M = 4;              //梯形函数有四个参数
	for (int i = 0; i < 3; i++)
	{
		temp_five[i] = trapmf(five, five_mf_paras[i * 4], five_mf_paras[i * 4 + 1], five_mf_paras[i * 4 + 2], five_mf_paras[i * 4 + 3]);//e模糊化，计算它的隶
	}

	if (mf_three == "trimf")
		M = 3;               //三角函数有三个参数
	else if (mf_three == "gaussmf")
		M = 2;              //正态函数有两个参数
	else if (mf_three == "trapmf")
		M = 4;              //梯形函数有四个参数
	for (int i = 0; i < 3; i++)
	{
		temp_three[i] = trapmf(three, three_mf_paras[i * 4], three_mf_paras[i * 4 + 1], three_mf_paras[i * 4 + 2], three_mf_paras[i * 4 + 3]);//e模糊化，计算它的隶
	}


	double den = 0, num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
			for(int k=0;k<3;k++)
		{
				num += temp_zero[m] * temp_five[n] * temp_three[k] * rule(m, n, k);
			den += temp_zero[m] * temp_five[n] * temp_three[k];
		}
	if (den == 0)
		cout << "分母为零" << endl;
	u = num / den;
	if (u <=0.6)   u = 0.6;
	if (u >= 1.2) u = 1.2;
	return u;
}


int main(int argc, char **argv)
{
  double u = 0;
	double zero_mf_paras[12] = { 0,0,0.1,0.25,0.1,0.2,0.25,0.35,0.2,0.35,0.5,0.5 };
	double five_mf_paras[12] = { 0,0,0.1,0.25,0.1,0.2,0.25,0.35,0.2,0.35,0.5,0.5 };
	double three_mf_paras[12] = { 0,0,0.1,0.25,0.1,0.2,0.25,0.35,0.2,0.35,0.5,0.5 };
	Fuzzy_controller fuzzy;
	fuzzy.setMf("trapmf", zero_mf_paras, "trapmf", five_mf_paras, "trapmf", three_mf_paras);
//赋值
  tar_x[0]=-0.611453;
 tar_y[0]=0.722686;
 tar_x[1]=-0.611453;
 tar_y[1]=0.722686;
 tar_x[2]=-0.597199;
 tar_y[2]=0.721619;
 tar_x[3]=-0.396418;
 tar_y[3]=0.719728;
 tar_x[4]=-0.132341;
 tar_y[4]=0.723510;
 tar_x[5]=0.166234;
 tar_y[5]=0.717118;
 tar_x[6]=0.482546;
 tar_y[6]=0.707553;
 tar_x[7]=0.793526;
 tar_y[7]=0.701546;
 tar_x[8]=1.098287;
 tar_y[8]=0.704099;
 tar_x[9]=1.379419;
 tar_y[9]=0.693288;
 tar_x[10]=1.700987;
 tar_y[10]=0.695169;
 tar_x[11]=2.010806;
 tar_y[11]=0.688604;
 tar_x[12]=2.316776;
 tar_y[12]=0.713575;
 tar_x[13]=2.615586;
 tar_y[13]=0.709380;
 tar_x[14]=2.915184;
 tar_y[14]=0.706296;
 tar_x[15]=3.215049;
 tar_y[15]=0.705521;
 tar_x[16]=3.508333;
 tar_y[16]=0.699257;
 tar_x[17]=3.800266;
 tar_y[17]=0.696974;
 tar_x[18]=4.096155;
 tar_y[18]=0.696862;
 tar_x[19]=4.398743;
 tar_y[19]=0.700682;
 tar_x[20]=4.685294;
 tar_y[20]=0.696850;
 tar_x[21]=4.970318;
 tar_y[21]=0.693868;
 tar_x[22]=5.264818;
 tar_y[22]=0.688142;
 tar_x[23]=5.556486;
 tar_y[23]=0.685990;
 tar_x[24]=5.816729;
 tar_y[24]=0.713581;
 tar_x[25]=6.008043;
 tar_y[25]=0.776134;
 tar_x[26]=6.188371;
 tar_y[26]=0.829324;
 tar_x[27]=6.359057;
 tar_y[27]=0.914519;
 tar_x[28]=6.579622;
 tar_y[28]=1.012904;
 tar_x[29]=6.772966;
 tar_y[29]=1.127265;
 tar_x[30]=6.959725;
 tar_y[30]=1.266403;
 tar_x[31]=7.169576;
 tar_y[31]=1.434016;
 tar_x[32]=7.379439;
 tar_y[32]=1.594235;
 tar_x[33]=7.575871;
 tar_y[33]=1.723814;
 tar_x[34]=7.789216;
 tar_y[34]=1.851912;
 tar_x[35]=8.008689;
 tar_y[35]=1.929586;
 tar_x[36]=8.219540;
 tar_y[36]=1.993113;
 tar_x[37]=8.455388;
 tar_y[37]=2.032664;
 tar_x[38]=8.698713;
 tar_y[38]=2.062177;
 tar_x[39]=8.973379;
 tar_y[39]=2.072125;
 tar_x[40]=9.272554;
 tar_y[40]=2.083026;
 tar_x[41]=9.585761;
 tar_y[41]=2.104136;
 tar_x[42]=9.869749;
 tar_y[42]=2.108367;
 tar_x[43]=10.162581;
 tar_y[43]=2.097966;
 tar_x[44]=10.454621;
 tar_y[44]=2.096026;
 tar_x[45]=10.754468;
 tar_y[45]=2.085315;
 tar_x[46]=11.049094;
 tar_y[46]=2.079002;
 tar_x[47]=11.347845;
 tar_y[47]=2.076082;
 tar_x[48]=11.636951;
 tar_y[48]=2.070308;
 tar_x[49]=11.945019;
 tar_y[49]=2.056054;
 tar_x[50]=12.229628;
 tar_y[50]=2.052789;
 tar_x[51]=12.537690;
 tar_y[51]=2.040585;
 tar_x[52]=12.839373;
 tar_y[52]=2.028260;
 tar_x[53]=13.139316;
 tar_y[53]=2.027685;
 tar_x[54]=13.442912;
 tar_y[54]=2.024657;
 tar_x[55]=13.750995;
 tar_y[55]=2.021456;
 tar_x[56]=14.070208;
 tar_y[56]=2.013849;
 tar_x[57]=14.372508;
 tar_y[57]=2.006390;
 tar_x[58]=14.674674;
 tar_y[58]=1.993526;
 tar_x[59]=14.965075;
 tar_y[59]=1.998911;
 tar_x[60]=15.241080;
 tar_y[60]=1.974475;
 tar_x[61]=15.504838;
 tar_y[61]=1.941569;
 tar_x[62]=15.753900;
 tar_y[62]=1.902834;
 tar_x[63]=16.000023;
 tar_y[63]=1.844031;
 tar_x[64]=16.267655;
 tar_y[64]=1.771776;
 tar_x[65]=16.520372;
 tar_y[65]=1.678138;
 tar_x[66]=16.762767;
 tar_y[66]=1.585371;
 tar_x[67]=17.012922;
 tar_y[67]=1.458804;
 tar_x[68]=17.269632;
 tar_y[68]=1.332957;
 tar_x[69]=17.509733;
 tar_y[69]=1.197989;
 tar_x[70]=17.746707;
 tar_y[70]=1.057488;
 tar_x[71]=17.992138;
 tar_y[71]=0.888828;
 tar_x[72]=18.234263;
 tar_y[72]=0.728747;
 tar_x[73]=18.484118;
 tar_y[73]=0.574780;
 tar_x[74]=18.720615;
 tar_y[74]=0.440497;
 tar_x[75]=18.961880;
 tar_y[75]=0.331899;
 tar_x[76]=19.213462;
 tar_y[76]=0.259629;
 tar_x[77]=19.453043;
 tar_y[77]=0.208457;
 tar_x[78]=19.682085;
 tar_y[78]=0.184649;
 tar_x[79]=19.908641;
 tar_y[79]=0.159686;
 tar_x[80]=20.190897;
 tar_y[80]=0.150583;
 tar_x[81]=20.482196;
 tar_y[81]=0.144939;
 tar_x[82]=20.780443;
 tar_y[82]=0.134820;
 tar_x[83]=21.073598;
 tar_y[83]=0.123064;
 tar_x[84]=21.367376;
 tar_y[84]=0.112563;
 tar_x[85]=21.669544;
 tar_y[85]=0.096238;
 tar_x[86]=21.956350;
 tar_y[86]=0.091248;
 tar_x[87]=22.247331;
 tar_y[87]=0.086627;
 tar_x[88]=22.519442;
 tar_y[88]=0.061212;
 tar_x[89]=22.751867;
 tar_y[89]=0.029334;
 tar_x[90]=22.976904;
 tar_y[90]=-0.018035;
 tar_x[91]=23.200309;
 tar_y[91]=-0.092528;
 tar_x[92]=23.414557;
 tar_y[92]=-0.183230;
 tar_x[93]=23.622188;
 tar_y[93]=-0.300217;
 tar_x[94]=23.800468;
 tar_y[94]=-0.431345;
 tar_x[95]=23.961253;
 tar_y[95]=-0.574571;
 tar_x[96]=24.118376;
 tar_y[96]=-0.727459;
 tar_x[97]=24.276019;
 tar_y[97]=-0.920558;
 tar_x[98]=24.423540;
 tar_y[98]=-1.132979;
 tar_x[99]=24.592776;
 tar_y[99]=-1.352609;
 tar_x[100]=24.765100;
 tar_y[100]=-1.598453;
 tar_x[101]=24.940328;
 tar_y[101]=-1.840750;
 tar_x[102]=25.099773;
 tar_y[102]=-2.064485;
 tar_x[103]=25.281294;
 tar_y[103]=-2.321309;

 curv[0]=0.020317;
curv[1]=0.020317;
curv[2]=0.020205;
curv[3]=0.018635;
curv[4]=0.016568;
curv[5]=0.014231;
curv[6]=0.006090;
curv[7]=0.001911;
curv[8]=0.009753;
curv[9]=0.016990;
curv[10]=0.025270;
curv[11]=0.019368;
curv[12]=0.013537;
curv[13]=0.007844;
curv[14]=0.002137;
curv[15]=0.003576;
curv[16]=0.015523;
curv[17]=0.027419;
curv[18]=0.039470;
curv[19]=0.051766;
curv[20]=0.063340;
curv[21]=0.073023;
curv[22]=0.214346;
curv[23]=0.350043;
curv[24]=0.452323;
curv[25]=0.502477;
curv[26]=0.407417;
curv[27]=0.327793;
curv[28]=0.241688;
curv[29]=0.180177;
curv[30]=0.130573;
curv[31]=0.021160;
curv[32]=0.084591;
curv[33]=0.196482;
curv[34]=0.351945;
curv[35]=0.571293;
curv[36]=0.535041;
curv[37]=0.438297;
curv[38]=0.303327;
curv[39]=0.136922;
curv[40]=0.046221;
curv[41]=0.028818;
curv[42]=0.013029;
curv[43]=0.003251;
curv[44]=0.019488;
curv[45]=0.036157;
curv[46]=0.028417;
curv[47]=0.020570;
curv[48]=0.012983;
curv[49]=0.004905;
curv[50]=0.002555;
curv[51]=0.008250;
curv[52]=0.013831;
curv[53]=0.019384;
curv[54]=0.025008;
curv[55]=0.030716;
curv[56]=0.012452;
curv[57]=0.053323;
curv[58]=0.094037;
curv[59]=0.132583;
curv[60]=0.167811;
curv[61]=0.177524;
curv[62]=0.184938;
curv[63]=0.190275;
curv[64]=0.193555;
curv[65]=0.194079;
curv[66]=0.178581;
curv[67]=0.163102;
curv[68]=0.148042;
curv[69]=0.134871;
curv[70]=0.122815;
curv[71]=0.023060;
curv[72]=0.071427;
curv[73]=0.181510;
curv[74]=0.314096;
curv[75]=0.493748;
curv[76]=0.436356;
curv[77]=0.341273;
curv[78]=0.230705;
curv[79]=0.114062;
curv[80]=0.033587;
curv[81]=0.014233;
curv[82]=0.005553;
curv[83]=0.025013;
curv[84]=0.044559;
curv[85]=0.064704;
curv[86]=0.052630;
curv[87]=0.171212;
curv[88]=0.278361;
curv[89]=0.359980;
curv[90]=0.420492;
curv[91]=0.441733;
curv[92]=0.435397;
curv[93]=0.406410;
curv[94]=0.368459;
curv[95]=0.328482;
curv[96]=0.289049;
curv[97]=0.249877;
curv[98]=0.215420;
curv[99]=0.179820;
curv[100]=0.148480;
curv[101]=0.121671;
curv[102]=0.101352;
curv[103]=0.082342;

 tar_theta[0]=0.000000;
 tar_theta[1]=-4.280959;
 tar_theta[2]=-0.539608;
 tar_theta[3]=0.820510;
 tar_theta[4]=-1.226421;
 tar_theta[5]=-1.732047;
 tar_theta[6]=-1.106608;
 tar_theta[7]=0.479959;
 tar_theta[8]=-2.202238;
 tar_theta[9]=0.335146;
 tar_theta[10]=-1.213904;
 tar_theta[11]=4.665716;
 tar_theta[12]=-0.804324;
 tar_theta[13]=-0.589770;
 tar_theta[14]=-0.148080;
 tar_theta[15]=-1.223545;
 tar_theta[16]=-0.448060;
 tar_theta[17]=-0.021688;
 tar_theta[18]=0.723288;
 tar_theta[19]=-0.766161;
 tar_theta[20]=-0.599422;
 tar_theta[21]=-1.113869;
 tar_theta[22]=-0.422735;
 tar_theta[23]=6.051899;
 tar_theta[24]=18.105938;
 tar_theta[25]=16.434083;
 tar_theta[26]=26.525293;
 tar_theta[27]=24.039690;
 tar_theta[28]=30.603881;
 tar_theta[29]=36.686613;
 tar_theta[30]=38.615197;
 tar_theta[31]=37.359768;
 tar_theta[32]=33.411375;
 tar_theta[33]=30.981723;
 tar_theta[34]=19.489453;
 tar_theta[35]=16.766955;
 tar_theta[36]=9.519750;
 tar_theta[37]=6.915650;
 tar_theta[38]=2.074263;
 tar_theta[39]=2.086756;
 tar_theta[40]=3.855876;
 tar_theta[41]=0.853559;
 tar_theta[42]=-2.034214;
 tar_theta[43]=-0.380606;
 tar_theta[44]=-2.045824;
 tar_theta[45]=-1.227498;
 tar_theta[46]=-0.559993;
 tar_theta[47]=-1.144154;
 tar_theta[48]=-2.649129;
 tar_theta[49]=-0.657261;
 tar_theta[50]=-2.268609;
 tar_theta[51]=-2.339469;
 tar_theta[52]=-0.109838;
 tar_theta[53]=-0.571437;
 tar_theta[54]=-0.595285;
 tar_theta[55]=-1.365128;
 tar_theta[56]=-1.413439;
 tar_theta[57]=-2.437760;
 tar_theta[58]=1.062332;
 tar_theta[59]=-5.059469;
 tar_theta[60]=-7.111381;
 tar_theta[61]=-8.840023;
 tar_theta[62]=-13.437054;
 tar_theta[63]=-15.108450;
 tar_theta[64]=-20.330934;
 tar_theta[65]=-20.942350;
 tar_theta[66]=-26.837326;
 tar_theta[67]=-26.115490;
 tar_theta[68]=-29.341652;
 tar_theta[69]=-30.663544;
 tar_theta[70]=-34.496821;
 tar_theta[71]=-33.470694;
 tar_theta[72]=-31.642431;
 tar_theta[73]=-29.587911;
 tar_theta[74]=-24.233423;
 tar_theta[75]=-16.027358;
 tar_theta[76]=-12.056616;
 tar_theta[77]=-5.934355;
 tar_theta[78]=-6.287753;
 tar_theta[79]=-1.847198;
 tar_theta[80]=-1.109983;
 tar_theta[81]=-1.943200;
 tar_theta[82]=-2.296425;
 tar_theta[83]=-2.047148;
 tar_theta[84]=-3.092469;
 tar_theta[85]=-0.996761;
 tar_theta[86]=-0.909824;
 tar_theta[87]=-5.335911;
 tar_theta[88]=-7.809615;
 tar_theta[89]=-11.886900;
 tar_theta[90]=-18.440642;
 tar_theta[91]=-22.945395;
 tar_theta[92]=-29.398522;
 tar_theta[93]=-36.335120;
 tar_theta[94]=-41.694396;
 tar_theta[95]=-44.217343;
 tar_theta[96]=-50.772339;
 tar_theta[97]=-55.221001;
 tar_theta[98]=-52.383934;
 tar_theta[99]=-54.971521;
 tar_theta[100]=-54.125731;
 tar_theta[101]=-54.524385;
 tar_theta[102]=-54.747658;
 tar_theta[103]=-54.747658;


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
  p_size=103;
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
  
  index_five=min_ind;
  d_five=points_dis(tar_x[min_ind],tar_y[min_ind],tar_x[index_five],tar_y[index_five]);
  while(d_five<1.5)
  {
    index_five++;
    if(index_five>103)
    {
      index_five=103;
      break;
    }
    d_five=points_dis(tar_x[min_ind],tar_y[min_ind],tar_x[index_five],tar_y[index_five]);
  }

  index_three=min_ind;
  d_three=points_dis(tar_x[min_ind],tar_y[min_ind],tar_x[index_three],tar_y[index_three]);
  while(d_three<3)
  {
    index_three++;
    if(index_three>103)
    {
      index_three=103;
      break;
    }
    d_three=points_dis(tar_x[min_ind],tar_y[min_ind],tar_x[index_three],tar_y[index_three]);
  }

  //设定线速度
  if(curv[min_ind]<0.1 || curv[min_ind]==0.1)
  {
    d_curv=2;
    v_pp=1;
  }
  if(curv[min_ind]>0.1 && curv[min_ind]<=0.2)
  {
    d_curv=1.5;   
    v_pp=0.8;
  }
  if(curv[min_ind]>0.2 && curv[min_ind]<0.3)
  {
    d_curv=1;   
    v_pp=0.8;
  }                  

  if(curv[min_ind]>0.3 || curv[min_ind]==0.3)
  {
    d_curv=0.8;
    v_pp=0.6;
  }
  v_pp=0.6;
  d_ym=fuzzy.realize(curv[min_ind],curv[index_five],curv[index_three]);
  //d_ym=0.8;
  target_x=tar_x[num_];
  target_y=tar_y[num_];
  d=sqrt((target_x-x)*(target_x-x)+(target_y-y)*(target_y-y));
  d_tar=sqrt((target_x-tar_x[min_ind])*(target_x-tar_x[min_ind])+(target_y-tar_y[min_ind])*(target_y-tar_y[min_ind]));
   // while ((d<d_ym || d_tar<1.2*d_min)&& ros::ok())
    while (d<d_ym && ros::ok())
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
  yaw_rate[re_count]=msg_twist.angular.z;


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
  printf("x end\n");
  show_y(re_count);
  printf("y end\n");
  show_d(re_count);
  printf("d end\n");
  show_w(re_count);
  printf("theta end\n");
  show_yaw(re_count);
  return 0;

}