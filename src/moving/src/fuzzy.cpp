#include<iostream>
#include<iostream>
#include<string>
#include<map>


using namespace std;

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
		return 1.2;
	if (i == 0 && j == 0 && k == 1)
		return 0.8;
	if (i == 0 && j == 0 && k == 2)
		return 0.8;
	if (i == 0 && j == 1)
		return 0.8;
	if (i == 0 && j == 2)
		return 0.6;
	if (i == 2)
		return 0.6;
	if (i == 1 && j == 0)
		return 0.8;
	if (i == 1 && j == 1)
		return 0.8;
	if (i == 1 && j == 2)
		return 0.6;
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


int main()
{
	double u = 0;
	double zero_mf_paras[12] = { 0,0,0.1,0.15,0.1,0.2,0.25,0.3,0.25,0.35,0.5,0.5 };
	double five_mf_paras[12] = { 0,0,0.1,0.15,0.1,0.2,0.25,0.3,0.25,0.35,0.5,0.5 };
	double three_mf_paras[12] = { 0,0,0.1,0.15,0.1,0.2,0.25,0.3,0.25,0.35,0.5,0.5 };
	Fuzzy_controller fuzzy;
	fuzzy.setMf("trapmf", zero_mf_paras, "trapmf", five_mf_paras, "trapmf", three_mf_paras);
	u = fuzzy.realize(0.13,0.1,0.1);
	cout << u<<endl;

	return 0;
}