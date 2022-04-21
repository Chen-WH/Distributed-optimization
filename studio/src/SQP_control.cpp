#define EIGEN_USE_MKL_ALL
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/TwistStamped.h>
//#include "offb_posctl/controlstate.h"
#include "mkl.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace Eigen;
using namespace std;

//User define
constexpr int n = 7;                 // 点的个数
constexpr int var_num = 7*n;          //目标函数变量个数
constexpr int ec_num = 5*n;           //等号约束个数
float t0 = 0;
float tf = 2;
float g = 9.8;
float k[2] = {50, 50};
float c[2] = {1.5, 0.38};
float epsilon1 = 1e-3;    //迭代停止下限
float epsilon2 = 1e-5;    //线搜索精度
float epsilon3 = 1e-5;    //偏导精度
float epsilon4 = 1;       //迭代停止的等号约束下限
float lambda = 1;     //等号约束拉格朗日乘子
int discretized_point_persecond = 50;
int pointnumber = tf * discretized_point_persecond;  // 离散点数

VectorXf x0(5);
VectorXf w(7);
Matrix<float, 7, 8, RowMajor> D;
Matrix<float, 8, 8, RowMajor> L1;
Matrix<float, 7, 7, RowMajor> L2;

//System define
constexpr int total_num = var_num + ec_num;
// offb_posctl::controlstate controlstate_msg;
float c0 = (tf - t0)/2;
float* tau = new float[pointnumber];
float gap = 2/(pointnumber - 1);

//目标函数
inline float f(VectorXf guess) {
	float ans;
	ArrayXf X1 = guess.segment(0, n).array();
	ArrayXf X2 = guess.segment(n, n).array();
	ArrayXf U1 = guess.segment(5*n, n).array();
	ArrayXf U2 = guess.segment(6*n, n).array();
	ans = w.dot((0.5*(U1 - g)*(U1 - g) + 0.5*U2*U2 + k[0]*(X1 + 3)*(X1 + 3) + k[1]*(X1*U2 + X2)*(X1*U2 + X2)).matrix());
	return ans*c0;
}

// 等号约束
VectorXf h(VectorXf guess) {
	VectorXf ans(ec_num);
	VectorXf X1 = guess.segment(0, n);
	VectorXf X2 = guess.segment(n, n);
	VectorXf X3 = guess.segment(2*n, n);
	VectorXf X4 = guess.segment(3*n, n);
	VectorXf X5 = guess.segment(4*n, n);
	VectorXf U1 = guess.segment(5*n, n);
	VectorXf U2 = guess.segment(6*n, n);
	VectorXf X1_plus(8);
	VectorXf X2_plus(8);
	VectorXf X3_plus(8);
	VectorXf X4_plus(8);
	VectorXf X5_plus(8);
	X1_plus << x0(0), X1;
	X2_plus << x0(1), X2;
	X3_plus << x0(2), X3;
	X4_plus << x0(3), X4;
	X5_plus << x0(4), X5;
	VectorXf Ceq1(7);
	VectorXf Ceq2(7);
	VectorXf Ceq3(7);
	VectorXf Ceq4(7);
	VectorXf Ceq5(7);
	Ceq1 = D*X1_plus - c0*X3;
	Ceq2 = D*X2_plus - c0*X4;
	Ceq3 = D*X3_plus - c0*(g*U2 - c[0]*X5);
	Ceq4 = D*X4_plus - c0*(U1.array() - g - c[1]*X5.array()).matrix();
	Ceq5 = D*X5_plus - c0*(g*U2 - c[0]*X5);
	ans << Ceq1, Ceq2, Ceq3, Ceq4, Ceq5;
	return (ans.array().pow(2)*lambda).matrix();
}

//惩罚函数
inline float penalty(VectorXf guess) {
	float ans = f(guess);
	VectorXf tmp = h(guess); 
	for (int i = 0; i < ec_num; ++i){
		ans += tmp(i);
	}
	return ans;
}

//f偏导函数
float fpartial(int x, VectorXf guess) {
	VectorXf tmp = guess;
	tmp(x) += epsilon3;
	return (f(tmp) - f(guess))/epsilon3;
}

float penalty_partial(int x, VectorXf guess) {
	VectorXf tmp = guess;
	tmp(x) += epsilon3;
	return (penalty(tmp) - penalty(guess))/epsilon3;
}

//f二阶偏导函数
float Qpartial(int x, int y, VectorXf guess) {
	VectorXf tmp = guess;
	tmp(x) += epsilon3;
	return (fpartial(y, tmp) - fpartial(y, guess))/epsilon3;
}

//h偏导函数
VectorXf hpartial(int x, VectorXf guess){
	VectorXf tmp = guess;
	tmp(x) += epsilon3;
	return (h(tmp) - h(guess))/epsilon3;
}

//gradient函数
VectorXf Gradient(VectorXf guess) {
	VectorXf tmp(total_num);
	for (int i = 0; i < var_num; ++i){
		tmp(i) = fpartial(i, guess);
	}
	tmp.segment(var_num, ec_num) = h(guess);
	//cout << tmp;
	return tmp;
}

VectorXf penalty_Gradient(VectorXf guess) {
	VectorXf tmp(var_num);
	for (int i = 0; i < var_num; ++i){
		tmp(i) = penalty_partial(i, guess);
	}
	return tmp;
}

//Hessian矩阵函数
MatrixXf Hessian(VectorXf guess) {
	Matrix<float, total_num, total_num, RowMajor> tmp;
	//赋值 \nabla^2 f
	for (int i = 0; i < var_num; ++i){
		for (int j = i; j < var_num; ++j){
			tmp(i, j) = tmp(j, i) = Qpartial(i, j, guess);
		}
	}
	//赋值 \nabla h 
	for (int i = 0; i < var_num; ++i){
		tmp.block(var_num, i, ec_num, 1) = hpartial(i, guess);
	}
	tmp.block(0, var_num, var_num, ec_num) = tmp.block(var_num, 0, ec_num, var_num).transpose();
	/*
	for (int i = 0; i < var_num; ++i){
		for (int j = var_num ; j < total_num; ++j){
			tmp(i, j) = tmp(j, i);
		}
	}*/
	//赋值零矩阵
	for (int i = var_num; i < total_num; ++i){
		for (int j = var_num; j < total_num; ++j){
			tmp(i, j) = 0;
		}
	}
	return tmp;
}

//一维线搜索函数
VectorXf linesearch(VectorXf gradient, VectorXf guess) {
	float alpha1 = 0;
	float alpha2 = 0.382;
	float alpha3 = 0.618;
	float alpha4 = 1;
	float tp1, tp2;
	while (alpha4 - alpha1 > epsilon2) {
		tp1 = penalty(guess - alpha2 * gradient.segment(0, var_num));
		tp2 = penalty(guess - alpha3 * gradient.segment(0, var_num));
		if (tp1 < tp2) {
			alpha4 = alpha3;
			alpha3 = alpha2;
			alpha2 = alpha1 + 0.382 * (alpha4 - alpha1);
		}
		else {
			alpha1 = alpha2;
			alpha2 = alpha3;
			alpha3 = alpha1 + 0.618 * (alpha4 - alpha1);
		}
	}
	return alpha2*gradient.segment(0, var_num);
}
//QP子问题
VectorXf SubQP(VectorXf guess) {
	VectorXf gradient = Gradient(guess);
	VectorXf penalty_gradient = penalty_Gradient(guess);
	//cout << gradient;
	MatrixXf hessian = Hessian(guess);
	/*
	for (int i = 0; i < var_num; ++i){
					for (int j = var_num; j < total_num; ++j){
									hessian(i, j) = fabs(hessian(i, j));
					}
	}
	*/
	//cout << hessian;
	VectorXf p = hessian.lu().solve(gradient);
	VectorXf p_sqp = linesearch(p, guess);
	VectorXf p_dec = linesearch(penalty_gradient, guess);
	if (penalty(guess - p_sqp) < penalty(guess - p_dec)){
		return p_sqp;
	}
	else {
		return p_dec;
	}
}

/*
void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    x0[4] = *msg.twist.linear.x;
}

void pos_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    x0[0] = *msg.pose.pose.position.x;
	x0[1] = *msg.pose.pose.position.z;
	x0[2] = *msg.twist.twist.linear.x;
	x0[3] = *msg.twist.twist.linear.z;
}
*/

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "SQP_control");
	//ros::NodeHandle n;
	//ros::Subscriber current_relativepostwist_pub = nh.subscribe<nav_msgs::Odometry>("current_relative_postwist", 1, pos_twist_cb);//当前状态方程中的状态量,即相对量
	//ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, plane_vel_cb); //twist
	//ros::Publisher controlstate_pub = nh.advertise<offb_posctl::controlstate>("SQP_controlstate", 1);
	//ros::Rate loop_rate(50);
	int count;
	clock_t start, end;
	bool flag;
	for (int i = 0; i < pointnumber; ++i){
		tau[i] = -1 + gap*i;
	}
	x0 << -5, 0, 0, 0, 0;
	w << 0.12948,0.27971,0.38183,0.41796,0.38183,0.27971,0.12948;
	D << -1.2478496e+01 ,  1.0081238e+01 ,  2.9698248e+00 ,-7.8567250e-01 ,  3.0587843e-01, -1.3313413e-01  , 5.4117220e-02, -1.3755260e-02,
		3.9856107e+00, -7.8146918e+00, 2.2215730e+00, 2.0625702e+00, -6.3507577e-01, 2.5503411e-01, -1.0007313e-01, 2.5052668e-02,
  		-2.5041560e+00 ,  4.3125940e+00, -4.3025381e+00,   1.1971894e+00 ,  1.6759174e+00, -5.2068108e-01  , 1.8682242e-01 ,-4.5148013e-02,
   		2.1875000e+00 ,-3.6292756e+00 ,  2.8636215e+00 ,-3.6226525e+00  , 1.0000000e+00 ,  1.5310481e+00 ,-4.2500350e-01 ,  9.4762025e-02,
  		-2.5041560e+00 ,  4.0913073e+00, -2.9784475e+00 ,  2.9150599e+00, -3.9654314e+00 ,  1.1971894e+00  , 1.5109130e+00 ,-2.6643471e-01,
   		3.9856107e+00, -6.4649133e+00 ,  4.5432192e+00 ,-4.0659236e+00 ,  4.2790627e+00 ,-5.8734597e+00  , 2.2215730e+00 ,  1.3748311e+00,
  		-1.2478496e+01 ,  2.0176230e+01, -1.3965105e+01 ,  1.2064598e+01, -1.1714789e+01 ,  1.2717136e+01 ,-1.6880812e+01 ,  1.0081238e+01;
	L1 << -2.6812000e+01 ,  7.2910000e-15 ,  4.3312000e+01,  0.0 , -1.9687000e+01  , 9.1137000e-16  , 2.1875000e+00 , 0.0,
   		4.2221000e+01 ,  2.1487000e+00 , -7.0242000e+01 , -1.5354000e+00 ,  3.2458000e+01,   1.9461000e-01 , -3.6293000e+00 , 0.0,
  		-2.6028000e+01 , -6.7273000e+00  , 4.7033000e+01 ,  7.1681000e+00 , -2.4427000e+01 , -9.9815000e-01  , 2.8636000e+00 , 0.0,
   		1.8021000e+01 ,  1.0707000e+01,  -3.3456000e+01 , -1.5533000e+01 ,  1.9536000e+01 ,  5.3035000e+00,  -3.6227000e+00 , 0.0,
  		-1.2257000e+01  ,-1.2257000e+01 ,  1.9800000e+01 ,  1.9800000e+01 , -9.0000000e+00 , -9.0000000e+00 ,  1.0000000e+00,   1.0,
   		7.6162000e+00 ,  1.0707000e+01 , -7.9576000e+00 , -1.5533000e+01 , -7.1154000e-01 ,  5.3035000e+00  , 1.5310000e+00 , 0.0,
  		-3.8629000e+00 , -6.7273000e+00 ,  1.2515000e+00  , 7.1681000e+00 ,  2.4790000e+00 , -9.9815000e-01 , -4.2500000e-01 , 0.0,
   		1.1024000e+00 ,  2.1487000e+00 ,  2.5854000e-01 , -1.5354000e+00 , -6.4782000e-01  , 1.9461000e-01 ,  9.4762000e-02 ,  0.0;
	L2 << 2.1487000e+00 , -2.0393000e+00 , -1.5354000e+00  , 1.4573000e+00,   1.9461000e-01,  -1.8470000e-01 , 0.0,
  		-6.7273000e+00  , 4.9885000e+00,   7.1681000e+00 , -5.3154000e+00 , -9.9815000e-01 ,  7.4016000e-01 , 0.0,
   		1.0707000e+01 , -4.3455000e+00,  -1.5533000e+01 ,  6.3039000e+00 ,  5.3035000e+00  ,-2.1524000e+00 , 0.0,
  		-1.2257000e+01 , -5.6148000e-15 ,  1.9800000e+01 , -3.7432000e-15 , -9.0000000e+00 ,  2.3395000e-16,   1.0,
   		1.0707000e+01 ,  4.3455000e+00,  -1.5533000e+01,  -6.3039000e+00  , 5.3035000e+00 ,  2.1524000e+00 , 0.0,
  		-6.7273000e+00 , -4.9885000e+00 ,  7.1681000e+00 ,  5.3154000e+00 , -9.9815000e-01,  -7.4016000e-01 , 0.0,
   		2.1487000e+00 ,  2.0393000e+00,  -1.5354000e+00 , -1.4573000e+00 ,  1.9461000e-01 ,  1.8470000e-01  , 0.0;
  	VectorXf guess(var_num);
  	//while (ros::ok())
  	//{
	count = 0;
	start = clock();
	flag = false;
	srand((unsigned)time(NULL));
	for (int i = 0; i < var_num; ++i) {
		guess(i) = rand() / float(RAND_MAX);
	}
	while (!flag && count < 200) {
		flag = true;
		VectorXf p = SubQP(guess);
		if (h(guess).norm() > epsilon4 || p.norm() > epsilon1) {
			flag = false;
		}
		guess = guess - p;
		++count;
		cout << f(guess) << ' ' << penalty(guess) << endl;
	}
	end = clock();
	cout << "结果" << f(guess) << endl << h(guess);
	printf("程序运行时间%fms\n", (float)(end - start)/CLOCKS_PER_SEC * 1000);
	printf("规划结果%f\n", f(guess));
	/*
	for (int i = 0;, i < 5; ++i){
		poly_x(i, 0) = x0[i];
	}
	for (int i = 0; i < 5; ++i){
		for (int j = 1; j < 8; ++j){
			poly_x(i, j) = x[i + 7*j - 7];
		}
	}
	for (int i = 0; i < 2; ++i){
		for (int j = 0; j < 7; ++j){
			poly_u(i, j) = x[i + 7*j + 5];
		}
	}
	Matrix res
	end = clock();
	printf("程序运行时间%lfms\n", (float)(end - start) / CLOCKS_PER_SEC * 1000);
    controlstate_msg.inicounter = 0;
    controlstate_msg.discrepointpersecond = discretized_point_persecond;
    controlstate_msg.arraylength = pointnumber;
	controlstate_msg.thrustarray = res[4,:];
    controlstate_msg.thetaarray = res[5, :];
    controlstate_msg.stateXarray = res[0, :];
    controlstate_msg.stateZarray = res[1, :];
    controlstate_msg.stateVXarray = res[2, :];
    controlstate_msg.stateVZarray = res[3, :];
	controlstate_pub.publish(controlstate_msg);
	*/
    //ros::spinOnce();
    //loop_rate.sleep();
	//}
  return 0;
}
