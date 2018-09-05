// Copyright (c) 2014 Locomotec
//
#include <iostream>
#define pi 3.1416
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include <cv.h>
#include <highgui.h>
//#include "http_api.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <ctime>


using namespace youbot;
using namespace cv;
using namespace std;

class exp_point {
public:
	double x;
	double y;
	double z;

};

class img_point{
public:
	double x;
	double y;
	double z;
	double alpha;
	double beta;
	double gamma;


};



double deg2rad = pi / 180;

double rad2deg = 180 / pi;

double theta1_zero = 2.56244;
double theta2_zero = 1.04883;
double theta3_zero = -2.43523;
double theta4_zero = 1.73184;
double theta5_zero = 2.8761045;



///Inverse Kinemetics

cv::Mat T10 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T20 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T30 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T40 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T50 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T54inv = cv::Mat(4, 4, CV_64FC1);

double ang1 = 0.0, ang2 = 0.0, ang3 = 0.0, ang4 = 0.0, ang5 = 0.0;
double init1 = 0.0, init2 = 0.0, init3 = 0.0, init4 = 0.0, init5 = 0.0;
double sang1 = 0.0, sang2 = 0.0, sang3 = 0.0, sang4 = 0.0, sang5 = 0.0;
//////////////////////////力矩////////////////////////////

double c_2 = cos(ang2);
double c_3 = cos(ang3);
double c_4 = cos(ang4);
double c_5 = cos(ang5);
double s_2 = sin(ang2);
double s_3 = sin(ang3);
double s_4 = sin(ang4);
double  s_5 = sin(ang5);
double s_23 = sin(ang2 - ang3);
double s_34 = sin(ang3 - ang4);
double s2_2 = sin(2 * ang2);
double s2_5 = sin(2 * ang5);
double c_23 = cos(ang2 - ang3);
double c_34 = cos(ang3 - ang4);
double c_45 = cos(ang4 + ang5);
double c_4_5 = cos(ang4 - ang5);
double c_25 = cos(ang2 + ang5);
double c_2_5 = cos(ang2 - ang5);
double s2_23 = sin(2 * ang2 - 2 * ang3);
double c_345 = cos(ang3 - ang4 + ang5);
double c_34_5 = cos(ang3 - ang4 - ang5);
double c_235 = cos(ang2 - ang3 + ang5);
double c_23_5 = cos(ang2 - ang3 - ang5);
double c_234 = cos(ang2 - ang3 + ang4);
double s_234 = sin(ang2 - ang3 + ang4);
double s_223 = sin(2 * ang2 - ang3);
double s2_234 = sin(2 * ang2 - 2 * ang3 + 2 * ang4);
double s_2234 = sin(2 * ang2 - ang3 + ang4);
double c_2234 = cos(2 * ang2 - ang3 + ang4);
double c2_234 = cos(2 * ang2 - 2 * ang3 + 2 * ang4);
double c_22345 = cos(2 * ang2 - ang3 + ang4 + ang5);
double c_2234_5 = cos(2 * ang2 - ang3 + ang4 - ang5);
double c_2345 = cos(ang2 - ang3 + ang4 + ang5);
double s_2345 = sin(ang2 - ang3 + ang4 + ang5);
double s_234_5 = sin(ang2 - ang3 + ang4 - ang5);
double c_234_5 = cos(ang2 - ang3 + ang4 - ang5);
double s_234_25 = sin(ang2 - ang3 + ang4 - 2 * ang5);
double s_23425 = sin(ang2 - ang3 + ang4 + 2 * ang5);
double s_22234 = sin(2 * ang2 - 2 * ang3 + ang4);
double c_22234 = cos(2 * ang2 - 2 * ang3 + ang4);
double c_22234_5 = cos(2 * ang2 - 2 * ang3 + ang4 - ang5);
double c_222345 = cos(2 * ang2 - 2 * ang3 + ang4 + ang5);
double s_2223245 = sin(2 * ang2 - 2 * ang3 + 2 * ang4 + ang5);
double s_222324_5 = sin(2 * ang2 - 2 * ang3 + 2 * ang4 - ang5);
double c_222324_5 = cos(2 * ang2 - 2 * ang3 + 2 * ang4 - ang5);
double c_2223245 = cos(2 * ang2 - 2 * ang3 + 2 * ang4 + ang5);
double s2_2345 = sin(2 * ang2 - 2 * ang3 + 2 * ang4 + 2 * ang5);
double s2_234_5 = sin(2 * ang2 - 2 * ang3 + 2 * ang4 - 2 * ang5);

double term_1 = (2259.0*sin(ang2 - ang3 + ang4 + ang5)) / 62500000.0;
double  term_2 = (753.0*cos(ang2 - ang3 + ang4 - ang5)) / 1000000000.0;
double term_3 = (3969783.0*c_4);
double  term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
double  term_5 = (753.0*c_5);
double  term_6 = (27027.0*s_4);
double  term_7 = (20331.0*c_5*s_4);
double  term_8 = (4557899.0*s_3*s_4);
double  term_9 = (4557899.0*c_3*c_4);
double  term_10 = (31031.0*c_3*s_4);
double  term_11 = (753.0*cos(ang2 - ang3 + ang4 + ang5));
double  term_12 = (31031.0*c_4*s_3);
double  term_13 = (2259.0*cos(ang2 - ang3 + ang4 - 2 * ang5));
double  term_14 = (2259.0*cos(ang2 - ang3 + ang4 + 2 * ang5));
double  term_15 = (2259.0*sin(ang2 - ang3 + ang4 - ang5));
double  term_16 = (20331.0*sin(ang2 - ang3 + ang5));
double  term_17 = (25433.0*cos(ang2 - ang3 + ang4));
double  term_18 = (23343.0*c_3*c_5*s_4);
double  term_19 = (23343.0*c_4*c_5*s_3);
cv::Mat M_matrix = cv::Mat(5, 5, CV_64FC1);//慣性
cv::Mat C_matrix = cv::Mat(5, 5, CV_64FC1);//科氏力
cv::Mat N_matrix = cv::Mat(5, 1, CV_64FC1);//重力
cv::Mat tau_matrix = cv::Mat(5, 1, CV_64FC1);//重力
cv::Mat ang_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角度
cv::Mat vel_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角速度
cv::Mat acc_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角加速度
////////////////////////力矩//////////////////////////////
cv::Mat Homogenous_matrix_T10 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T21 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T32 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T43 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T54 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_TBG = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_TB0 = cv::Mat(4, 4, CV_64FC1);




cv::Mat Homogenous_matrix = cv::Mat(4, 4, CV_64FC1); //T10 T21 T32 T43 T54 = T60
//ARM PARAMETERS
double d1 = 14.7;
double a2 = 15.5;
double a3 = 13.5;
double d5 = 11.3 + 10.5;

double xF, yF, zF;//Final Coordinate
double error_x, error_y, error_z;//定在全域
double error_ThetaX, error_ThetaY, error_ThetaZ;//定在全域


cv::Mat InverseKinemetics(double x, double y, double z, double alpha, double beta, double gamma);

std::ofstream Error("./Error.txt");//寫記事本
std::ofstream Errorx("./Errorx.txt");//寫記事本
std::ofstream Errory("./Errory.txt");//寫記事本
std::ofstream Errorz("./Errorz.txt");//寫記事本
std::ofstream Errorwx("./Errorwx.txt");//寫記事本
std::ofstream Errorwy("./Errorwy.txt");//寫記事本
std::ofstream Errorwz("./Errorwz.txt");//寫記事本
std::ofstream Command("./Command.txt");//寫記事本
std::ofstream desiredx("./desiredx.txt");//寫記事本
std::ofstream desiredy("./desiredy.txt");//寫記事本
std::ofstream desiredz("./desiredz.txt");//寫記事本
std::ofstream nowx("./nowx.txt");//寫記事本
std::ofstream nowy("./nowy.txt");//寫記事本
std::ofstream nowz("./nowz.txt");//寫記事本
std::ofstream nowr("./nowr.txt");//寫記事本
std::ofstream nowa("./nowa.txt");//寫記事本
std::ofstream nowb("./nowb.txt");//寫記事本
std::ofstream nowtime("./nowtime.txt");//寫記事本
std::ofstream CurrentABC("./CurrentABC.txt");//寫記事本
std::ofstream CurrentAngle("./CurrentAngle.txt");//寫記事本
std::ofstream nowvel("./nowvel.txt");//寫記事本
std::ofstream JointVelocity("./JointVelocity.txt");//寫記事本
int special;
int bomb;
int upg;
clock_t start, stop;

int main()
{

	// configuration flags for different system configuration (e.g. base without arm)
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	// define velocities
	double vLinear = 0.05; //meter_per_second
	double vRotate = 0.2; //radian_per_second
	double trspeed = 0.05;

	// create handles for youBot base and manipulator (if available)
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;


	//////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////----------------/////////	//////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	printf("%d \n", atoi("54321"));
	printf("%d \n", atoi("54.321"));
	printf("%d \n", atoi("$54321"));
	/* configuration flags for different system configuration (e.g. base without arm)*/


	bool youBotHasGripper = false;

	/* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	/* create handles for youBot base and manipulator (if available) */
	YouBotGripper* myYouBotGripper;



	try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);//YOUBOT_CONFIGURATIONS_DIR
		myYouBotBase->doJointCommutation();

		youBotHasBase = true;
	}
	catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);//YOUBOT_CONFIGURATIONS_DIR
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();
		cout << "calibrateManipulator done" << endl;

		youBotHasArm = true;
		youBotHasGripper = true;
	}
	catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
		youBotHasGripper = false;
	}


	/*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle;
	JointVelocitySetpoint desiredJointVelocity;
	JointTorqueSetpoint desiredJointTorque;

	//vector<JointSensedAngle> aaa;
	JointSensedAngle EachJointAngle;/////encoder讀出來的手臂角度
	vector<JointSensedAngle> EachJointAngle1;/////encoder讀出來的手臂角度存成vector型式
	JointSensedVelocity EachJointVelocity;//encoder讀出來的手臂速度
	vector<JointSensedVelocity> EachJointVelocity1;
	double exp_x, exp_y, exp_z;//期望位置
	double expn_x, expn_y, expn_z;
	double exp_gamma, exp_beta, exp_alpha;//期望角度alpha:z軸角度，beta:y軸角度，gamma:x軸角度
	double expn_gamma, expn_beta, expn_alpha;





	double sample_time = 0.15;

#pragma region 夾取成功(暫時註解)
	if (youBotHasGripper)
	{
		myYouBotManipulator->getArmGripper().open();
		SLEEP_MILLISEC(4000);
	}


	cout << "w a s d x" << endl;
#pragma endregion 夾取成功
	myYouBotManipulator->getArmGripper().open();
	SLEEP_MILLISEC(1000);
	//while (1){//////車子
	//	if (youBotHasBase) {
	//		// Variable for the base.
	//		// Here "boost units" is used to set values, that means you have to set a value and a unit.
	//		quantity<si::velocity> vx = 0 * meter_per_second;
	//		quantity<si::velocity> vy = 0 * meter_per_second;
	//		quantity<si::angular_velocity> va = 0 * radian_per_second;////另quantity必須要以算式去命名

	//		if (GetAsyncKeyState(0x57))	//w
	//		{
	//			// forward
	//			vx = vLinear * meter_per_second;
	//			vy = 0 * meter_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);

	//			SLEEP_MILLISEC(10);
	//		}
	//		else if (GetAsyncKeyState(0x53)) //s
	//		{
	//			// backwards
	//			vx = -vLinear * meter_per_second;
	//			vy = 0 * meter_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);
	//			//LOG(info) << "drive backwards";
	//			SLEEP_MILLISEC(10);
	//		}
	//		else if (GetAsyncKeyState(0x41)) //a
	//		{
	//			// left
	//			vx = 0 * meter_per_second;
	//			vy = vLinear * meter_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);
	//			//LOG(info) << "drive left";
	//			SLEEP_MILLISEC(10);
	//		}
	//		else if (GetAsyncKeyState(0x44)) //d
	//		{
	//			// right 
	//			vx = 0 * meter_per_second;
	//			vy = -vLinear * meter_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);
	//			//LOG(info) << "drive right";
	//			SLEEP_MILLISEC(10);
	//		}
	//		else if (GetAsyncKeyState(0x58)) //x
	//		{
	//			// stop base 
	//			vx = 0 * meter_per_second;
	//			vy = 0 * meter_per_second;
	//			va = 0 * radian_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);
	//			LOG(info) << "stop base";

	//			////////////////////
	//			desiredJointAngle.angle = 2.56244 * radian;
	//			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

	//			desiredJointAngle.angle = 1.04883 * radian;
	//			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

	//			desiredJointAngle.angle = -2.43523 * radian;
	//			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

	//			desiredJointAngle.angle = 1.73184 * radian;
	//			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
	//			LOG(info) << "unfold arm";
	//			SLEEP_MILLISEC(4000);
	//			//////////////////////////////////////////////////////////////

	//			break;
	//		}
	//		else if (GetAsyncKeyState(VK_ESCAPE)) break;
	//		else
	//		{
	//			// stop base 
	//			vx = 0 * meter_per_second;
	//			vy = 0 * meter_per_second;
	//			va = 0 * radian_per_second;
	//			myYouBotBase->setBaseVelocity(vx, vy, va);
	//			//LOG(info) << "stop base";
	//			//break;
	//		}
	//	}
	//}



	double end_x, end_y, end_z;
	cv::Mat tool_end = cv::Mat(4, 1, CV_64FC1);
	cv::Mat tool = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1);//////Mat_<double>对应的是CV_64F
	Mat R_ZYZ = cv::Mat(4, 4, CV_64FC1);
	Mat T01 = cv::Mat(4, 4, CV_64FC1);
	Mat T45 = cv::Mat(4, 4, CV_64FC1);
	Mat T04 = cv::Mat(4, 4, CV_64FC1);
	Mat T45inv;
	int count = 0;
	double r11;
	double r12;
	double r13;
	double r21;
	double r22;
	double r23;
	double r31;
	double r32;
	double r33;
	double r14;
	double r24;
	double r34;
	double bg11;
	double bg12;
	double bg13;
	double bg21;
	double bg22;
	double bg23;
	double bg31;
	double bg32;
	double bg33;
	double bg14;
	double bg24;
	double bg34;
	double bg44 = 1.0;
	double bg41 = 0.0;
	double bg42 = 0.0;
	double bg43 = 0.0;
	double b011;
	double b012;
	double b013;
	double b021;
	double b022;
	double b023;
	double b031;
	double b032;
	double b033;
	double b014;
	double b024;
	double b034;
	double b044 = 1.0;
	double b041 = 0.0;
	double b042 = 0.0;
	double b043 = 0.0;
	double zz;
	double x2;
	double y2;
	double z2;
	double x4;
	double y4;
	double z4;
	double theta1, theta2, theta3, theta4, theta5, theta234;
	double beta;
	double alpha;
	double gamma;
	double half_pi = pi / 2;
	int i = 0;
	double sang1, sang2, sang3, sang4, sang5;
	double svel1, svel2, svel3, svel4, svel5;
	double sacc1, sacc2, sacc3, sacc4, sacc5;
	double px, py, pz, p_gamma, p_beta, p_alpha;
	double ca, cb, cr, sa, sb, sr;
	double theta1_inv, theta2_inv, theta3_inv, theta4_inv, theta5_inv, theta234_inv;
	double cos_theta3;
	exp_point temp;
	vector<exp_point> test;
	img_point imgtemp;
	vector<img_point> imgtest;
	int ccc = 0;
	int xxx = 0;
	while (true)
	{

		if (count == 0)
		{
			cout << "1:";

			cin >> ang1;

			cout << "2:";

			cin >> ang2;

			cout << "3:";
			cin >> ang3;

			cout << "4:";
			cin >> ang4;

			cout << "5:";

			cin >> ang5;





			//初始姿態
			init1 = 20;//原本是20
			init2 = -10;
			init3 = -60;
			init4 = -70;
			init5 = 0;


			theta1 = theta2 = theta3 = theta4 = theta5 = theta234 = 0.0;
#pragma region 逆向運動學
			///////////////////////逆向運動學///////////////////////////
			//p_alpha = p_alpha*deg2rad;
			//p_beta = p_beta*deg2rad;
			//p_gamma = p_gamma*deg2rad;
			//ca = cos(p_alpha);
			//
			//cb = cos(p_beta);
			//cr = cos(p_gamma);
			//sa = sin(p_alpha);
			//sb = sin(p_beta);
			//sr = sin(p_gamma);

			//R_ZYZ.at<double>(0, 0) = ca*cb*cr - sa*sr; R_ZYZ.at<double>(0, 1) = -ca*cb*sr - sa*cr;		 R_ZYZ.at<double>(0, 2) = ca*sb; R_ZYZ.at<double>(0, 3) = px;
			//R_ZYZ.at<double>(1, 0) = sa*cb*cr + ca*sr; R_ZYZ.at<double>(1, 1) = -sa*cb*sr + ca*cr;		 R_ZYZ.at<double>(1, 2) = sa*sb; R_ZYZ.at<double>(1, 3) = py;
			//R_ZYZ.at<double>(2, 0) = -sb*cr;		R_ZYZ.at<double>(2, 1) = sb*sr;		 R_ZYZ.at<double>(2, 2) = cb;	      R_ZYZ.at<double>(2, 3) = pz;
			//R_ZYZ.at<double>(3, 0) = 0.0;		R_ZYZ.at<double>(3, 1) = 0.0;		 R_ZYZ.at<double>(3, 2) = 0.0;	      R_ZYZ.at<double>(3, 3) = 1.0;

			//theta1_inv = atan2(py, px);
			//theta1_inv = -theta1_inv;
			//theta1_inv = theta1_inv + 140 * deg2rad;

			//theta5_inv = atan2(cos(theta1_inv)*(sa*cb*cr + ca*sr) - sin(theta1_inv)*(ca*cb*cr - sa*sr), cos(theta1_inv)*(-sa*cb*sr + ca*cr) - sin(theta1_inv)*(-ca*cb*sr - sa*cr));
			//theta5_inv = theta5_inv;
			//theta234_inv = atan2(-cos(theta1_inv)*(ca*sb) - sin(theta1_inv)*(sa*sb), (cb));

			//T01.at<double>(0, 0) = cos(theta1_inv); T01.at<double>(0, 1) = 0;		 T01.at<double>(0, 2) = sin(theta1_inv); T01.at<double>(0, 3) = 0;
			//T01.at<double>(1, 0) = sin(theta1_inv); T01.at<double>(1, 1) = 0;		 T01.at<double>(1, 2) = -cos(theta1_inv); T01.at<double>(1, 3) = 0;
			//T01.at<double>(2, 0) = 0.0;				T01.at<double>(2, 1) = 1;		 T01.at<double>(2, 2) = 0;					T01.at<double>(2, 3) = d1;
			//T01.at<double>(3, 0) = 0.0;				T01.at<double>(3, 1) = 0.0;		 T01.at<double>(3, 2) = 0.0;	      T01.at<double>(3, 3) = 1.0;

			//T45.at<double>(0, 0) = cos(theta5_inv - 90); T45.at<double>(0, 1) = -sin(theta5_inv - 90);		 T45.at<double>(0, 2) = 0; T45.at<double>(0, 3) = 0;
			//T45.at<double>(1, 0) = sin(theta5_inv - 90); T45.at<double>(1, 1) = cos(theta5_inv - 90);		 T45.at<double>(1, 2) = 0; T45.at<double>(1, 3) = 0;
			//T45.at<double>(2, 0) = 0.0;					T45.at<double>(2, 1) = 0;							 T45.at<double>(2, 2) = 1;					T45.at<double>(2, 3) = d5;
			//T45.at<double>(3, 0) = 0.0;					T45.at<double>(3, 1) = 0.0;								T45.at<double>(3, 2) = 0.0;	      T45.at<double>(3, 3) = 1.0;

			//T45inv = T45.inv(DECOMP_SVD);

			//T04 = R_ZYZ*T45inv;

			//double l2 = 15.5;
			//double l3 = 13.5;
			//double pxn, pyn, pzn;
			//double l24;
			//x2 = 0;
			//y2 = 0;
			//z2 = d1;
			//x4 = T04.at<double>(0, 3);
			//y4 = T04.at<double>(1, 3);
			//z4 = T04.at<double>(2, 3);
			//pxn = x4 - x2;
			//pyn = y4 - y2;
			//pzn = z4 - z2;
			//l24 = sqrt((x4 - x2)*(x4 - x2) + (y4 - y2)*(y4 - y2) + (z4 - z2)*(z4 - z2));
			//cos_theta3 = (l24*l24 - l2*l2 - l3*l3) / (2 * l2*l3);
			//theta3_inv = atan2(sqrt(1 - cos_theta3*cos_theta3), cos_theta3);

			//theta2_inv = -atan2(l3*sin(theta3_inv), l2 + l3*cos(theta3_inv)) - atan2(pzn, sqrt(pxn*pxn + pyn*pyn));
			//theta2_inv = theta2_inv + 90*deg2rad;

			//theta4_inv = theta234_inv - theta2_inv - theta3_inv;

			///////////////////////////逆向運動學end//////////////////////////////

#pragma endregion 逆向運動學

			//half_pi *= deg2rad ;

			//ang2 = ang2 + 90 ;

			//ang1 = theta1_inv;//ANG1=ANG1*DEG2RAD
			//ang2 = theta2_inv;//逆向運動學得到的角度，逆向運動學其實只會用一次沒啥鬼屌用
			//ang3 = theta3_inv;
			//ang4 = theta4_inv;
			//ang5 = theta5_inv;

			init1 *= deg2rad;
			init2 *= deg2rad;
			init3 *= deg2rad;
			init4 *= deg2rad;
			init5 *= deg2rad;

			ang1 *= deg2rad;
			ang2 *= deg2rad;
			ang3 *= deg2rad;
			ang4 *= deg2rad;
			ang5 *= deg2rad;

			ang1 = -ang1;
			ang2 = -ang2;
			ang3 = -ang3;
			ang4 = -ang4;
			ang5 = -ang5;



			init1 = -init1;
			init2 = -init2;
			init3 = -init3;
			init4 = -init4;
			init5 = -init5;

			///====Forword=====//
			Homogenous_matrix_T10.at<double>(0, 0) = cos(ang1); Homogenous_matrix_T10.at<double>(0, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(0, 2) = sin(ang1); Homogenous_matrix_T10.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T10.at<double>(1, 0) = sin(ang1); Homogenous_matrix_T10.at<double>(1, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(1, 2) = -cos(ang1); Homogenous_matrix_T10.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T10.at<double>(2, 0) = 0.0;		Homogenous_matrix_T10.at<double>(2, 1) = 1.0;		 Homogenous_matrix_T10.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(2, 3) = d1;
			Homogenous_matrix_T10.at<double>(3, 0) = 0.0;		Homogenous_matrix_T10.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T21.at<double>(0, 0) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 1) = -sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(0, 3) = a2*cos(ang2 + pi / 2);
			Homogenous_matrix_T21.at<double>(1, 0) = sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 1) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(1, 3) = a2*sin(ang2 + pi / 2);
			Homogenous_matrix_T21.at<double>(2, 0) = 0.0;		Homogenous_matrix_T21.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T21.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T21.at<double>(3, 0) = 0.0;		Homogenous_matrix_T21.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T21.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T32.at<double>(0, 0) = cos(ang3); Homogenous_matrix_T32.at<double>(0, 1) = -sin(ang3); Homogenous_matrix_T32.at<double>(0, 2) = 0.0;       Homogenous_matrix_T32.at<double>(0, 3) = a3*cos(ang3);
			Homogenous_matrix_T32.at<double>(1, 0) = sin(ang3); Homogenous_matrix_T32.at<double>(1, 1) = cos(ang3); Homogenous_matrix_T32.at<double>(1, 2) = 0.0;       Homogenous_matrix_T32.at<double>(1, 3) = a3*sin(ang3);
			Homogenous_matrix_T32.at<double>(2, 0) = 0.0;		Homogenous_matrix_T32.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T32.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T32.at<double>(3, 0) = 0.0;		Homogenous_matrix_T32.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T32.at<double>(3, 3) = 1.0;
			///Origin
			Homogenous_matrix_T43.at<double>(0, 0) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 1) = 0.0;       Homogenous_matrix_T43.at<double>(0, 2) = -sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(1, 0) = sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 1) = 0.0;       Homogenous_matrix_T43.at<double>(1, 2) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(2, 0) = 0.0;		Homogenous_matrix_T43.at<double>(2, 1) = -1.0;		 Homogenous_matrix_T43.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(3, 0) = 0.0;		Homogenous_matrix_T43.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T43.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T54.at<double>(0, 0) = cos(ang5); Homogenous_matrix_T54.at<double>(0, 1) = -sin(ang5); Homogenous_matrix_T54.at<double>(0, 2) = 0.0;       Homogenous_matrix_T54.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T54.at<double>(1, 0) = sin(ang5); Homogenous_matrix_T54.at<double>(1, 1) = cos(ang5); Homogenous_matrix_T54.at<double>(1, 2) = 0.0;       Homogenous_matrix_T54.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T54.at<double>(2, 0) = 0.0;		Homogenous_matrix_T54.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T54.at<double>(2, 3) = d5;
			Homogenous_matrix_T54.at<double>(3, 0) = 0.0;		Homogenous_matrix_T54.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T54.at<double>(3, 3) = 1.0;
			///Origin




			T10 = Homogenous_matrix_T10;
			T20 = T10*Homogenous_matrix_T21;
			T30 = T20*Homogenous_matrix_T32;
			T40 = T30*Homogenous_matrix_T43;
			T50 = T40*Homogenous_matrix_T54;

			r11 = T50.at<double>(0, 0);
			r12 = T50.at<double>(0, 1);
			r13 = T50.at<double>(0, 2);
			r14 = T50.at<double>(0, 3);
			r21 = T50.at<double>(1, 0);
			r22 = T50.at<double>(1, 1);
			r23 = T50.at<double>(1, 2);
			r24 = T50.at<double>(1, 3);
			r31 = T50.at<double>(2, 0);
			r32 = T50.at<double>(2, 1);
			r33 = T50.at<double>(2, 2);
			r34 = T50.at<double>(2, 3);







			double show_ang1, show_ang2, show_ang3, show_ang4, show_ang5, show_ang0234;
			double t_ang1, t_ang2, t_ang3, t_ang4, t_ang5; ///before input youBot
			show_ang1 = ang1 * rad2deg;
			show_ang2 = ang2 * rad2deg;
			show_ang3 = ang3 * rad2deg;
			show_ang4 = ang4 * rad2deg;
			show_ang5 = ang5 * rad2deg;
			show_ang0234 = show_ang2 + show_ang3 + show_ang4;

			t_ang1 = ang1;
			t_ang2 = ang2;
			t_ang3 = ang3;
			t_ang4 = ang4;
			t_ang5 = ang5;

			///Because Assume opposite to real Arm.
			ang1 = -ang1;
			ang5 = -ang5;

			init1 = -init1;
			init5 = -init5;

			/*double theta1_zero = 2.56244;
			double theta2_zero = 1.04883;
			double theta3_zero = -2.43523;
			double theta4_zero = 1.73184;
			double theta5_zero = 2.8761045;*///從上面複製過來的 這是昊昊自己看的

			ang1 = ang1 + theta1_zero;
			ang2 = ang2 + theta2_zero;
			ang3 = ang3 + theta3_zero;
			ang4 = ang4 + theta4_zero;
			ang5 = ang5 + theta5_zero;


			init1 = init1 + theta1_zero;
			init2 = init2 + theta2_zero;
			init3 = init3 + theta3_zero;
			init4 = init4 + theta4_zero;
			init5 = init5 + theta5_zero;
			cout << "T10" << T10 << endl;
			cout << "T20" << T20 << endl;
			cout << "T30" << T30 << endl;
			cout << "T40" << T40 << endl;
			cout << "T50" << T50 << endl;
			cout << "給youbot角度 " << ang1 << " " << ang2 << "" << ang3 << " " << ang4 << " " << ang5 << endl;
			///Test Move
			cout << "Test Forward Move " << endl;


			//desiredJointAngle.angle = ang1 * radian;//2.9150354
			//myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
			//控制部分
			desiredJointAngle.angle = ang1 * radian;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

			desiredJointAngle.angle = ang2 * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = ang3 * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

			desiredJointAngle.angle = ang4 * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

			desiredJointAngle.angle = ang5 * radian;
			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

			myYouBotManipulator->getArmGripper().open();
			LOG(info) << "unfold arm";
			SLEEP_MILLISEC(3000);

			////////////////////////testtesttesttest///////////////////////////////////



			//////////////////////////////////////////////////////////////
			//算gamma,beta,alpha
			////////////////佳又論文P34?????????????????????????
			beta = atan2(sqrt(r31*r31 + r32*r32), r33);

			if (beta == 0.0)
			{
				alpha = 0;
				gamma = atan2(-r12, r11) * 180 / 3.1416;
			}

			else if (abs(180.0 - beta * 180 / 3.1416) < 1 && abs(180.0 - beta * 180 / 3.1416) > 0.0)
			{
				alpha = 0;
				gamma = atan2(r12, -r11) * 180 / 3.1416;
			}

			else
			{
				alpha = (atan2(r23 / sin(beta), r13 / sin(beta))) * 180 / 3.1416;
				gamma = (atan2(r32 / sin(beta), -r31 / sin(beta))) * 180 / 3.1416;
			}


			beta = beta * 180 / 3.1416;



			tool_end = T50*tool;//只有位移量?????????????????????????????


			end_x = tool_end.at<double>(0, 0);
			end_y = tool_end.at<double>(1, 0);
			end_z = tool_end.at<double>(2, 0);
			//cout << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
			printf("\n");
			//}
			int qaz;
			cout << "ok" << endl;
			cin >> qaz;
			cout << "BG矩陣" << endl;
			FILE *fPtr_floatx = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_x.txt", "r");
			float data_floatx[1];
			fscanf(fPtr_floatx, "%f", &data_floatx[0]);
			fclose(fPtr_floatx);
			FILE *fPtr_floaty = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_y.txt", "r");
			float data_floaty[1];
			fscanf(fPtr_floaty, "%f", &data_floaty[0]);
			fclose(fPtr_floaty);
			FILE *fPtr_floatz = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_z.txt", "r");
			float data_floatz[1];
			fscanf(fPtr_floatz, "%f", &data_floatz[0]);
			fclose(fPtr_floatz);




			cout << "bg11:";

			bg11 = 1;

			cout << "bg12:";

			bg12 = 0;

			cout << "bg13:";
			bg13 = 0;

			cout << "bg14:";
			bg14 = data_floatx[0];

			cout << "bg21:";
			bg21 = 0;

			cout << "bg22:";

			bg22 = 1;

			cout << "bg23:";
			bg23 = 0;

			cout << "bg24:";
			bg24 = data_floaty[0];

			cout << "bg31:";
			bg31 = 0;

			cout << "bg32:";

			bg32 = 0;

			cout << "bg33:";
			bg33 = 1;

			cout << "bg34:";
			bg34 = data_floatz[0];

			cout << "bg41:";
			bg41 = 0;

			cout << "bg42:";

			bg42 = 0;

			cout << "bg43:";
			bg43 = 0;

			cout << "bg44:";
			bg44 = 1;

			Homogenous_matrix_TBG.at<double>(0, 0) = bg11;
			Homogenous_matrix_TBG.at<double>(0, 1) = bg12;
			Homogenous_matrix_TBG.at<double>(0, 2) = bg13;
			Homogenous_matrix_TBG.at<double>(0, 3) = bg14;
			Homogenous_matrix_TBG.at<double>(1, 0) = bg21;
			Homogenous_matrix_TBG.at<double>(1, 1) = bg22;
			Homogenous_matrix_TBG.at<double>(1, 2) = bg23;
			Homogenous_matrix_TBG.at<double>(1, 3) = bg24;
			Homogenous_matrix_TBG.at<double>(2, 0) = bg31;
			Homogenous_matrix_TBG.at<double>(2, 1) = bg32;
			Homogenous_matrix_TBG.at<double>(2, 2) = bg33;
			Homogenous_matrix_TBG.at<double>(2, 3) = bg34;
			Homogenous_matrix_TBG.at<double>(3, 0) = bg41;
			Homogenous_matrix_TBG.at<double>(3, 1) = bg42;
			Homogenous_matrix_TBG.at<double>(3, 2) = bg43;
			Homogenous_matrix_TBG.at<double>(3, 3) = bg44;

			Homogenous_matrix_TB0 = Homogenous_matrix_TBG*T50;

			b011 = Homogenous_matrix_TB0.at<double>(0, 0);
			b012 = Homogenous_matrix_TB0.at<double>(0, 1);
			b013 = Homogenous_matrix_TB0.at<double>(0, 2);
			b014 = Homogenous_matrix_TB0.at<double>(0, 3);
			b021 = Homogenous_matrix_TB0.at<double>(1, 0);
			b022 = Homogenous_matrix_TB0.at<double>(1, 1);
			b023 = Homogenous_matrix_TB0.at<double>(1, 2);
			b024 = Homogenous_matrix_TB0.at<double>(1, 3);
			b031 = Homogenous_matrix_TB0.at<double>(2, 0);
			b032 = Homogenous_matrix_TB0.at<double>(2, 1);
			b033 = Homogenous_matrix_TB0.at<double>(2, 2);
			b034 = Homogenous_matrix_TB0.at<double>(2, 3);
			b041 = Homogenous_matrix_TB0.at<double>(3, 0);
			b042 = Homogenous_matrix_TB0.at<double>(3, 1);
			b043 = Homogenous_matrix_TB0.at<double>(3, 2);
			b044 = Homogenous_matrix_TB0.at<double>(3, 3);

			if (b033 < 1)
			{
				if (b033 > -1)
				{
					exp_beta = acos(b033);
					exp_alpha = atan2(b023, b013);
					exp_gamma = atan2(b032, -b031);
				}
				else
				{
					exp_beta = pi;
					exp_alpha = -atan2(b021, b022);
					exp_gamma = 0;
				}

			}
			else
			{
				exp_beta = 0;
				exp_alpha = -atan2(b021, b022);
				exp_gamma = 0;
			}

			exp_alpha = exp_alpha*rad2deg;
			exp_beta = exp_beta*rad2deg;
			exp_gamma = exp_gamma*rad2deg;

			exp_x = b014;
			exp_y = b024;
			exp_z = b034;
#pragma region 期望點切割

			//if ((exp_x - end_x) > 0 || (exp_x - end_x) < 0 || (exp_y - end_y) > 0 || (exp_y - end_y) < 0 || (exp_z - end_z) > 0 || (exp_z - end_z) < 0)//切割x y z，程式順序會造成追蹤的順序，0.3跟0.1為經度
			//{
			//	while (abs((exp_x - end_x)) > 0.3 || abs((exp_y - end_y)) > 0.3 || abs((exp_z - end_z)) > 0.3)
			//	{


			//		if ((exp_x - end_x) > 0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_x = end_x + 0.2;
			//			test.push_back(temp);
			//		}
			//		if ((exp_x - end_x) < -0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_x = end_x - 0.2;
			//			test.push_back(temp);
			//		}
			//		if ((exp_y - end_y) > 0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_y = end_y + 0.2;
			//			test.push_back(temp);
			//		}
			//		if ((exp_y - end_y) < -0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_y = end_y - 0.2;
			//			test.push_back(temp);
			//		}
			//		if ((exp_z - end_z) > 0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_z = end_z + 0.2;
			//			test.push_back(temp);
			//		}
			//		if ((exp_z - end_z) < -0.2)
			//		{
			//			temp.x = end_x;
			//			temp.y = end_y;
			//			temp.z = end_z;
			//			end_z = end_z - 0.2;
			//			test.push_back(temp);
			//		}

			//	}



			//}
			//ccc = test.size();
#pragma endregion 期望點切割



			cout << "T50: " << "\n" << r11 << " " << r12 << " " << r13 << " " << r14 << "\n" << " " << r21 << " " << r22 << " " << r23 << " " << r24 << "\n" << r31 << " " << r32 << " " << r33 << " " << r34 << endl;
			cout << "TB0: " << "\n" << b011 << " " << b012 << " " << b013 << " " << b014 << "\n" << " " << b021 << " " << b022 << " " << b023 << " " << b024 << "\n" << b031 << " " << b032 << " " << b033 << " " << b034 << endl;
			cout << "exp_gamma" << endl;
			cin >> exp_gamma;
			cout << "exp_beta" << endl;
			cin >> exp_beta;
			cout << "exp_alpha" << endl;
			cin >> exp_alpha;
			cout << "exp_x" << endl;
			cin >> exp_x;
			cout << "exp_y" << endl;
			cin >> exp_y;
			cout << "exp_z" << endl;
			cin >> exp_z;
		}




		double lamda_v, lamda_w;
		//正常5軸		
		lamda_v = lamda_w = 0.5;

		//4軸
		//lamda_v = lamda_w = 0.1;
#pragma region 4軸速度控制
		//		//		do
		//		//		{
		//		//
		//		//			//回傳算當前角度
		//		//			myYouBotManipulator->getJointData(EachJointAngle1);
		//		//			//sang1 = EachJointAngle1[0].angle.value();
		//		//			sang1 = theta1_zero;
		//		//			sang2 = EachJointAngle1[1].angle.value();
		//		//			sang3 = EachJointAngle1[2].angle.value();
		//		//			sang4 = EachJointAngle1[3].angle.value();
		//		//			sang5 = EachJointAngle1[4].angle.value();
		//		//			CurrentAngle << count << ":    " << sang1 << "  " << sang2 << "  " << sang3 << "  " << sang4 << "  " << sang5 << endl;
		//		//
		//		//			ang1 = sang1;
		//		//			ang2 = sang2;
		//		//			ang3 = sang3;
		//		//			ang4 = sang4;
		//		//			ang5 = sang5;
		//		//
		//		//			ang1 = ang1 - theta1_zero;
		//		//			ang2 = ang2 - theta2_zero;
		//		//			ang3 = ang3 - theta3_zero;
		//		//			ang4 = ang4 - theta4_zero;
		//		//			ang5 = ang5 - theta5_zero;
		//		//
		//		//
		//		//
		//		//
		//		//			///Because Assume opposite to real Arm.
		//		//			ang1 = -ang1;
		//		//			ang5 = -ang5;
		//		//
		//		//			///====Forword=====
		//		//			Homogenous_matrix_T10.at<double>(0, 0) = cos(ang1); Homogenous_matrix_T10.at<double>(0, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(0, 2) = sin(ang1); Homogenous_matrix_T10.at<double>(0, 3) = 0.0;
		//		//			Homogenous_matrix_T10.at<double>(1, 0) = sin(ang1); Homogenous_matrix_T10.at<double>(1, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(1, 2) = -cos(ang1); Homogenous_matrix_T10.at<double>(1, 3) = 0.0;
		//		//			Homogenous_matrix_T10.at<double>(2, 0) = 0.0;		Homogenous_matrix_T10.at<double>(2, 1) = 1.0;		 Homogenous_matrix_T10.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(2, 3) = d1;
		//		//			Homogenous_matrix_T10.at<double>(3, 0) = 0.0;		Homogenous_matrix_T10.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(3, 3) = 1.0;
		//		//
		//		//			Homogenous_matrix_T21.at<double>(0, 0) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 1) = -sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(0, 3) = a2*cos(ang2 + pi / 2);
		//		//			Homogenous_matrix_T21.at<double>(1, 0) = sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 1) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(1, 3) = a2*sin(ang2 + pi / 2);
		//		//			Homogenous_matrix_T21.at<double>(2, 0) = 0.0;		Homogenous_matrix_T21.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T21.at<double>(2, 3) = 0.0;
		//		//			Homogenous_matrix_T21.at<double>(3, 0) = 0.0;		Homogenous_matrix_T21.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T21.at<double>(3, 3) = 1.0;
		//		//
		//		//			Homogenous_matrix_T32.at<double>(0, 0) = cos(ang3); Homogenous_matrix_T32.at<double>(0, 1) = -sin(ang3); Homogenous_matrix_T32.at<double>(0, 2) = 0.0;       Homogenous_matrix_T32.at<double>(0, 3) = a3*cos(ang3);
		//		//			Homogenous_matrix_T32.at<double>(1, 0) = sin(ang3); Homogenous_matrix_T32.at<double>(1, 1) = cos(ang3); Homogenous_matrix_T32.at<double>(1, 2) = 0.0;       Homogenous_matrix_T32.at<double>(1, 3) = a3*sin(ang3);
		//		//			Homogenous_matrix_T32.at<double>(2, 0) = 0.0;		Homogenous_matrix_T32.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T32.at<double>(2, 3) = 0.0;
		//		//			Homogenous_matrix_T32.at<double>(3, 0) = 0.0;		Homogenous_matrix_T32.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T32.at<double>(3, 3) = 1.0;
		//		//			///Origin
		//		//			Homogenous_matrix_T43.at<double>(0, 0) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 1) = 0.0;       Homogenous_matrix_T43.at<double>(0, 2) = -sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 3) = 0.0;
		//		//			Homogenous_matrix_T43.at<double>(1, 0) = sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 1) = 0.0;       Homogenous_matrix_T43.at<double>(1, 2) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 3) = 0.0;
		//		//			Homogenous_matrix_T43.at<double>(2, 0) = 0.0;		Homogenous_matrix_T43.at<double>(2, 1) = -1.0;		 Homogenous_matrix_T43.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(2, 3) = 0.0;
		//		//			Homogenous_matrix_T43.at<double>(3, 0) = 0.0;		Homogenous_matrix_T43.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T43.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(3, 3) = 1.0;
		//		//
		//		//			Homogenous_matrix_T54.at<double>(0, 0) = cos(ang5); Homogenous_matrix_T54.at<double>(0, 1) = -sin(ang5); Homogenous_matrix_T54.at<double>(0, 2) = 0.0;       Homogenous_matrix_T54.at<double>(0, 3) = 0.0;
		//		//			Homogenous_matrix_T54.at<double>(1, 0) = sin(ang5); Homogenous_matrix_T54.at<double>(1, 1) = cos(ang5); Homogenous_matrix_T54.at<double>(1, 2) = 0.0;       Homogenous_matrix_T54.at<double>(1, 3) = 0.0;
		//		//			Homogenous_matrix_T54.at<double>(2, 0) = 0.0;		Homogenous_matrix_T54.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T54.at<double>(2, 3) = d5;
		//		//			Homogenous_matrix_T54.at<double>(3, 0) = 0.0;		Homogenous_matrix_T54.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T54.at<double>(3, 3) = 1.0;
		//		//			///Origin
		//		//			//逆向需要變ang5 - pi / 2
		//		//
		//		//			T10 = Homogenous_matrix_T10;
		//		//			T20 = T10*Homogenous_matrix_T21;
		//		//			T30 = T20*Homogenous_matrix_T32;
		//		//			T40 = T30*Homogenous_matrix_T43;
		//		//			T50 = T40*Homogenous_matrix_T54;
		//		//
		//		//			r11 = T50.at<double>(0, 0);
		//		//			r12 = T50.at<double>(0, 1);
		//		//			r13 = T50.at<double>(0, 2);
		//		//			r21 = T50.at<double>(1, 0);
		//		//			r22 = T50.at<double>(1, 1);
		//		//			r23 = T50.at<double>(1, 2);
		//		//			r31 = T50.at<double>(2, 0);
		//		//			r32 = T50.at<double>(2, 1);
		//		//			r33 = T50.at<double>(2, 2);
		//		//#pragma region Jacobian
		//		//			cv::Mat r0e = cv::Mat(3, 1, CV_64FC1);
		//		//			//Mat r1e = Mat(3, 1, CV_64FC1);
		//		//			cv::Mat r2e = cv::Mat(3, 1, CV_64FC1);
		//		//			cv::Mat r3e = cv::Mat(3, 1, CV_64FC1);
		//		//			cv::Mat r4e = cv::Mat(3, 1, CV_64FC1);
		//		//
		//		//			r0e.at<double>(0, 0) = T50.at<double>(0, 3);//夾爪位移量
		//		//			r0e.at<double>(1, 0) = T50.at<double>(1, 3);
		//		//			r0e.at<double>(2, 0) = T50.at<double>(2, 3);
		//		//			//r1e.at<double>(0, 0) = T50.at<double>(0, 3) - T10.at<double>(0, 3);
		//		//			//r1e.at<double>(1, 0) = T50.at<double>(1, 3) - T10.at<double>(1, 3);
		//		//			//r1e.at<double>(2, 0) = T50.at<double>(2, 3) - T10.at<double>(2, 3);
		//		//			r2e.at<double>(0, 0) = T50.at<double>(0, 3) - T20.at<double>(0, 3);//第二軸位移量
		//		//			r2e.at<double>(1, 0) = T50.at<double>(1, 3) - T20.at<double>(1, 3);
		//		//			r2e.at<double>(2, 0) = T50.at<double>(2, 3) - T20.at<double>(2, 3);
		//		//			r3e.at<double>(0, 0) = T50.at<double>(0, 3) - T30.at<double>(0, 3);
		//		//			r3e.at<double>(1, 0) = T50.at<double>(1, 3) - T30.at<double>(1, 3);
		//		//			r3e.at<double>(2, 0) = T50.at<double>(2, 3) - T30.at<double>(2, 3);
		//		//			r4e.at<double>(0, 0) = T50.at<double>(0, 3) - T40.at<double>(0, 3);
		//		//			r4e.at<double>(1, 0) = T50.at<double>(1, 3) - T40.at<double>(1, 3);
		//		//			r4e.at<double>(2, 0) = T50.at<double>(2, 3) - T40.at<double>(2, 3);
		//		//
		//		//			////////////////////////////////////////////////////////////看不懂看不懂看不懂看不懂看不懂看不懂看不懂
		//		//			cv::Mat b = cv::Mat(3, 1, CV_64FC1);
		//		//			cv::Mat b0 = cv::Mat(3, 1, CV_64FC1);
		//		//			//Mat b1 = Mat(3, 1, CV_64FC1);
		//		//			cv::Mat b2 = cv::Mat(3, 1, CV_64FC1);
		//		//			cv::Mat b3 = cv::Mat(3, 1, CV_64FC1);
		//		//			cv::Mat b4 = cv::Mat(3, 1, CV_64FC1);
		//		//
		//		//			b.at<double>(0, 0) = 0;
		//		//			b.at<double>(1, 0) = 0;
		//		//			b.at<double>(2, 0) = 1;
		//		//			b0.at<double>(0, 0) = 0;
		//		//			b0.at<double>(1, 0) = 0;
		//		//			b0.at<double>(2, 0) = 1;
		//		//			//b1 = T10(Range(0, 3), Range(0, 3))*b;
		//		//			b2 = T20(cv::Range(0, 3), cv::Range(0, 3))*b;
		//		//			b3 = T30(cv::Range(0, 3), cv::Range(0, 3))*b;
		//		//			b4 = T40(cv::Range(0, 3), cv::Range(0, 3))*b;
		//		//
		//		//			cv::Mat J1, /*J2,*/ J3, J4, J5;
		//		//			J1 = b0.cross(r0e);
		//		//			//J2 = b1.cross(r1e);
		//		//			J3 = b2.cross(r2e);
		//		//			J4 = b3.cross(r3e);
		//		//			J5 = b4.cross(r4e);
		//		//
		//		//
		//		//			cv::Mat Jacobian = cv::Mat(4, 4, CV_64FC1);
		//		//
		//		//
		//		// //			Jacobian.at<double>(0, 0) = J1.at<double>(0, 0);	Jacobian.at<double>(0, 1) = J3.at<double>(0, 0);  Jacobian.at<double>(0, 2) = J4.at<double>(0, 0);	Jacobian.at<double>(0, 3) = J5.at<double>(0, 0);
		//		//			Jacobian.at<double>(0, 0) = J1.at<double>(1, 0);	Jacobian.at<double>(0, 1) = J3.at<double>(1, 0);  Jacobian.at<double>(0, 2) = J4.at<double>(1, 0);	Jacobian.at<double>(0, 3) = J5.at<double>(1, 0);
		//		//			Jacobian.at<double>(1, 0) = J1.at<double>(2, 0);	Jacobian.at<double>(1, 1) = J3.at<double>(2, 0);  Jacobian.at<double>(1, 2) = J4.at<double>(2, 0);	Jacobian.at<double>(1, 3) = J5.at<double>(2, 0);
		//		//			Jacobian.at<double>(2, 0) = b0.at<double>(0, 0);	Jacobian.at<double>(2, 1) = b2.at<double>(0, 0);  Jacobian.at<double>(2, 2) = b3.at<double>(0, 0);	Jacobian.at<double>(2, 3) = b4.at<double>(0, 0);
		//		//			Jacobian.at<double>(3, 0) = b0.at<double>(1, 0);	Jacobian.at<double>(3, 1) = b2.at<double>(1, 0);  Jacobian.at<double>(3, 2) = b3.at<double>(1, 0);	Jacobian.at<double>(3, 3) = b4.at<double>(1, 0);
		//		//
		//		//
		//		//
		//		//			//Jacobian = [J11 J12 J13 J14 J15 J16;
		//		//			//			J21 J22 J23 J24 J25 J26;
		//		//			//			J31 J32 J33 J34 J35 J36;
		//		//			//			J41 J42 J43 J44 J45 J46;
		//		//			//			J51 J52 J53 J54 J55 J56;]
		//		//#pragma endregion Jacobian
		//		//
		//		//			// 算gamma, beta, alpha
		//		//			beta = atan2(sqrt(r31*r31 + r32*r32), r33);
		//		//			if (beta == 0.0)
		//		//			{
		//		//				alpha = 0;
		//		//				gamma = atan2(-r12, r11) * 180 / 3.1416;
		//		//			}
		//		//			else if (abs(180.0 - beta * 180 / 3.1416) < 1 && abs(180.0 - beta * 180 / 3.1416) > 0.0)
		//		//			{
		//		//				alpha = 0;
		//		//				gamma = atan2(r12, -r11) * 180 / 3.1416;
		//		//			}
		//		//			else
		//		//			{
		//		//				alpha = (atan2(r23 / sin(beta), r13 / sin(beta))) * 180 / 3.1416;
		//		//				gamma = (atan2(r32 / sin(beta), -r31 / sin(beta))) * 180 / 3.1416;
		//		//			}
		//		//			beta = beta * 180 / 3.1416;
		//		//			//if (gamma > 0)
		//		//			//{
		//		//
		//		//			//	beta = -beta;
		//		//			//}
		//		//			tool_end = T50*tool;
		//		//			end_x = tool_end.at<double>(0, 0);
		//		//			end_y = tool_end.at<double>(1, 0);
		//		//			end_z = tool_end.at<double>(2, 0);
		//		//
		//		//			cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
		//		//			printf("\n");
		//		//
		//		//			double aaa = 0.0;
		//		//			aaa = cos(60);
		//		//			aaa = cos(60 * 3.1416 / 180);
		//		//			cv::Mat Jacob(3, 3, CV_64FC1);
		//		//			Jacob.at<double>(0, 0) = cos(alpha* 3.1416 / 180)*sin(beta* 3.1416 / 180);	Jacob.at<double>(0, 1) = -sin(alpha* 3.1416 / 180);	Jacob.at<double>(0, 2) = 0.0;
		//		//			Jacob.at<double>(1, 0) = sin(alpha* 3.1416 / 180)*sin(beta* 3.1416 / 180);	Jacob.at<double>(1, 1) = cos(alpha* 3.1416 / 180);	Jacob.at<double>(1, 2) = 0.0;
		//		//			Jacob.at<double>(2, 0) = cos(beta* 3.1416 / 180);							Jacob.at<double>(2, 1) = 0.0;						Jacob.at<double>(2, 2) = 1.0;
		//		//
		//		//
		//		//			//============================
		//		//
		//		//			cv::Mat Q_Jacob(3, 3, CV_64FC1);
		//		//			Q_Jacob.at<double>(0, 0) = 0.0;		Q_Jacob.at<double>(0, 1) = -end_z;	Q_Jacob.at<double>(0, 2) = end_y;
		//		//			Q_Jacob.at<double>(1, 0) = end_z;	Q_Jacob.at<double>(1, 1) = 0.0;		Q_Jacob.at<double>(1, 2) = -end_x;
		//		//			Q_Jacob.at<double>(2, 0) = -end_y;	Q_Jacob.at<double>(2, 1) = end_x;	Q_Jacob.at<double>(2, 2) = 0.0;
		//		//
		//		//			Q_Jacob = -Q_Jacob;//CROSS反過來
		//		//
		//		///////////////////////////////////////////////////////////////////////////////////
		//		//			//Visual Servo
		//		//			//兩個三維點，一個期望點，一個當前點(兩夾具中心)
		//		//
		//		//			exp_x = -22;
		//		//			exp_y = 0;
		//		//			exp_z = 10;
		//		//			exp_gamma = -15.5878;
		//		//			exp_beta = 139.992;
		//		//			exp_alpha = 180;
		//		//			#pragma region 位置x,y,z 
		//
		//		//			cv::Mat r(3, 1, CV_64FC1);//N10左眼所得到的三維點(當前)
		//		//				r.at<double>(0, 0) = end_x;
		//		//				r.at<double>(1, 0) = end_y;
		//		//				r.at<double>(2, 0) = end_z;
		//		//				cv::Mat rd(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
		//		//				rd.at<double>(0, 0) = exp_x;
		//		//				rd.at<double>(1, 0) = exp_y;
		//		//				rd.at<double>(2, 0) = exp_z;
		//		//				cv::Mat error(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
		//		//				error = r - rd;
		//		//				//double error_x, error_y, error_z;定在全域
		//		//				error_x = error.at<double>(0, 0);
		//		//				error_y = error.at<double>(1, 0);
		//		//				error_z = error.at<double>(2, 0);
		//		//				string write_error;
		//		//				std::stringstream xx;
		//		//				xx << error_x;
		//		//				std::stringstream yy;
		//		//				yy << error_y;
		//		//				std::stringstream zz;
		//		//				zz << error_z;
		//		//			#pragma endregion 位置x,y,z				
		//		//			#pragma region 角度wx,wy,wz 
		//		//				double neg = 1;//第五軸方向
		//		//				if (gamma < 0)
		//		//				{
		//		//					gamma = gamma+360;
		//		//					//neg = -neg;
		//		//				}
		//		//
		//		//				cv::Mat wr(3, 1, CV_64FC1);//N10左眼所得到的三維點(當前)
		//		//				wr.at<double>(0, 0) = gamma* 3.1416 / 180;
		//		//				wr.at<double>(1, 0) = beta* 3.1416 / 180;
		//		//				wr.at<double>(2, 0) = alpha* 3.1416 / 180;
		//		//				cv::Mat wrd(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
		//		//				wrd.at<double>(0, 0) = exp_gamma* 3.1416 / 180;
		//		//				wrd.at<double>(1, 0) = exp_beta* 3.1416 / 180;
		//		//				wrd.at<double>(2, 0) = exp_alpha* 3.1416 / 180;
		//		//				cv::Mat w_error(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
		//		//				w_error = wr - wrd; 
		//		//
		//		//				//if (abs(w_error.at<double>(0, 0)) < (2 * pi + (3.1416 / 180)) || abs(w_error.at<double>(0, 0)) > (2 * pi - (3.1416 / 180)))
		//		//				//{
		//		//				//	w_error.at<double>(0, 0) = 0.0;
		//		//				//}
		//		//				//double error_wx, error_wy, error_wz;定在全域
		//		//				error_ThetaX = w_error.at<double>(0, 0);
		//		//				error_ThetaY = w_error.at<double>(1, 0);
		//		//				error_ThetaZ = w_error.at<double>(2, 0);
		//		//				string write_theta_error;
		//		//				std::stringstream xxx;
		//		//				xxx << error_ThetaX;
		//		//				std::stringstream yyy;
		//		//				yyy << error_ThetaY;
		//		//				std::stringstream zzz;
		//		//				zzz << error_ThetaZ;
		//		//			
		//		//				write_error =  xx.str() + "   " + yy.str() + "    " + zz.str() + "    " + xxx.str() + "    " + yyy.str() + "    " + zzz.str();
		//		//				
		//		//				Error <<count<<":    "<< write_error << endl;
		//		//			
		//		//			
		//		//				CurrentABC << count << ":    " << end_x << "  " << end_y << "  " << end_z << "  " << gamma* 3.1416 / 180 << "  " << beta* 3.1416 / 180 << "  " << alpha* 3.1416 / 180 << endl;
		//		//			#pragma endregion 角度wx,wy,wz	
		//		////////////////////////////////////////////////////////////////////////////////////////////
		//		//				cv::Mat vel_org(3, 1, CV_64FC1);//速度命令
		//		//				cv::Mat velw_org(3, 1, CV_64FC1);//角速度命令	
		//		//			//Mat QQ(3, 1, CV_64FC1);
		//		//			//QQ = Q_Jacob*(wr - wrd);
		//		//			//.at<double>(0, 0) = 0.0;
		//		//
		//		//
		//		//			//vel = -0.3 * ((r - rd) - Q_Jacob*(wr - wrd));
		//		//			vel_org = -lamda_v * ((r - rd) - Q_Jacob*(wr - wrd));//			vel = -lamda_v * ((r - rd) - Q_Jacob*(wr - wrd));
		//		//			velw_org = -lamda_w * (0 + Jacob * (wr - wrd));//			velw = -lamda_w * (0 + Jacob * (wr - wrd));
		//		//
		//		//			cv::Mat vel(2, 1, CV_64FC1);//速度命令
		//		//			cv::Mat velw(2, 1, CV_64FC1);//角速度命令	
		//		//
		//		//
		//		//			vel.at<double>(0, 0) = vel_org.at<double>(0, 0);
		//		//			vel.at<double>(1, 0) = vel_org.at<double>(2, 0);
		//		//			velw.at<double>(0, 0) = velw_org.at<double>(0, 0);
		//		//			velw.at<double>(1, 0) = velw_org.at<double>(1, 0);
		//		//
		//		//
		//		//			cv::Mat Error_dot = cv::Mat(4, 1, CV_64FC1);//加角度是6
		//		//
		//		//
		//		//			double vx, vz, wx, wy;
		//		//			////原本
		//		//			Error_dot.at<double>(0, 0) = vel.at<double>(0, 0);
		//		//			Error_dot.at<double>(1, 0) = vel.at<double>(1, 0);
		//		//			Error_dot.at<double>(2, 0) = velw.at<double>(0, 0);
		//		//			Error_dot.at<double>(3, 0) = velw.at<double>(1, 0);
		//		//
		//		//			//限制各軸速度極限
		//		//			vx = Error_dot.at<double>(0, 0);
		//		//			vz = Error_dot.at<double>(1, 0);
		//		//			wx = Error_dot.at<double>(2, 0);
		//		//			wy = Error_dot.at<double>(3, 0);
		//		//
		//		//
		//		//			string comands;
		//		//			std::stringstream vxx;
		//		//			vxx << vx;
		//		//			std::stringstream vzz;
		//		//			vzz << vz;
		//		//			std::stringstream wxx;
		//		//			wxx << wx;
		//		//			std::stringstream wyy;
		//		//			wyy << wy;
		//		//			comands = vxx.str() + "   " + vzz.str() + "   " + wxx.str() + "   " + wyy.str() ;
		//		//			Command << count << ":    " << comands << endl;
		//		//
		//		//
		//		//			cv::Mat Theta_dot;
		//		//			cv::Mat JacobianInv;
		//		//			//6*5
		//		//			//invert(Jacobian, JacobianInv , DECOMP_SVD);
		//		//			//Theta_dot = JacobianInv * Error_dot;
		//		//
		//		//			////5*5
		//		//			//Mat JacobianNew = Jacobian(Range(0, 5), Range(0, 5));
		//		//			//Mat Error_dot5 = Error_dot(Range(0, 5), Range(0, 1));
		//		//			//JacobianInv = JacobianNew.inv(DECOMP_SVD);
		//		//			//Theta_dot = JacobianInv * Error_dot5;
		//		//
		//		//
		//		//			//4*4
		//		//			JacobianInv = Jacobian.inv(cv::DECOMP_SVD);
		//		//			Theta_dot = JacobianInv * Error_dot;
		//		//
		//		//
		//		//			if (Theta_dot.at<double>(0, 0) >  1)
		//		//				Theta_dot.at<double>(0, 0) =  1;
		//		//			if (Theta_dot.at<double>(0, 0) < -1)
		//		//				Theta_dot.at<double>(0, 0) = -1;
		//		//			if (Theta_dot.at<double>(1, 0) >  1)
		//		//				Theta_dot.at<double>(1, 0) =  1;
		//		//			if (Theta_dot.at<double>(1, 0) < -1)
		//		//				Theta_dot.at<double>(1, 0) = -1;
		//		//			if (Theta_dot.at<double>(2, 0) >  1)
		//		//				Theta_dot.at<double>(2, 0) =  1;
		//		//			if (Theta_dot.at<double>(2, 0) < -1)
		//		//				Theta_dot.at<double>(2, 0) = -1;
		//		//			if (Theta_dot.at<double>(3, 0) >  1)
		//		//				Theta_dot.at<double>(3, 0) =  1;
		//		//			if (Theta_dot.at<double>(3, 0) < -1)
		//		//				Theta_dot.at<double>(3, 0) = -1;
		//		//
		//		//
		//		//			//Theta_dot.at<double>(0, 0) = -Theta_dot.at<double>(0, 0);		 //J2
		//		//			//Theta_dot.at<double>(1, 0) = -Theta_dot.at<double>(1, 0);		 //J3
		//		//			//Theta_dot.at<double>(2, 0) = -Theta_dot.at<double>(2, 0);		 //J4
		//		//			Theta_dot.at<double>(3, 0) = -Theta_dot.at<double>(3, 0)*neg;		 //J5
		//		//			JointVelocity << count << ":    " << Theta_dot.at<double>(0, 0) << "  " << Theta_dot.at<double>(1, 0) << "  " << Theta_dot.at<double>(2, 0) << "  " << Theta_dot.at<double>(3, 0) << endl;
		//		//			//velocity
		//		//
		//		//			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(0, 0) * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(2).setData(desiredJointVelocity);//desiredJointVelocity
		//		//			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(1, 0) * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(3).setData(desiredJointVelocity);
		//		//			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(2, 0) * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(4).setData(desiredJointVelocity);
		//		//			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(3, 0) * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(5).setData(desiredJointVelocity);
		//		//
		//		//
		//		//			count++;
		//		//
		//		//
		//		//		} while (abs(error_x) > 0.3 || abs(error_z) > 0.3 || abs(error_ThetaX) > 0.1 || abs(error_ThetaY) > 0.1 );//abs(error_x) < 1.0
		//		//		if (abs(error_x) < 0.3 &&  abs(error_z) < 0.3 && abs(error_ThetaX) < 0.1 && abs(error_ThetaY) < 0.1 )
		//		//		{
		//		//			//velocity
		//		//			desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(2).setData(desiredJointVelocity);
		//		//			desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(3).setData(desiredJointVelocity);
		//		//			desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(4).setData(desiredJointVelocity);
		//		//			desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
		//		//			myYouBotManipulator->getArmJoint(5).setData(desiredJointVelocity);
		//		//			cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
		//		//			printf("\n");
		//		//			//夾取
		//		//			myYouBotManipulator->getArmGripper().close();
		//		//			SLEEP_MILLISEC(4000);//setpointBar1.setpointBar2.barEncoder = 500;
		//		//			desiredJointAngle.angle = init2 * radian;
		//		//			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
		//		//			//SLEEP_MILLISEC(1000);
		//		//			desiredJointAngle.angle = init3 * radian;
		//		//			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
		//		//			//SLEEP_MILLISEC(1000);
		//		//			desiredJointAngle.angle = init4 * radian;
		//		//			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
		//		//			//SLEEP_MILLISEC(1000);
		//		//			desiredJointAngle.angle = init5 * radian;
		//		//			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);
		//		//			SLEEP_MILLISEC(10000);
		//		//			myYouBotManipulator->getArmGripper().open();
		//		//			SLEEP_MILLISEC(3000);
		//		//
		//		//
		//		//		}
		//		
		//		
		//		#pragma endregion 4軸速度控制
		//
		//		double seperate;
		//
		//
#pragma region 速度控制

		do
		{


			theta1 = theta2 = theta3 = theta4 = theta5 = theta234 = 0.0;





			//cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
			printf("\n");

			sang1 = sang2 = sang3 = sang4 = sang5 = 0.0;
			//回傳算當前角度


			myYouBotManipulator->getJointData(EachJointAngle1);
			sang1 = EachJointAngle1[0].angle.value();
			//sang1 = 0.0;
			sang2 = EachJointAngle1[1].angle.value();
			sang3 = EachJointAngle1[2].angle.value();
			sang4 = EachJointAngle1[3].angle.value();
			sang5 = EachJointAngle1[4].angle.value();
			CurrentAngle << count << ":    " << sang1 << "  " << sang2 << "  " << sang3 << "  " << sang4 << "  " << sang5 << endl;

			ang1 = sang1;
			ang2 = sang2;
			ang3 = sang3;
			ang4 = sang4;
			ang5 = sang5;

			//if (count == 0)
			//{
			ang1 = ang1 - theta1_zero;
			ang2 = ang2 - theta2_zero;
			ang3 = ang3 - theta3_zero;
			ang4 = ang4 - theta4_zero;
			ang5 = ang5 - theta5_zero;




			///Because Assume opposite to real Arm.
			ang1 = -ang1;
			//    ang2 = -ang2;
			//    ang3 = -ang3;
			//    ang4 = -ang4;
			ang5 = -ang5;
			//}




			///====Forword=====
			Homogenous_matrix_T10.at<double>(0, 0) = cos(ang1); Homogenous_matrix_T10.at<double>(0, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(0, 2) = sin(ang1); Homogenous_matrix_T10.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T10.at<double>(1, 0) = sin(ang1); Homogenous_matrix_T10.at<double>(1, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(1, 2) = -cos(ang1); Homogenous_matrix_T10.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T10.at<double>(2, 0) = 0.0;		Homogenous_matrix_T10.at<double>(2, 1) = 1.0;		 Homogenous_matrix_T10.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(2, 3) = d1;
			Homogenous_matrix_T10.at<double>(3, 0) = 0.0;		Homogenous_matrix_T10.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T21.at<double>(0, 0) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 1) = -sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(0, 3) = a2*cos(ang2 + pi / 2);
			Homogenous_matrix_T21.at<double>(1, 0) = sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 1) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(1, 3) = a2*sin(ang2 + pi / 2);
			Homogenous_matrix_T21.at<double>(2, 0) = 0.0;		Homogenous_matrix_T21.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T21.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T21.at<double>(3, 0) = 0.0;		Homogenous_matrix_T21.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T21.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T32.at<double>(0, 0) = cos(ang3); Homogenous_matrix_T32.at<double>(0, 1) = -sin(ang3); Homogenous_matrix_T32.at<double>(0, 2) = 0.0;       Homogenous_matrix_T32.at<double>(0, 3) = a3*cos(ang3);
			Homogenous_matrix_T32.at<double>(1, 0) = sin(ang3); Homogenous_matrix_T32.at<double>(1, 1) = cos(ang3); Homogenous_matrix_T32.at<double>(1, 2) = 0.0;       Homogenous_matrix_T32.at<double>(1, 3) = a3*sin(ang3);
			Homogenous_matrix_T32.at<double>(2, 0) = 0.0;		Homogenous_matrix_T32.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T32.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T32.at<double>(3, 0) = 0.0;		Homogenous_matrix_T32.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T32.at<double>(3, 3) = 1.0;
			///Origin
			Homogenous_matrix_T43.at<double>(0, 0) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 1) = 0.0;       Homogenous_matrix_T43.at<double>(0, 2) = -sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(1, 0) = sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 1) = 0.0;       Homogenous_matrix_T43.at<double>(1, 2) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(2, 0) = 0.0;		Homogenous_matrix_T43.at<double>(2, 1) = -1.0;		 Homogenous_matrix_T43.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(2, 3) = 0.0;
			Homogenous_matrix_T43.at<double>(3, 0) = 0.0;		Homogenous_matrix_T43.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T43.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(3, 3) = 1.0;

			Homogenous_matrix_T54.at<double>(0, 0) = cos(ang5); Homogenous_matrix_T54.at<double>(0, 1) = -sin(ang5); Homogenous_matrix_T54.at<double>(0, 2) = 0.0;       Homogenous_matrix_T54.at<double>(0, 3) = 0.0;
			Homogenous_matrix_T54.at<double>(1, 0) = sin(ang5); Homogenous_matrix_T54.at<double>(1, 1) = cos(ang5); Homogenous_matrix_T54.at<double>(1, 2) = 0.0;       Homogenous_matrix_T54.at<double>(1, 3) = 0.0;
			Homogenous_matrix_T54.at<double>(2, 0) = 0.0;		Homogenous_matrix_T54.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T54.at<double>(2, 3) = d5;
			Homogenous_matrix_T54.at<double>(3, 0) = 0.0;		Homogenous_matrix_T54.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T54.at<double>(3, 3) = 1.0;
			///Origin
			//逆向需要變ang5 - pi / 2

			T10 = Homogenous_matrix_T10;
			T20 = T10*Homogenous_matrix_T21;
			T30 = T20*Homogenous_matrix_T32;
			T40 = T30*Homogenous_matrix_T43;
			T50 = T40*Homogenous_matrix_T54;

			r11 = T50.at<double>(0, 0);
			r12 = T50.at<double>(0, 1);

			r13 = T50.at<double>(0, 2);
			r21 = T50.at<double>(1, 0);
			r22 = T50.at<double>(1, 1);
			r23 = T50.at<double>(1, 2);
			r31 = T50.at<double>(2, 0);
			r32 = T50.at<double>(2, 1);
			r33 = T50.at<double>(2, 2);


#pragma region Jacobian
			Mat r0e = Mat(3, 1, CV_64FC1);
			Mat r1e = Mat(3, 1, CV_64FC1);
			Mat r2e = Mat(3, 1, CV_64FC1);
			Mat r3e = Mat(3, 1, CV_64FC1);
			Mat r4e = Mat(3, 1, CV_64FC1);

			r0e.at<double>(0, 0) = T50.at<double>(0, 3);
			r0e.at<double>(1, 0) = T50.at<double>(1, 3);
			r0e.at<double>(2, 0) = T50.at<double>(2, 3);
			r1e.at<double>(0, 0) = T50.at<double>(0, 3) - T10.at<double>(0, 3);
			r1e.at<double>(1, 0) = T50.at<double>(1, 3) - T10.at<double>(1, 3);
			r1e.at<double>(2, 0) = T50.at<double>(2, 3) - T10.at<double>(2, 3);
			r2e.at<double>(0, 0) = T50.at<double>(0, 3) - T20.at<double>(0, 3);
			r2e.at<double>(1, 0) = T50.at<double>(1, 3) - T20.at<double>(1, 3);
			r2e.at<double>(2, 0) = T50.at<double>(2, 3) - T20.at<double>(2, 3);
			r3e.at<double>(0, 0) = T50.at<double>(0, 3) - T30.at<double>(0, 3);
			r3e.at<double>(1, 0) = T50.at<double>(1, 3) - T30.at<double>(1, 3);
			r3e.at<double>(2, 0) = T50.at<double>(2, 3) - T30.at<double>(2, 3);
			r4e.at<double>(0, 0) = T50.at<double>(0, 3) - T40.at<double>(0, 3);
			r4e.at<double>(1, 0) = T50.at<double>(1, 3) - T40.at<double>(1, 3);
			r4e.at<double>(2, 0) = T50.at<double>(2, 3) - T40.at<double>(2, 3);

			Mat b = Mat(3, 1, CV_64FC1);
			Mat b0 = Mat(3, 1, CV_64FC1);
			Mat b1 = Mat(3, 1, CV_64FC1);
			Mat b2 = Mat(3, 1, CV_64FC1);
			Mat b3 = Mat(3, 1, CV_64FC1);
			Mat b4 = Mat(3, 1, CV_64FC1);



			b.at<double>(0, 0) = 0;
			b.at<double>(1, 0) = 0;
			b.at<double>(2, 0) = 1;
			b0.at<double>(0, 0) = 0;
			b0.at<double>(1, 0) = 0;
			b0.at<double>(2, 0) = 1;
			b1 = T10(Range(0, 3), Range(0, 3))*b;
			b2 = T20(Range(0, 3), Range(0, 3))*b;
			b3 = T30(Range(0, 3), Range(0, 3))*b;
			b4 = T40(Range(0, 3), Range(0, 3))*b;

			Mat J1, J2, J3, J4, J5;
			J1 = b0.cross(r0e);
			J2 = b1.cross(r1e);
			J3 = b2.cross(r2e);
			J4 = b3.cross(r3e);
			J5 = b4.cross(r4e);


			Mat Jacobian = Mat(6, 5, CV_64FC1);


			Jacobian.at<double>(0, 0) = J1.at<double>(0, 0);	Jacobian.at<double>(0, 1) = J2.at<double>(0, 0);	Jacobian.at<double>(0, 2) = J3.at<double>(0, 0);
			Jacobian.at<double>(1, 0) = J1.at<double>(1, 0);	Jacobian.at<double>(1, 1) = J2.at<double>(1, 0);	Jacobian.at<double>(1, 2) = J3.at<double>(1, 0);
			Jacobian.at<double>(2, 0) = J1.at<double>(2, 0);	Jacobian.at<double>(2, 1) = J2.at<double>(2, 0);	Jacobian.at<double>(2, 2) = J3.at<double>(2, 0);
			Jacobian.at<double>(3, 0) = b0.at<double>(0, 0);	Jacobian.at<double>(3, 1) = b1.at<double>(0, 0);	Jacobian.at<double>(3, 2) = b2.at<double>(0, 0);
			Jacobian.at<double>(4, 0) = b0.at<double>(1, 0);	Jacobian.at<double>(4, 1) = b1.at<double>(1, 0);	Jacobian.at<double>(4, 2) = b2.at<double>(1, 0);
			Jacobian.at<double>(5, 0) = b0.at<double>(2, 0);	Jacobian.at<double>(5, 1) = b1.at<double>(2, 0);	Jacobian.at<double>(5, 2) = b2.at<double>(2, 0);

			Jacobian.at<double>(0, 3) = J4.at<double>(0, 0);	Jacobian.at<double>(0, 4) = J5.at<double>(0, 0);
			Jacobian.at<double>(1, 3) = J4.at<double>(1, 0);	Jacobian.at<double>(1, 4) = J5.at<double>(1, 0);
			Jacobian.at<double>(2, 3) = J4.at<double>(2, 0);	Jacobian.at<double>(2, 4) = J5.at<double>(2, 0);
			Jacobian.at<double>(3, 3) = b3.at<double>(0, 0);	Jacobian.at<double>(3, 4) = b4.at<double>(0, 0);
			Jacobian.at<double>(4, 3) = b3.at<double>(1, 0);	Jacobian.at<double>(4, 4) = b4.at<double>(1, 0);
			Jacobian.at<double>(5, 3) = b3.at<double>(2, 0);	Jacobian.at<double>(5, 4) = b4.at<double>(2, 0);

			//Jacobian = [J11 J12 J13 J14 J15 J16;
			//			J21 J22 J23 J24 J25 J26;
			//			J31 J32 J33 J34 J35 J36;
			//			J41 J42 J43 J44 J45 J46;
			//			J51 J52 J53 J54 J55 J56;]
#pragma endregion Jacobian

			// 算gamma, beta, alpha
			if (r33 < 1)
			{
				if (r33 > -1)
				{
					beta = acos(r33);
					alpha = atan2(r23, r13);
					gamma = atan2(r32, -r31);
				}
				else
				{
					beta = pi;
					alpha = -atan2(r21, r22);
					gamma = 0;
				}

			}
			else
			{
				beta = 0;
				alpha = -atan2(r21, r22);
				gamma = 0;
			}

			alpha = alpha*rad2deg;
			beta = beta*rad2deg;
			gamma = gamma*rad2deg;
			//if (gamma > 0)
			//{

			//	beta = -beta;
			//}
			tool_end = T50*tool;
			end_x = tool_end.at<double>(0, 0);//夾爪當前的x y z
			end_y = tool_end.at<double>(1, 0);
			end_z = tool_end.at<double>(2, 0);
			cout << "期望位置: " << exp_x << " " << exp_y << " " << exp_z << " " << exp_gamma << " " << exp_beta << " " << exp_alpha << "  " << " 正確的位置" << endl;
			cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << "  " << " 正確的位置" << endl;
			printf("\n");

			//=====Jacob==========
			//Mat Jacob(3, 3, CV_64FC1);
			//Jacob.at<double>(0, 0) = 0.0;	Jacob.at<double>(0, 1) = -sin(alpha);	Jacob.at<double>(0, 2) = cos(alpha)*sin(beta);
			//Jacob.at<double>(1, 0) = 0.0;	Jacob.at<double>(1, 1) = cos(alpha);	Jacob.at<double>(1, 2) = sin(alpha)*sin(beta);
			//Jacob.at<double>(2, 0) = 1.0;	Jacob.at<double>(2, 1) = 0.0;			Jacob.at<double>(2, 2) = cos(beta);
			//*3.1416/180

			double aaa = 0.0;//垃圾
			aaa = cos(60);
			aaa = cos(60 * 3.1416 / 180);
			Mat Jacob(3, 3, CV_64FC1);//旋轉速度
			Jacob.at<double>(0, 0) = cos(alpha* 3.1416 / 180)*sin(beta* 3.1416 / 180);	Jacob.at<double>(0, 1) = -sin(alpha* 3.1416 / 180);	Jacob.at<double>(0, 2) = 0.0;
			Jacob.at<double>(1, 0) = sin(alpha* 3.1416 / 180)*cos(beta* 3.1416 / 180);	Jacob.at<double>(1, 1) = cos(alpha* 3.1416 / 180);	Jacob.at<double>(1, 2) = 0.0;
			Jacob.at<double>(2, 0) = cos(beta* 3.1416 / 180);							Jacob.at<double>(2, 1) = 0.0;						Jacob.at<double>(2, 2) = 1.0;


			//============================

			Mat Q_Jacob(3, 3, CV_64FC1);
			Q_Jacob.at<double>(0, 0) = 0.0;		Q_Jacob.at<double>(0, 1) = -end_z;	Q_Jacob.at<double>(0, 2) = end_y;
			Q_Jacob.at<double>(1, 0) = end_z;	Q_Jacob.at<double>(1, 1) = 0.0;		Q_Jacob.at<double>(1, 2) = -end_x;
			Q_Jacob.at<double>(2, 0) = -end_y;	Q_Jacob.at<double>(2, 1) = end_x;	Q_Jacob.at<double>(2, 2) = 0.0;

			Q_Jacob = -Q_Jacob;//CROSS反過來


			//Visual Servo
			//兩個三維點，一個期望點，一個當前點(兩夾具中心)











#pragma region 位置x,y,z 
			Mat r(3, 1, CV_64FC1);//N10左眼所得到的三維點(當前)
			r.at<double>(0, 0) = end_x;
			r.at<double>(1, 0) = end_y;
			r.at<double>(2, 0) = end_z;
			Mat rd(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
			//exp_x = test[i].x;//讀取test vector的值 i 會更新  這樣子就會有新的點了
			//exp_y = test[i].y;
			//exp_z = test[i].z;
			rd.at<double>(0, 0) = exp_x;
			rd.at<double>(1, 0) = exp_y;
			rd.at<double>(2, 0) = exp_z;

			Mat error(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
			error = r - rd;
			//double error_x, error_y, error_z;定在全域
			error_x = error.at<double>(0, 0);
			error_y = error.at<double>(1, 0);
			error_z = error.at<double>(2, 0);
			string write_error;
			std::stringstream xx;
			xx << error_x;
			std::stringstream yy;
			yy << error_y;
			std::stringstream zz;
			zz << error_z;
			/*write_error = " error_x = " + xx.str() + " error_y = " + yy.str() + " error_z = " + zz.str() ;
			Error << write_error << endl;*/
			//write_error = ConvertToString(error_x);
			
#pragma endregion 位置x,y,z		
#pragma region 角度wx,wy,wz 
			Mat wr(3, 1, CV_64FC1);//N10左眼所得到的三維點(當前)
			wr.at<double>(0, 0) = gamma* 3.1416 / 180;
			wr.at<double>(1, 0) = beta* 3.1416 / 180;
			wr.at<double>(2, 0) = alpha* 3.1416 / 180;
			Mat wrd(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)//這裡故意不讓角度更新，因為選轉的話手臂會壞掉
			wrd.at<double>(0, 0) = exp_gamma* 3.1416 / 180;
			wrd.at<double>(1, 0) = exp_beta* 3.1416 / 180;
			wrd.at<double>(2, 0) = exp_alpha* 3.1416 / 180;
			Mat w_error(3, 1, CV_64FC1);//N10左眼所得到的三維點(期望)
			w_error = wr - wrd;
			//double error_wx, error_wy, error_wz;定在全域
			error_ThetaX = w_error.at<double>(0, 0);
			error_ThetaY = w_error.at<double>(1, 0);
			error_ThetaZ = w_error.at<double>(2, 0);
			string write_theta_error;
			std::stringstream xxx;
			xxx << error_ThetaX;
			std::stringstream yyy;
			yyy << error_ThetaY;
			std::stringstream zzz;
			zzz << error_ThetaZ;

			//write_error = xx.str() + "   " + yy.str() + "    " + zz.str() + "    " + xxx.str() + "    " + yyy.str() + "    " + zzz.str();

			/*Error << sqrt((test[ccc - 1].x - end_x)*(test[ccc - 1].x - end_x) + (test[ccc - 1].y - end_y)*(test[ccc - 1].y - end_y) + (test[ccc - 1].z - end_z)*(test[ccc - 1].z - end_z)) << endl;
			Errorx << test[ccc - 1].x - end_x << endl;
			Errory << test[ccc - 1].y - end_y << endl;
			Errorz << test[ccc - 1].z - end_z << endl;
			desiredx << test[ccc - 1].x << endl;
			desiredy << test[ccc - 1].y << endl;
			desiredz << test[ccc - 1].z << endl;*/
			nowx << end_x << endl;
			nowy << end_y << endl;
			nowz << end_z << endl;
			nowr << gamma << endl;
			nowb << beta << endl;
			nowa << alpha << endl;
			//write_error = ConvertToString(error_x);



			//if (abs(error_x) <= 2.5 && abs(error_y) <= 2.5)
			//{
			//	lamda_v = lamda_v + 0.1;
			//	lamda_w = lamda_w + 0.1;
			//	if (lamda_v >= 1.0)
			//	{
			//		lamda_v = 1.0;
			//	}
			//	if (lamda_w >= 1.0)
			//	{
			//		lamda_w = 1.0;
			//	}
			//}

#pragma endregion 角度wx,wy,wz	





			Mat vel(3, 1, CV_64FC1);////相對基底速度命令

			Mat velw(3, 1, CV_64FC1);////相對基底速度命令
			Mat QQ(3, 1, CV_64FC1);
			QQ = Q_Jacob*(wr - wrd);
			//.at<double>(0, 0) = 0.0;
			/////5.31

			
			//vel = -lamda_v * ((r - rd) - Q_Jacob*(wr - wrd));
			vel = -lamda_v * ((r - rd) + Q_Jacob*Jacob*(wr - wrd));
			//velw = -lamda_w * (0 + Jacob * (wr - wrd));
			velw = -lamda_w * (0 + Jacob * (wr - wrd));





			Mat Error_dot = Mat(6, 1, CV_64FC1);//加角度是6

			//Mat Error_dot_base = Mat(6, 1, CV_64FC1);//加角度是6
			//Error_dot.at<double>(0, 0) = vel.at<double>(0, 0);
			//Error_dot.at<double>(1, 0) = vel.at<double>(1, 0);
			//Error_dot.at<double>(2, 0) = vel.at<double>(2, 0);
			//Error_dot.at<double>(3, 0) = 0;//velw.at<double>(0, 0);
			//Error_dot.at<double>(4, 0) = 0;//velw.at<double>(1, 0);
			//Error_dot.at<double>(5, 0) = 0;//velw.at<double>(2, 0);


			/*Mat SK(3, 3, CV_64FC1);
			SK.at<double>(0, 0) = 0.0;		SK.at<double>(0, 1) = -end_z;	SK.at<double>(0, 2) = end_y;
			SK.at<double>(1, 0) = end_z;	SK.at<double>(1, 1) = 0.0;		SK.at<double>(1, 2) = -end_x;
			SK.at<double>(2, 0) = -end_y;	SK.at<double>(2, 1) = end_x;	SK.at<double>(2, 2) = 0.0;*/

			//Mat RT0 = T50(Range(0, 3), Range(0, 3));
			//Mat RT0(3, 3, CV_64FC1);
			//invert(R0T, RT0);

			Mat T(6, 6, CV_64FC1);
			//T = SK*RT0;

			//T(Range(0, 3), Range(0, 3)) = RT0;
			//Mat vel_b(3, 1, CV_64FC1);//相對基底速度命令  沒用到
			//Mat velw_b(3, 1, CV_64FC1);//相對基底速度命令	沒用到

			//vel_b = RT0 * Error_dot(Range(0, 3), Range(0, 1)) + SK*RT0*Error_dot(Range(3, 6), Range(0, 1));
			//velw_b = RT0 * Error_dot(Range(3, 6), Range(0, 1));

			double vx, vy, vz, wx, wy, wz;
			////原本
			Error_dot.at<double>(0, 0) = vel.at<double>(0, 0);
			Error_dot.at<double>(1, 0) = vel.at<double>(1, 0);
			Error_dot.at<double>(2, 0) = vel.at<double>(2, 0);
			Error_dot.at<double>(3, 0) = velw.at<double>(0, 0);
			Error_dot.at<double>(4, 0) = velw.at<double>(1, 0);
			Error_dot.at<double>(5, 0) = velw.at<double>(2, 0);

			//測試

			//Error_dot.at<double>(0, 0) = 0;//vel.at<double>(0, 0);
			//Error_dot.at<double>(1, 0) = 0;//vel.at<double>(1, 0);
			//Error_dot.at<double>(2, 0) = 0;//vel.at<double>(2, 0);
			//Error_dot.at<double>(3, 0) = 0;//velw.at<double>(0, 0);
			//Error_dot.at<double>(4, 0) = 0.05;//velw.at<double>(1, 0);
			//Error_dot.at<double>(5, 0) = 0;//velw.at<double>(2, 0);

			//vx = Error_dot.at<double>(0, 0);
			//vy = Error_dot.at<double>(1, 0);
			//vz = Error_dot.at<double>(2, 0);
			//wx = Error_dot.at<double>(3, 0);
			//wy = Error_dot.at<double>(4, 0);
			//wz = Error_dot.at<double>(5, 0);
			//Mat SK_i(3, 3, CV_64FC1);
			//invert(SK,SK);


			////測試轉到基底
			Error_dot(Range(0, 3), Range(0, 1)) = Error_dot(Range(3, 6), Range(0, 1));//-SK*Error_dot(Range(3, 6), Range(0, 1))
			vx = Error_dot.at<double>(0, 0);
			vy = Error_dot.at<double>(1, 0);
			vz = Error_dot.at<double>(2, 0);
			Error_dot(Range(3, 6), Range(0, 1)) = Error_dot(Range(3, 6), Range(0, 1));
			wx = Error_dot.at<double>(3, 0);
			wy = Error_dot.at<double>(4, 0);
			wz = Error_dot.at<double>(5, 0);


			Error_dot.at<double>(0, 0) = Error_dot.at<double>(0, 0);
			Error_dot.at<double>(1, 0) = Error_dot.at<double>(1, 0);
			Error_dot.at<double>(2, 0) = Error_dot.at<double>(2, 0);
			Error_dot.at<double>(3, 0) = Error_dot.at<double>(3, 0);
			Error_dot.at<double>(4, 0) = Error_dot.at<double>(4, 0);	//0.05;//Error_dot.at<double>(4, 0);	
			Error_dot.at<double>(5, 0) = Error_dot.at<double>(5, 0);



			//限制各軸速度極限
			vx = Error_dot.at<double>(0, 0);
			vy = Error_dot.at<double>(1, 0);
			vz = Error_dot.at<double>(2, 0);
			wx = Error_dot.at<double>(3, 0);
			wy = Error_dot.at<double>(4, 0);
			wz = Error_dot.at<double>(5, 0);
			
			if (vx > 2)
				Error_dot.at<double>(0, 0) = 2;
			if (vx < -2)
				Error_dot.at<double>(0, 0) = -2;
			if (vy > 2)
				Error_dot.at<double>(1, 0) = 2;
			if (vy < -2)
				Error_dot.at<double>(1, 0) = -2;
			if (vz > 2)
				Error_dot.at<double>(2, 0) = 2;
			if (vz < -2)
				Error_dot.at<double>(2, 0) = -2;
			if (wx > 2)
				Error_dot.at<double>(3, 0) = 2;
			if (wx < -2)
				Error_dot.at<double>(3, 0) = -2;
			if (wy > 2)
				Error_dot.at<double>(4, 0) = 2;
			if (wy < -2)
				Error_dot.at<double>(4, 0) = -2;
			if (wz > 2)
				Error_dot.at<double>(5, 0) = 2;
			if (wz < -2)
				Error_dot.at<double>(5, 0) = -2;

			nowvel << "vx: " << Error_dot.at<double>(0, 0) << "  vy: " << Error_dot.at<double>(1, 0) << "  vz: " << Error_dot.at<double>(2, 0) << "  wx: " << Error_dot.at<double>(3, 0) << "  wy: " << Error_dot.at<double>(4, 0) << "  wz: " << Error_dot.at<double>(5, 0) << endl;
			string comands;
			std::stringstream vxx;
			vxx << vx;
			std::stringstream vyy;
			vyy << vy;
			std::stringstream vzz;
			vzz << vz;
			std::stringstream wxx;
			wxx << wx;
			std::stringstream wyy;
			wyy << wy;
			std::stringstream wzz;
			wzz << wz;
			comands = vxx.str() + "   " + vyy.str() + "   " + vzz.str() + "   " + wxx.str() + "   " + wyy.str() + "   " + wzz.str();
			Command << count << ":    " << comands << endl;
			//% Pdot = J * Qdot
			//% Pdot = -lamda * error = error_dot
			//cur = [-20.4687 7.45002 7.73058 89.9996 165 160]';   
			//exp = [-22.2214 8.8794 9.47906 89.9996 160 160]';
			//error = cur - exp;

			Mat Theta_dot;
			Mat JacobianInv;
			//6*5
			//invert(Jacobian, JacobianInv , DECOMP_SVD);
			//Theta_dot = JacobianInv * Error_dot;

			//5*5
			Mat JacobianNew = Jacobian(Range(0, 5), Range(0, 5));
			Mat Error_dot5 = Error_dot(Range(0, 5), Range(0, 1));
			JacobianInv = JacobianNew.inv(DECOMP_SVD);
			Theta_dot = JacobianInv * Error_dot5;


			Theta_dot.at<double>(0, 0) = -Theta_dot.at<double>(0, 0);
			//Theta_dot.at<double>(1, 0) = -Theta_dot.at<double>(1, 0);
			//Theta_dot.at<double>(2, 0) = -Theta_dot.at<double>(2, 0);
			//Theta_dot.at<double>(3, 0) = -Theta_dot.at<double>(3, 0);
			Theta_dot.at<double>(4, 0) = -Theta_dot.at<double>(4, 0);




			//velocity
			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(0, 0) * radian_per_second;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointVelocity);
			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(1, 0) * radian_per_second;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointVelocity);//desiredJointVelocity
			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(2, 0) * radian_per_second;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointVelocity);
			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(3, 0) * radian_per_second;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointVelocity);
			desiredJointVelocity.angularVelocity = Theta_dot.at<double>(4, 0) * radian_per_second;
			myYouBotManipulator->getArmJoint(5).setData(desiredJointVelocity);

			//if (count == 1)看東西的垃圾
			//{
			//	int fyjg;
			//	fyjg = 1;
			//}
			count++;


		}



		while (abs(error_x) > 1 || abs(error_y) > 1 || abs(error_z) > 1 || abs(error_ThetaZ) > 0.12 || abs(error_ThetaY) > 0.12 || abs(error_ThetaX) > 0.12);//abs(error_x) < 1.0





		if (abs(error_x) < 1 && abs(error_y) < 1 && abs(error_z) < 1 && abs(error_ThetaZ) < 0.12 && abs(error_ThetaY) < 0.12 && abs(error_ThetaX) < 0.12)
		{
			//velocity


			if (i < ccc)
			{
				cout << i << endl;
				i++;

			}


			if (i == ccc)//當i讀取位置到 vector的最後一格時
			{

				cout << "抓到炸彈魔了" << endl;

				desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
				myYouBotManipulator->getArmJoint(1).setData(desiredJointVelocity);
				desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
				myYouBotManipulator->getArmJoint(2).setData(desiredJointVelocity);
				desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
				myYouBotManipulator->getArmJoint(3).setData(desiredJointVelocity);
				desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
				myYouBotManipulator->getArmJoint(4).setData(desiredJointVelocity);
				desiredJointVelocity.angularVelocity = 0.0 * radian_per_second;
				myYouBotManipulator->getArmJoint(5).setData(desiredJointVelocity);

				cout << "給期望點按1 拍照按2 夾爪放開按3 開車按7 新姿態9 夾爪10 " << endl;
				cin >> bomb;
			}

			while (bomb == 1 || bomb == 10 || bomb == 5 || bomb == 2 || bomb == 3 || bomb == 9 || bomb == 7)//while迴圈為了vector設計的
			{
				if (bomb == 10)
				{
					myYouBotManipulator->getArmGripper().close();
					SLEEP_MILLISEC(4000);//setpointBar1.setpointBar2.barEncoder = 500;
					cout << "給期望點按1 拍照按2 夾爪放開按3 開車按7 新姿態9 夾爪10 " << endl;
					cin >> bomb;
				}

#pragma region 新期望點
				if (bomb == 1)
				{
					i = 0;//把讀vector的位置放回一開始的0
					test.clear();//vector清除
					//cout << "exp_x:";

					//cin >> exp_x;

					//cout << "exp_y:";

					//cin >> exp_y;

					//cout << "exp_z:";
					//cin >> exp_z;

					//cout << "exp_gamma:";
					//cin >> exp_gamma;

					//cout << "exp_beta:";

					//cin >> exp_beta;
					//cout << "exp_alpha:";

					//cin >> exp_alpha;



					cout << "BG矩陣" << endl;
					cout << "bg11:";

					cin >> bg11;

					cout << "bg12:";

					cin >> bg12;

					cout << "bg13:";
					cin >> bg13;

					cout << "bg14:";
					cin >> bg14;

					cout << "bg21:";
					cin >> bg21;

					cout << "bg22:";

					cin >> bg22;

					cout << "bg23:";
					cin >> bg23;

					cout << "bg24:";
					cin >> bg24;

					cout << "bg31:";
					cin >> bg31;

					cout << "bg32:";

					cin >> bg32;

					cout << "bg33:";
					cin >> bg33;

					cout << "bg34:";
					cin >> bg34;

					cout << "bg41:";
					cin >> bg41;

					cout << "bg42:";

					cin >> bg42;

					cout << "bg43:";
					cin >> bg43;

					cout << "bg44:";
					cin >> bg44;

					//cout << "上升請按";
					//cin >> upg;


					Homogenous_matrix_TBG.at<double>(0, 0) = bg11;
					Homogenous_matrix_TBG.at<double>(0, 1) = bg12;
					Homogenous_matrix_TBG.at<double>(0, 2) = bg13;
					Homogenous_matrix_TBG.at<double>(0, 3) = bg14;
					Homogenous_matrix_TBG.at<double>(1, 0) = bg21;
					Homogenous_matrix_TBG.at<double>(1, 1) = bg22;
					Homogenous_matrix_TBG.at<double>(1, 2) = bg23;
					Homogenous_matrix_TBG.at<double>(1, 3) = bg24;
					Homogenous_matrix_TBG.at<double>(2, 0) = bg31;
					Homogenous_matrix_TBG.at<double>(2, 1) = bg32;
					Homogenous_matrix_TBG.at<double>(2, 2) = bg33;
					Homogenous_matrix_TBG.at<double>(2, 3) = bg34;
					Homogenous_matrix_TBG.at<double>(3, 0) = bg41;
					Homogenous_matrix_TBG.at<double>(3, 1) = bg42;
					Homogenous_matrix_TBG.at<double>(3, 2) = bg43;
					Homogenous_matrix_TBG.at<double>(3, 3) = bg44;

					Homogenous_matrix_TB0 = Homogenous_matrix_TBG*T50;

					b011 = Homogenous_matrix_TB0.at<double>(0, 0);
					b012 = Homogenous_matrix_TB0.at<double>(0, 1);
					b013 = Homogenous_matrix_TB0.at<double>(0, 2);
					b014 = Homogenous_matrix_TB0.at<double>(0, 3);
					b021 = Homogenous_matrix_TB0.at<double>(1, 0);
					b022 = Homogenous_matrix_TB0.at<double>(1, 1);
					b023 = Homogenous_matrix_TB0.at<double>(1, 2);
					b024 = Homogenous_matrix_TB0.at<double>(1, 3);
					b031 = Homogenous_matrix_TB0.at<double>(2, 0);
					b032 = Homogenous_matrix_TB0.at<double>(2, 1);
					b033 = Homogenous_matrix_TB0.at<double>(2, 2);
					b034 = Homogenous_matrix_TB0.at<double>(2, 3);
					b041 = Homogenous_matrix_TB0.at<double>(3, 0);
					b042 = Homogenous_matrix_TB0.at<double>(3, 1);
					b043 = Homogenous_matrix_TB0.at<double>(3, 2);
					b044 = Homogenous_matrix_TB0.at<double>(3, 3);

					exp_beta = atan2(sqrt(b031*b031 + b032*b032), b033);

					if (exp_beta == 0.0)
					{
						exp_alpha = 0;
						exp_gamma = atan2(-b012, b011) * 180 / 3.1416;
					}

					else if (abs(180.0 - exp_beta * 180 / 3.1416) < 1 && abs(180.0 - exp_beta * 180 / 3.1416) > 0.0)
					{
						exp_alpha = 0;
						exp_gamma = atan2(b012, -b011) * 180 / 3.1416;
					}

					else
					{
						exp_alpha = (atan2(b023 / sin(exp_beta), b013 / sin(exp_beta))) * 180 / 3.1416;
						exp_gamma = (atan2(b032 / sin(exp_beta), -b031 / sin(exp_beta))) * 180 / 3.1416;
					}


					exp_beta = exp_beta * 180 / 3.1416;

					exp_x = b014;
					exp_y = b024;
					exp_z = b034;




					if ((exp_x - end_x) > 0 || (exp_x - end_x) < 0 || (exp_y - end_y) > 0 || (exp_y - end_y) < 0 || (exp_z - end_z) > 0 || (exp_z - end_z) < 0)//切割x y z，程式順序會造成追蹤的順序，0.3跟0.1為經度
					{
						while (abs((exp_x - end_x)) > 0.3 || abs((exp_y - end_y)) > 0.3 || abs((exp_z - end_z)) > 0.3)
						{


							if ((exp_x - end_x) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_x = end_x + 0.2;
								test.push_back(temp);
							}
							if ((exp_x - end_x) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_x = end_x - 0.2;
								test.push_back(temp);
							}
							if ((exp_y - end_y) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_y = end_y + 0.2;
								test.push_back(temp);
							}
							if ((exp_y - end_y) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_y = end_y - 0.2;
								test.push_back(temp);
							}
							if ((exp_z - end_z) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_z = end_z + 0.2;
								test.push_back(temp);
							}
							if ((exp_z - end_z) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_z = end_z - 0.2;
								test.push_back(temp);
							}

						}



					}

					ccc = test.size();
					bomb = 0;
				}

#pragma endregion 新期望點
#pragma region 拍照
				if (bomb == 2)
				{
					cout << "BG矩陣" << endl;
					FILE *fPtr_floatx = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_x.txt", "r");
					float data_floatx[1];
					fscanf(fPtr_floatx, "%f", &data_floatx[0]);
					fclose(fPtr_floatx);
					FILE *fPtr_floaty = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_y.txt", "r");
					float data_floaty[1];
					fscanf(fPtr_floaty, "%f", &data_floaty[0]);
					fclose(fPtr_floaty);
					FILE *fPtr_floatz = fopen("C:\\Users\\笙昊\\Desktop\\LAB STUFF\\youbot\\N10\\Project1\\Project1\\tgb_z.txt", "r");
					float data_floatz[1];
					fscanf(fPtr_floatz, "%f", &data_floatz[0]);
					fclose(fPtr_floatz);




					cout << "bg11:";

					bg11 = 1;

					cout << "bg12:";

					bg12 = 0;

					cout << "bg13:";
					bg13 = 0;

					cout << "bg14:";
					bg14 = data_floatx[0];

					cout << "bg21:";
					bg21 = 0;

					cout << "bg22:";

					bg22 = 1;

					cout << "bg23:";
					bg23 = 0;

					cout << "bg24:";
					bg24 = data_floaty[0];

					cout << "bg31:";
					bg31 = 0;

					cout << "bg32:";

					bg32 = 0;

					cout << "bg33:";
					bg33 = 1;

					cout << "bg34:";
					bg34 = data_floatz[0];

					cout << "bg41:";
					bg41 = 0;

					cout << "bg42:";

					bg42 = 0;

					cout << "bg43:";
					bg43 = 0;

					cout << "bg44:";
					bg44 = 1;
					Homogenous_matrix_TBG.at<double>(0, 0) = bg11;
					Homogenous_matrix_TBG.at<double>(0, 1) = bg12;
					Homogenous_matrix_TBG.at<double>(0, 2) = bg13;
					Homogenous_matrix_TBG.at<double>(0, 3) = bg14;
					Homogenous_matrix_TBG.at<double>(1, 0) = bg21;
					Homogenous_matrix_TBG.at<double>(1, 1) = bg22;
					Homogenous_matrix_TBG.at<double>(1, 2) = bg23;
					Homogenous_matrix_TBG.at<double>(1, 3) = bg24;
					Homogenous_matrix_TBG.at<double>(2, 0) = bg31;
					Homogenous_matrix_TBG.at<double>(2, 1) = bg32;
					Homogenous_matrix_TBG.at<double>(2, 2) = bg33;
					Homogenous_matrix_TBG.at<double>(2, 3) = bg34;
					Homogenous_matrix_TBG.at<double>(3, 0) = bg41;
					Homogenous_matrix_TBG.at<double>(3, 1) = bg42;
					Homogenous_matrix_TBG.at<double>(3, 2) = bg43;
					Homogenous_matrix_TBG.at<double>(3, 3) = bg44;

					Homogenous_matrix_TB0 = Homogenous_matrix_TBG*T50;

					b011 = Homogenous_matrix_TB0.at<double>(0, 0);
					b012 = Homogenous_matrix_TB0.at<double>(0, 1);
					b013 = Homogenous_matrix_TB0.at<double>(0, 2);
					b014 = Homogenous_matrix_TB0.at<double>(0, 3);
					b021 = Homogenous_matrix_TB0.at<double>(1, 0);
					b022 = Homogenous_matrix_TB0.at<double>(1, 1);
					b023 = Homogenous_matrix_TB0.at<double>(1, 2);
					b024 = Homogenous_matrix_TB0.at<double>(1, 3);
					b031 = Homogenous_matrix_TB0.at<double>(2, 0);
					b032 = Homogenous_matrix_TB0.at<double>(2, 1);
					b033 = Homogenous_matrix_TB0.at<double>(2, 2);
					b034 = Homogenous_matrix_TB0.at<double>(2, 3);
					b041 = Homogenous_matrix_TB0.at<double>(3, 0);
					b042 = Homogenous_matrix_TB0.at<double>(3, 1);
					b043 = Homogenous_matrix_TB0.at<double>(3, 2);
					b044 = Homogenous_matrix_TB0.at<double>(3, 3);

					exp_beta = atan2(sqrt(b031*b031 + b032*b032), b033);

					if (exp_beta == 0.0)
					{
						exp_alpha = 0;
						exp_gamma = atan2(-b012, b011) * 180 / 3.1416;
					}

					else if (abs(180.0 - exp_beta * 180 / 3.1416) < 1 && abs(180.0 - exp_beta * 180 / 3.1416) > 0.0)
					{
						exp_alpha = 0;
						exp_gamma = atan2(b012, -b011) * 180 / 3.1416;
					}

					else
					{
						exp_alpha = (atan2(b023 / sin(exp_beta), b013 / sin(exp_beta))) * 180 / 3.1416;
						exp_gamma = (atan2(b032 / sin(exp_beta), -b031 / sin(exp_beta))) * 180 / 3.1416;
					}


					exp_beta = exp_beta * 180 / 3.1416;

					exp_x = b014;
					exp_y = b024;
					exp_z = b034;




					if ((exp_x - end_x) > 0 || (exp_x - end_x) < 0 || (exp_y - end_y) > 0 || (exp_y - end_y) < 0 || (exp_z - end_z) > 0 || (exp_z - end_z) < 0)//切割x y z，程式順序會造成追蹤的順序，0.3跟0.1為經度
					{
						while (abs((exp_x - end_x)) > 0.3 || abs((exp_y - end_y)) > 0.3 || abs((exp_z - end_z)) > 0.3)
						{


							if ((exp_x - end_x) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_x = end_x + 0.2;
								test.push_back(temp);
							}
							if ((exp_x - end_x) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_x = end_x - 0.2;
								test.push_back(temp);
							}
							if ((exp_y - end_y) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_y = end_y + 0.2;
								test.push_back(temp);
							}
							if ((exp_y - end_y) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_y = end_y - 0.2;
								test.push_back(temp);
							}
							if ((exp_z - end_z) > 0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_z = end_z + 0.2;
								test.push_back(temp);
							}
							if ((exp_z - end_z) < -0.2)
							{
								temp.x = end_x;
								temp.y = end_y;
								temp.z = end_z;
								end_z = end_z - 0.2;
								test.push_back(temp);
							}

						}



					}

					ccc = test.size();
					bomb = 0;
				}
#pragma endregion 拍照

#pragma region 夾爪
				if (bomb == 3)
				{
					cout << "等夾爪開" << endl;
					SLEEP_MILLISEC(5000);
					cout << "爪子~~" << endl;
					myYouBotManipulator->getArmGripper().open();
					SLEEP_MILLISEC(3000);
					cout << "給期望點按1夾爪放開按3結束按5" << endl;
					cin >> bomb;
				}
#pragma endregion 夾爪
				if (bomb == 5)//結束
				{


					stop = clock(); //結束時間

					nowtime << ":    " << double(stop - start) / CLOCKS_PER_SEC << endl;
					cout << double(stop - start) / CLOCKS_PER_SEC << endl;
					exit(1);

				}

#pragma region 給新姿態
				if (bomb == 9)
				{
					cout << "1:";

					cin >> ang1;

					cout << "2:";

					cin >> ang2;

					cout << "3:";
					cin >> ang3;

					cout << "4:";
					cin >> ang4;

					cout << "5:";

					cin >> ang5;





					//初始姿態
					init1 = 20;//原本是20
					init2 = -10;
					init3 = -60;
					init4 = -70;
					init5 = 0;


					theta1 = theta2 = theta3 = theta4 = theta5 = theta234 = 0.0;
#pragma region 逆向
					///////////////////////逆向運動學///////////////////////////
					//p_alpha = p_alpha*deg2rad;
					//p_beta = p_beta*deg2rad;
					//p_gamma = p_gamma*deg2rad;
					//ca = cos(p_alpha);
					//
					//cb = cos(p_beta);
					//cr = cos(p_gamma);
					//sa = sin(p_alpha);
					//sb = sin(p_beta);
					//sr = sin(p_gamma);

					//R_ZYZ.at<double>(0, 0) = ca*cb*cr - sa*sr; R_ZYZ.at<double>(0, 1) = -ca*cb*sr - sa*cr;		 R_ZYZ.at<double>(0, 2) = ca*sb; R_ZYZ.at<double>(0, 3) = px;
					//R_ZYZ.at<double>(1, 0) = sa*cb*cr + ca*sr; R_ZYZ.at<double>(1, 1) = -sa*cb*sr + ca*cr;		 R_ZYZ.at<double>(1, 2) = sa*sb; R_ZYZ.at<double>(1, 3) = py;
					//R_ZYZ.at<double>(2, 0) = -sb*cr;		R_ZYZ.at<double>(2, 1) = sb*sr;		 R_ZYZ.at<double>(2, 2) = cb;	      R_ZYZ.at<double>(2, 3) = pz;
					//R_ZYZ.at<double>(3, 0) = 0.0;		R_ZYZ.at<double>(3, 1) = 0.0;		 R_ZYZ.at<double>(3, 2) = 0.0;	      R_ZYZ.at<double>(3, 3) = 1.0;

					//theta1_inv = atan2(py, px);
					//theta1_inv = -theta1_inv;
					//theta1_inv = theta1_inv + 140 * deg2rad;

					//theta5_inv = atan2(cos(theta1_inv)*(sa*cb*cr + ca*sr) - sin(theta1_inv)*(ca*cb*cr - sa*sr), cos(theta1_inv)*(-sa*cb*sr + ca*cr) - sin(theta1_inv)*(-ca*cb*sr - sa*cr));
					//theta5_inv = theta5_inv;
					//theta234_inv = atan2(-cos(theta1_inv)*(ca*sb) - sin(theta1_inv)*(sa*sb), (cb));

					//T01.at<double>(0, 0) = cos(theta1_inv); T01.at<double>(0, 1) = 0;		 T01.at<double>(0, 2) = sin(theta1_inv); T01.at<double>(0, 3) = 0;
					//T01.at<double>(1, 0) = sin(theta1_inv); T01.at<double>(1, 1) = 0;		 T01.at<double>(1, 2) = -cos(theta1_inv); T01.at<double>(1, 3) = 0;
					//T01.at<double>(2, 0) = 0.0;				T01.at<double>(2, 1) = 1;		 T01.at<double>(2, 2) = 0;					T01.at<double>(2, 3) = d1;
					//T01.at<double>(3, 0) = 0.0;				T01.at<double>(3, 1) = 0.0;		 T01.at<double>(3, 2) = 0.0;	      T01.at<double>(3, 3) = 1.0;

					//T45.at<double>(0, 0) = cos(theta5_inv - 90); T45.at<double>(0, 1) = -sin(theta5_inv - 90);		 T45.at<double>(0, 2) = 0; T45.at<double>(0, 3) = 0;
					//T45.at<double>(1, 0) = sin(theta5_inv - 90); T45.at<double>(1, 1) = cos(theta5_inv - 90);		 T45.at<double>(1, 2) = 0; T45.at<double>(1, 3) = 0;
					//T45.at<double>(2, 0) = 0.0;					T45.at<double>(2, 1) = 0;							 T45.at<double>(2, 2) = 1;					T45.at<double>(2, 3) = d5;
					//T45.at<double>(3, 0) = 0.0;					T45.at<double>(3, 1) = 0.0;								T45.at<double>(3, 2) = 0.0;	      T45.at<double>(3, 3) = 1.0;

					//T45inv = T45.inv(DECOMP_SVD);

					//T04 = R_ZYZ*T45inv;

					//double l2 = 15.5;
					//double l3 = 13.5;
					//double pxn, pyn, pzn;
					//double l24;
					//x2 = 0;
					//y2 = 0;
					//z2 = d1;
					//x4 = T04.at<double>(0, 3);
					//y4 = T04.at<double>(1, 3);
					//z4 = T04.at<double>(2, 3);
					//pxn = x4 - x2;
					//pyn = y4 - y2;
					//pzn = z4 - z2;
					//l24 = sqrt((x4 - x2)*(x4 - x2) + (y4 - y2)*(y4 - y2) + (z4 - z2)*(z4 - z2));
					//cos_theta3 = (l24*l24 - l2*l2 - l3*l3) / (2 * l2*l3);
					//theta3_inv = atan2(sqrt(1 - cos_theta3*cos_theta3), cos_theta3);

					//theta2_inv = -atan2(l3*sin(theta3_inv), l2 + l3*cos(theta3_inv)) - atan2(pzn, sqrt(pxn*pxn + pyn*pyn));
					//theta2_inv = theta2_inv + 90*deg2rad;

					//theta4_inv = theta234_inv - theta2_inv - theta3_inv;

					///////////////////////////逆向運動學end//////////////////////////////
#pragma endregion 逆向


					//half_pi *= deg2rad ;

					//ang2 = ang2 + 90 ;

					//ang1 = theta1_inv;//ANG1=ANG1*DEG2RAD
					//ang2 = theta2_inv;//逆向運動學得到的角度，逆向運動學其實只會用一次沒啥鬼屌用
					//ang3 = theta3_inv;
					//ang4 = theta4_inv;
					//ang5 = theta5_inv;

					init1 *= deg2rad;
					init2 *= deg2rad;
					init3 *= deg2rad;
					init4 *= deg2rad;
					init5 *= deg2rad;

					ang1 *= deg2rad;
					ang2 *= deg2rad;
					ang3 *= deg2rad;
					ang4 *= deg2rad;
					ang5 *= deg2rad;

					ang1 = -ang1;
					ang2 = -ang2;
					ang3 = -ang3;
					ang4 = -ang4;
					ang5 = -ang5;



					init1 = -init1;
					init2 = -init2;
					init3 = -init3;
					init4 = -init4;
					init5 = -init5;

					///====Forword=====//
					Homogenous_matrix_T10.at<double>(0, 0) = cos(ang1); Homogenous_matrix_T10.at<double>(0, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(0, 2) = sin(ang1); Homogenous_matrix_T10.at<double>(0, 3) = 0.0;
					Homogenous_matrix_T10.at<double>(1, 0) = sin(ang1); Homogenous_matrix_T10.at<double>(1, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(1, 2) = -cos(ang1); Homogenous_matrix_T10.at<double>(1, 3) = 0.0;
					Homogenous_matrix_T10.at<double>(2, 0) = 0.0;		Homogenous_matrix_T10.at<double>(2, 1) = 1.0;		 Homogenous_matrix_T10.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(2, 3) = d1;
					Homogenous_matrix_T10.at<double>(3, 0) = 0.0;		Homogenous_matrix_T10.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T10.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T10.at<double>(3, 3) = 1.0;

					Homogenous_matrix_T21.at<double>(0, 0) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 1) = -sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(0, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(0, 3) = a2*cos(ang2 + pi / 2);
					Homogenous_matrix_T21.at<double>(1, 0) = sin(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 1) = cos(ang2 + pi / 2); Homogenous_matrix_T21.at<double>(1, 2) = 0.0;		  Homogenous_matrix_T21.at<double>(1, 3) = a2*sin(ang2 + pi / 2);
					Homogenous_matrix_T21.at<double>(2, 0) = 0.0;		Homogenous_matrix_T21.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T21.at<double>(2, 3) = 0.0;
					Homogenous_matrix_T21.at<double>(3, 0) = 0.0;		Homogenous_matrix_T21.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T21.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T21.at<double>(3, 3) = 1.0;

					Homogenous_matrix_T32.at<double>(0, 0) = cos(ang3); Homogenous_matrix_T32.at<double>(0, 1) = -sin(ang3); Homogenous_matrix_T32.at<double>(0, 2) = 0.0;       Homogenous_matrix_T32.at<double>(0, 3) = a3*cos(ang3);
					Homogenous_matrix_T32.at<double>(1, 0) = sin(ang3); Homogenous_matrix_T32.at<double>(1, 1) = cos(ang3); Homogenous_matrix_T32.at<double>(1, 2) = 0.0;       Homogenous_matrix_T32.at<double>(1, 3) = a3*sin(ang3);
					Homogenous_matrix_T32.at<double>(2, 0) = 0.0;		Homogenous_matrix_T32.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T32.at<double>(2, 3) = 0.0;
					Homogenous_matrix_T32.at<double>(3, 0) = 0.0;		Homogenous_matrix_T32.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T32.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T32.at<double>(3, 3) = 1.0;
					///Origin
					Homogenous_matrix_T43.at<double>(0, 0) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 1) = 0.0;       Homogenous_matrix_T43.at<double>(0, 2) = -sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(0, 3) = 0.0;
					Homogenous_matrix_T43.at<double>(1, 0) = sin(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 1) = 0.0;       Homogenous_matrix_T43.at<double>(1, 2) = cos(ang4 - pi / 2); Homogenous_matrix_T43.at<double>(1, 3) = 0.0;
					Homogenous_matrix_T43.at<double>(2, 0) = 0.0;		Homogenous_matrix_T43.at<double>(2, 1) = -1.0;		 Homogenous_matrix_T43.at<double>(2, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(2, 3) = 0.0;
					Homogenous_matrix_T43.at<double>(3, 0) = 0.0;		Homogenous_matrix_T43.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T43.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T43.at<double>(3, 3) = 1.0;

					Homogenous_matrix_T54.at<double>(0, 0) = cos(ang5); Homogenous_matrix_T54.at<double>(0, 1) = -sin(ang5); Homogenous_matrix_T54.at<double>(0, 2) = 0.0;       Homogenous_matrix_T54.at<double>(0, 3) = 0.0;
					Homogenous_matrix_T54.at<double>(1, 0) = sin(ang5); Homogenous_matrix_T54.at<double>(1, 1) = cos(ang5); Homogenous_matrix_T54.at<double>(1, 2) = 0.0;       Homogenous_matrix_T54.at<double>(1, 3) = 0.0;
					Homogenous_matrix_T54.at<double>(2, 0) = 0.0;		Homogenous_matrix_T54.at<double>(2, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(2, 2) = 1.0;	      Homogenous_matrix_T54.at<double>(2, 3) = d5;
					Homogenous_matrix_T54.at<double>(3, 0) = 0.0;		Homogenous_matrix_T54.at<double>(3, 1) = 0.0;		 Homogenous_matrix_T54.at<double>(3, 2) = 0.0;	      Homogenous_matrix_T54.at<double>(3, 3) = 1.0;
					///Origin




					T10 = Homogenous_matrix_T10;
					T20 = T10*Homogenous_matrix_T21;
					T30 = T20*Homogenous_matrix_T32;
					T40 = T30*Homogenous_matrix_T43;
					T50 = T40*Homogenous_matrix_T54;

					r11 = T50.at<double>(0, 0);
					r12 = T50.at<double>(0, 1);
					r13 = T50.at<double>(0, 2);
					r14 = T50.at<double>(0, 3);
					r21 = T50.at<double>(1, 0);
					r22 = T50.at<double>(1, 1);
					r23 = T50.at<double>(1, 2);
					r24 = T50.at<double>(1, 3);
					r31 = T50.at<double>(2, 0);
					r32 = T50.at<double>(2, 1);
					r33 = T50.at<double>(2, 2);
					r34 = T50.at<double>(2, 3);







					double show_ang1, show_ang2, show_ang3, show_ang4, show_ang5, show_ang0234;
					double t_ang1, t_ang2, t_ang3, t_ang4, t_ang5; ///before input youBot
					show_ang1 = ang1 * rad2deg;
					show_ang2 = ang2 * rad2deg;
					show_ang3 = ang3 * rad2deg;
					show_ang4 = ang4 * rad2deg;
					show_ang5 = ang5 * rad2deg;
					show_ang0234 = show_ang2 + show_ang3 + show_ang4;

					t_ang1 = ang1;
					t_ang2 = ang2;
					t_ang3 = ang3;
					t_ang4 = ang4;
					t_ang5 = ang5;

					///Because Assume opposite to real Arm.
					ang1 = -ang1;
					ang5 = -ang5;

					init1 = -init1;
					init5 = -init5;

					/*double theta1_zero = 2.56244;
					double theta2_zero = 1.04883;
					double theta3_zero = -2.43523;
					double theta4_zero = 1.73184;
					double theta5_zero = 2.8761045;*///從上面複製過來的 這是昊昊自己看的

					ang1 = ang1 + theta1_zero;
					ang2 = ang2 + theta2_zero;
					ang3 = ang3 + theta3_zero;
					ang4 = ang4 + theta4_zero;
					ang5 = ang5 + theta5_zero;


					init1 = init1 + theta1_zero;
					init2 = init2 + theta2_zero;
					init3 = init3 + theta3_zero;
					init4 = init4 + theta4_zero;
					init5 = init5 + theta5_zero;
					cout << "T10" << T10 << endl;
					cout << "T20" << T20 << endl;
					cout << "T30" << T30 << endl;
					cout << "T40" << T40 << endl;
					cout << "T50" << T50 << endl;
					cout << "給youbot角度 " << ang1 << " " << ang2 << "" << ang3 << " " << ang4 << " " << ang5 << endl;
					///Test Move
					cout << "Test Forward Move " << endl;


					//desiredJointAngle.angle = ang1 * radian;//2.9150354
					//myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
					//控制部分
					desiredJointAngle.angle = ang1 * radian;
					myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

					desiredJointAngle.angle = ang2 * radian;
					myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

					desiredJointAngle.angle = ang3 * radian;
					myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

					desiredJointAngle.angle = ang4 * radian;
					myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

					desiredJointAngle.angle = ang5 * radian;
					myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

					beta = atan2(sqrt(r31*r31 + r32*r32), r33);

					if (beta == 0.0)
					{
						alpha = 0;
						gamma = atan2(-r12, r11) * 180 / 3.1416;
					}

					else if (abs(180.0 - beta * 180 / 3.1416) < 1 && abs(180.0 - beta * 180 / 3.1416) > 0.0)
					{
						alpha = 0;
						gamma = atan2(r12, -r11) * 180 / 3.1416;
					}

					else
					{
						alpha = (atan2(r23 / sin(beta), r13 / sin(beta))) * 180 / 3.1416;
						gamma = (atan2(r32 / sin(beta), -r31 / sin(beta))) * 180 / 3.1416;
					}


					beta = beta * 180 / 3.1416;



					tool_end = T50*tool;//只有位移量?????????????????????????????


					end_x = tool_end.at<double>(0, 0);
					end_y = tool_end.at<double>(1, 0);
					end_z = tool_end.at<double>(2, 0);

					LOG(info) << "unfold arm";
					SLEEP_MILLISEC(3000);
					cout << "給期望點按1 拍照按2 夾爪放開按3 開車按7 新姿態9 夾爪10 " << endl;
					cin >> bomb;
				}

#pragma endregion 給新姿態
#pragma region 開車車
				if (bomb == 7)//開車車
				{
					cout << "w a s d x" << endl;
					while (1){//////車子
						if (youBotHasBase) {
							// Variable for the base.
							// Here "boost units" is used to set values, that means you have to set a value and a unit.
							quantity<si::velocity> vx = 0 * meter_per_second;
							quantity<si::velocity> vy = 0 * meter_per_second;
							quantity<si::angular_velocity> va = 0 * radian_per_second;////另quantity必須要以算式去命名

							if (GetAsyncKeyState(0x57))	//w
							{
								// forward
								vx = vLinear * meter_per_second;
								vy = 0 * meter_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);

								SLEEP_MILLISEC(10);
							}
							else if (GetAsyncKeyState(0x53)) //s
							{
								// backwards
								vx = -vLinear * meter_per_second;
								vy = 0 * meter_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);
								//LOG(info) << "drive backwards";
								SLEEP_MILLISEC(10);
							}
							else if (GetAsyncKeyState(0x41)) //a
							{
								// left
								vx = 0 * meter_per_second;
								vy = vLinear * meter_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);
								//LOG(info) << "drive left";
								SLEEP_MILLISEC(10);
							}
							else if (GetAsyncKeyState(0x44)) //d
							{
								// right 
								vx = 0 * meter_per_second;
								vy = -vLinear * meter_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);
								//LOG(info) << "drive right";
								SLEEP_MILLISEC(10);
							}
							else if (GetAsyncKeyState(0x58)) //x
							{
								// stop base 
								vx = 0 * meter_per_second;
								vy = 0 * meter_per_second;
								va = 0 * radian_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);
								LOG(info) << "stop base";



								break;
							}
							else if (GetAsyncKeyState(VK_ESCAPE)) break;
							else
							{
								// stop base 
								vx = 0 * meter_per_second;
								vy = 0 * meter_per_second;
								va = 0 * radian_per_second;
								myYouBotBase->setBaseVelocity(vx, vy, va);
								//LOG(info) << "stop base";
								//break;
							}
						}
					}

					cout << "給期望點按1夾爪放開按3結束按5開車按7" << endl;
					cin >> bomb;
				}

#pragma endregion 開車車





			}


		}



	}


#pragma endregion 速度控制
	double all;

	//while迴圈括號


	SLEEP_MILLISEC(4000);
	return 0;
}





