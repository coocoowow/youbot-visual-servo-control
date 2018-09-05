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

class exp_joint_point {
public:
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;

};

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
class exp_acc {
public:
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;

};

class traj{

public:
	double ang1;
	double ang2;
	double ang3;
	double ang4;
	double ang5;
	double vel1;
	double vel2;
	double vel3;
	double vel4;
	double vel5;
	//double acc1;
	//double acc2;
	//double acc3;
	//double acc4;
	//double acc5;
};

class img_torque{
public:
	double t1;
	double t2;
	double t3;
	double t4;
	double t5;

};

class exp_vel {
public:
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;

};

exp_vel veltemp;
vector <exp_vel>vel_test;
img_torque imgtemp_torque;
vector<img_torque> imgtest_torque;
exp_point temp;
vector<exp_point>test;
traj traj_detial;
vector<traj> traj_all;


exp_acc acctemp;
vector<exp_acc> acc_test;


exp_joint_point temp_ang;
vector<exp_joint_point> temp_point;

double deg2rad = pi / 180;

double rad2deg = 180 / pi;

double theta1_zero = 2.9496;
double theta2_zero = 1.1345;
double theta3_zero = -2.5482;
double theta4_zero = 1.789;
double theta5_zero = 2.9234;
double car_fowpos;
double car_sidepos;
double car_rot;

///Inverse Kinemetics

cv::Mat T10 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T20 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T30 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T40 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T50 = cv::Mat(4, 4, CV_64FC1);
cv::Mat T54inv = cv::Mat(4, 4, CV_64FC1);

double ang1 = 0.0, ang2 = 0.0, ang3 = 0.0, ang4 = 0.0, ang5 = 0.0;
double init1 = 0.0, init2 = 0.0, init3 = 0.0, init4 = 0.0, init5 = 0.0;
long double sang1 = 0.0, sang2 = 0.0, sang3 = 0.0, sang4 = 0.0, sang5 = 0.0;
//////////////////////////力矩////////////////////////////
int i = 0;
int qqq = 0;
double c_2 = cos(sang2);
double c_3 = cos(sang3);
double c_4 = cos(sang4);
double c_5 = cos(sang5);
double s_2 = sin(sang2);
double s_3 = sin(sang3);
double s_4 = sin(sang4);
double  s_5 = sin(sang5);
double s_23 = sin(sang2 - sang3);
double s_34 = sin(sang3 - sang4);
double s2_2 = sin(2 * sang2);
double s2_5 = sin(2 * sang5);
double c_23 = cos(sang2 - sang3);
double c_34 = cos(sang3 - sang4);
double c_45 = cos(sang4 + sang5);
double c_4_5 = cos(sang4 - sang5);
double c_25 = cos(sang2 + sang5);
double c_2_5 = cos(sang2 - sang5);
double s2_23 = sin(2 * sang2 - 2 * sang3);
double c_345 = cos(sang3 - sang4 + sang5);
double c_34_5 = cos(sang3 - sang4 - sang5);
double c_235 = cos(sang2 - sang3 + sang5);
double c_23_5 = cos(sang2 - sang3 - sang5);
double c_234 = cos(sang2 - sang3 + sang4);
double s_234 = sin(sang2 - sang3 + sang4);
double s_223 = sin(2 * sang2 - sang3);
double s2_234 = sin(2 * sang2 - 2 * sang3 + 2 * sang4);
double s_2234 = sin(2 * sang2 - sang3 + sang4);
double c_2234 = cos(2 * sang2 - sang3 + sang4);
double c2_234 = cos(2 * sang2 - 2 * sang3 + 2 * sang4);
double c_22345 = cos(2 * sang2 - sang3 + sang4 + sang5);
double c_2234_5 = cos(2 * sang2 - sang3 + sang4 - sang5);
double c_2345 = cos(sang2 - sang3 + sang4 + sang5);
double s_2345 = sin(sang2 - sang3 + sang4 + sang5);
double s_234_5 = sin(sang2 - sang3 + sang4 - sang5);
double c_234_5 = cos(sang2 - sang3 + sang4 - sang5);
double s_234_25 = sin(sang2 - sang3 + sang4 - 2 * sang5);
double s_23425 = sin(sang2 - sang3 + sang4 + 2 * sang5);
double s_22234 = sin(2 * sang2 - 2 * sang3 + sang4);
double c_22234 = cos(2 * sang2 - 2 * sang3 + sang4);
double c_22234_5 = cos(2 * sang2 - 2 * sang3 + sang4 - sang5);
double c_222345 = cos(2 * sang2 - 2 * sang3 + sang4 + sang5);
double s_2223245 = sin(2 * sang2 - 2 * sang3 + 2 * sang4 + sang5);
double s_222324_5 = sin(2 * sang2 - 2 * sang3 + 2 * sang4 - sang5);
double c_222324_5 = cos(2 * sang2 - 2 * sang3 + 2 * sang4 - sang5);
double c_2223245 = cos(2 * sang2 - 2 * sang3 + 2 * sang4 + sang5);
double s2_2345 = sin(2 * sang2 - 2 * sang3 + 2 * sang4 + 2 * sang5);
double s2_234_5 = sin(2 * sang2 - 2 * sang3 + 2 * sang4 - 2 * sang5);

double term_1 = (2259.0*sin(sang2 - sang3 + sang4 + sang5)) / 62500000.0;
double  term_2 = (753.0*cos(sang2 - sang3 + sang4 - sang5)) / 1000000000.0;
double term_3 = (3969783.0*c_4);
double  term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
double  term_5 = (753.0*c_5);
double  term_6 = (27027.0*s_4);
double  term_7 = (20331.0*c_5*s_4);
double  term_8 = (4557899.0*s_3*s_4);
double  term_9 = (4557899.0*c_3*c_4);
double  term_10 = (31031.0*c_3*s_4);
double  term_11 = (753.0*cos(sang2 - sang3 + sang4 + sang5));
double  term_12 = (31031.0*c_4*s_3);
double  term_13 = (2259.0*cos(sang2 - sang3 + sang4 - 2 * sang5));
double  term_14 = (2259.0*cos(sang2 - sang3 + sang4 + 2 * sang5));
double  term_15 = (2259.0*sin(sang2 - sang3 + sang4 - sang5));
double  term_16 = (20331.0*sin(sang2 - sang3 + sang5));
double  term_17 = (25433.0*cos(sang2 - sang3 + sang4));
double  term_18 = (23343.0*c_3*c_5*s_4);
double  term_19 = (23343.0*c_4*c_5*s_3);
cv::Mat M_matrix = cv::Mat(5, 5, CV_64FC1);//慣性
cv::Mat C_matrix = cv::Mat(5, 5, CV_64FC1);//科氏力
cv::Mat N_matrix = cv::Mat(5, 1, CV_64FC1);//重力
cv::Mat tau_matrix = cv::Mat(5, 1, CV_64FC1);//重力
cv::Mat ang_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角度
cv::Mat vel_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角速度
cv::Mat velcal_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角速度
cv::Mat acc_matrix = cv::Mat(5, 1, CV_64FC1);//各軸角加速度
////////////////////////力矩//////////////////////////////
cv::Mat Homogenous_matrix_T10 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T21 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T32 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T43 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_T54 = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_TBG = cv::Mat(4, 4, CV_64FC1);
cv::Mat Homogenous_matrix_TB0 = cv::Mat(4, 4, CV_64FC1);
cv::Mat IDentity = cv::Mat(3, 3, CV_64FC1);
clock_t start_time, end_time;
//vector<long double>total_time;
long double total_time = 0;
long double temp_time = 0;
long double t_time = 0;



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


std::ofstream nowx("./nowx.txt");//寫記事本
std::ofstream nowy("./nowy.txt");//寫記事本
std::ofstream nowz("./nowz.txt");//寫記事本
std::ofstream nowa("./now_alpha.txt");//寫記事本
std::ofstream nowb("./now_beta.txt");//寫記事本
std::ofstream nowr("./now_gamma.txt");//寫記事本
std::ofstream tempvel("./tempvel.txt");//寫記事本
std::ofstream nowtime("./nowtime.txt");//寫記事本
std::ofstream toatime("./toatime.txt");//寫記事本
std::ofstream Velocity1("./Velocity1.txt");//寫記事本
std::ofstream Velocity2("./Velocity2.txt");//寫記事本
std::ofstream Velocity3("./Velocity3.txt");//寫記事本
std::ofstream Velocity4("./Velocity4.txt");//寫記事本
std::ofstream Velocity5("./Velocity5.txt");//寫記事本
std::ofstream joint1("./joint1.txt");//寫記事本
std::ofstream joint2("./joint2.txt");//寫記事本
std::ofstream joint3("./joint3.txt");//寫記事本
std::ofstream joint4("./joint4.txt");//寫記事本
std::ofstream joint5("./joint5.txt");//寫記事本
std::ofstream Acc1("./Acc1.txt");//寫記事本
std::ofstream Acc2("./Acc2.txt");//寫記事本
std::ofstream Acc3("./Acc3.txt");//寫記事本
std::ofstream Acc4("./Acc4.txt");//寫記事本
std::ofstream Acc5("./Acc5.txt");//寫記事本
std::ofstream Tau1("./Tau1.txt");//寫記事本
std::ofstream Tau2("./Tau2.txt");//寫記事本
std::ofstream Tau3("./Tau3.txt");//寫記事本
std::ofstream Tau4("./Tau4.txt");//寫記事本
std::ofstream Tau5("./Tau5.txt");//寫記事本
std::ofstream JointVelocity("./JointVelocity.txt");//寫記事本
std::ofstream JointAcc("./JointAcc.txt");//寫記事本
std::ofstream JointTau("./JointTau.txt");//寫記事本
std::ofstream CalJointTau("./CalJointTau.txt");//寫記事本
std::ofstream Jointx("./Jointx.txt");//寫記事本
std::ofstream jointerror("./jointerror.txt");//寫記事本
std::ofstream jointvelerror("./jointvelerror.txt");//寫記事本
std::ofstream jointdesiredvel("./jointdesiredvel.txt");//寫記事本
std::ofstream jointdesiredang("./jointdesiredang.txt");//寫記事本
std::ofstream sliding("./sliding.txt");//寫記事本
std::ofstream cutang("./cutang.txt");//寫記事本
std::ofstream cutvel("./cutvel.txt");//寫記事本
std::ofstream cutacc("./cutacc.txt");//寫記事本
std::ofstream Usww("./Usww.txt");//寫記事本
std::ofstream QAZ("./QAZ.txt");//寫記事本
std::ofstream Ueqq("./Ueqq.txt");//寫記事本
int special;
int bomb;
int upg;
clock_t start, stop;
long double ttime;
double lamda_v = 0.999, lamda_w = 0.999;//0.5

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
	JointSensedTorque EachJointTorque;//encoder讀出來的手臂力矩
	vector<JointSensedTorque> EachJointTorque1;

	double exp_x, exp_y, exp_z;//期望位置
	double expn_x, expn_y, expn_z;
	double exp_gamma, exp_beta, exp_alpha;//期望角度alpha:z軸角度，beta:y軸角度，gamma:x軸角度
	double expn_gamma, expn_beta, expn_alpha;





	double sample_time = 0.15;

#pragma region 夾取成功(暫時註解)
	if (youBotHasGripper)
	{
		myYouBotManipulator->getArmGripper().open();///夾爪開
		//myYouBotManipulator->getArmGripper().close();///夾爪關
		SLEEP_MILLISEC(4000);
	}


	cout << "w a s d x" << endl;
#pragma endregion 夾取成功
	//myYouBotManipulator->getArmGripper().open();
	SLEEP_MILLISEC(1000);



	double end_x, end_y, end_z;
	cv::Mat tool_end = cv::Mat(4, 1, CV_64FC1);
	cv::Mat tool = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1);//////Mat_<double>对应的是CV_64F
	Mat R_ZYZ = cv::Mat(4, 4, CV_64FC1);
	Mat T01 = cv::Mat(4, 4, CV_64FC1);
	Mat T12 = Mat(4, 4, CV_64FC1);
	Mat T23 = Mat(4, 4, CV_64FC1);
	Mat T34 = Mat(4, 4, CV_64FC1);
	Mat T05 = Mat(4, 4, CV_64FC1);
	Mat T45 = cv::Mat(4, 4, CV_64FC1);
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
	long double sang1 = 0, sang2 = 0, sang3 = 0, sang4 = 0, sang5 = 0;
	long double svel1 = 0, svel2 = 0, svel3 = 0, svel4 = 0, svel5 = 0;
	long double temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0;
	long double sang1_cal = 0, sang2_cal = 0, sang3_cal = 0, sang4_cal = 0, sang5_cal = 0;
	long double sacc1, sacc2, sacc3, sacc4, sacc5;
	long double stau1, stau2, stau3, stau4, stau5;
	double px, py, pz, p_gamma, p_beta, p_alpha;
	double ca, cb, cr, sa, sb, sr;
	double theta1_inv, theta2_inv, theta3_inv, theta4_inv, theta5_inv, theta234_inv;
	double cos_theta3;

	Mat fordwardKin = Mat(4, 4, CV_64FC1);
	Mat desired_acc = Mat(5, 1, CV_64FC1);
	Mat joint_poserror_dot = Mat(5, 1, CV_64FC1);
	Mat joint_poserror = Mat(5, 1, CV_64FC1);
	Mat desired_vel = Mat(5, 1, CV_64FC1);
	Mat desired_ang = Mat(5, 1, CV_64FC1);
	Mat desired_tempang = Mat(5, 1, CV_64FC1);
	Mat desired_tempvel = Mat(5, 1, CV_64FC1);
	Mat desired_tempacc = Mat(5, 1, CV_64FC1);
	Mat Kp = Mat(5, 5, CV_64FC1);
	Mat Kd = Mat(5, 5, CV_64FC1);
	Mat lambda = Mat(5, 5, CV_64FC1);
	Mat kk = Mat(5, 5, CV_64FC1);
	Mat FF = Mat(5, 1, CV_64FC1);
	Mat FB = Mat(5, 1, CV_64FC1);
	Mat ss = Mat(5, 1, CV_64FC1);
	Mat sw = Mat(5, 1, CV_64FC1);
	Mat Utemp = Mat(5, 1, CV_64FC1);
	Mat Ueq = Mat(5, 1, CV_64FC1);
	Mat Usw = Mat(5, 1, CV_64FC1);
	Mat adj = Mat(5, 5, CV_64FC1);
	Mat adj2 = Mat(5, 5, CV_64FC1);
	Mat check1 = Mat(5, 1, CV_64FC1);
	Mat check2 = Mat(5, 1, CV_64FC1);
	double qq;
	double qaz;
	double J1poserror, J2poserror, J3poserror, J4poserror, J5poserror;
	double J1velerror, J2velerror, J3velerror, J4velerror, J5velerror;
	double N1, N2, N3, N4, N5;
	double FF1, FF2, FF3, FF4, FF5;
	double FB1, FB2, FB3, FB4, FB5;
	double M11, M21, M31, M41, M51, M12, M22, M32, M42, M52, M13, M23, M33, M43, M53, M14, M24, M34, M44, M54, M15, M25, M35, M45, M55;
	double ch1, ch2, ch3, ch4, ch5, ch11, ch22, ch33, ch44, ch55;
	int ccc = 0;
	int xxx = 0;
	int ans;
	while (true)
	{
		while (1)
		{

			//sang1 = EachJointAngle1[0].angle.value();
			//sang2 = EachJointAngle1[1].angle.value();
			//sang3 = EachJointAngle1[2].angle.value();
			//sang4 = EachJointAngle1[3].angle.value();
			//sang5 = EachJointAngle1[4].angle.value();
			//nowx << "第一軸   " << sang1 << "   第二軸  " << sang2 << "   第三軸  " << sang3 << "   第四軸  " << sang4 << "   第五軸" << sang5 << endl;

			cout << "ccc=0回歸原位 ccc=1位置控制拿力矩數據 ccc=2測試 ccc=3重力補償 ccc=4pd控制 ccc=5sliding-mode ccc=6速度控制 ccc=7速度+slidingmode ccc=8開車  ccc=9Yuja" << endl;
			cin >> ccc;


			if (ccc == 8)
			{
				myYouBotManipulator->getArmGripper().close();
				SLEEP_MILLISEC(4000);
				cout << "w a s d x" << endl;
				while (1){//////車子
					if (youBotHasBase) {
						// Variable for the base.
						// Here "boost units" is used to set values, that means you have to set a value and a unit.
						quantity<si::velocity> vx = 0 * meter_per_second;////另quantity必須要以算式去命名
						quantity<si::velocity> vy = 0 * meter_per_second;////另quantity必須要以算式去命名
						quantity<si::angular_velocity> va = 0 * radian_per_second;////另quantity必須要以算式去命名
						quantity<si::length> longitudePosition = 0 * meter;////另quantity必須要以算式去命名
						quantity<si::length> transverPosition = 0 * meter;////另quantity必須要以算式去命名
						quantity<si::plane_angle> orient = 0 * radian;////另quantity必須要以算式去命名

						if (GetAsyncKeyState(0x50)) //p
						{
							
							myYouBotBase->getBasePosition(longitudePosition, transverPosition, orient);
						    car_fowpos = longitudePosition.value();
							car_sidepos = transverPosition.value();
							car_rot = orient.value();
							cout << "前進" << car_fowpos << endl;
							cout << "左右" << car_sidepos << endl;
							cout << "旋轉" << car_rot << endl;
						}
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


			}

			if (ccc == 9)
			{
				if (youBotHasBase) {
					// Variable for the base.
					int x, y = 0;
					cout << "往前移動(cm): ";
					cin >> x;
					cout << "往左移動(cm): ";
					cin >> y;
					cout << endl;

					int forward, left = 0;
					if (x >= 0){ forward = 1; }
					else { forward = 0; }
					if (y >= 0){ left = 1; }
					else{ left = 0; }

					double x_move_time = abs(x) / 5 * 1000;
					double y_move_time = abs(y) / 5 * 1000;


					//////車子

					// Variable for the base.
					// Here "boost units" is used to set values, that means you have to set a value and a unit.
					quantity<si::velocity> vx = 0 * meter_per_second;
					quantity<si::velocity> vy = 0 * meter_per_second;
					quantity<si::angular_velocity> va = 0 * radian_per_second;////另quantity必須要以算式去命名



					if (forward == 1)	//w
					{
						// forward
						vx = vLinear * meter_per_second;
						vy = 0 * meter_per_second;
						myYouBotBase->setBaseVelocity(vx, vy, va);
						SLEEP_MILLISEC(x_move_time);
						myYouBotBase->setBaseVelocity(0, 0, 0);
						cout << "forward" << endl;

						if (left == 1) //a
						{
							// left
							vx = 0 * meter_per_second;
							vy = vLinear * meter_per_second;
							myYouBotBase->setBaseVelocity(vx, vy, va);
							//LOG(info) << "drive left";
							SLEEP_MILLISEC(y_move_time);
							myYouBotBase->setBaseVelocity(0, 0, 0);
							cout << "left" << endl;
						}
						else if (left == 0) //d
						{
							// right 
							vx = 0 * meter_per_second;
							vy = -vLinear * meter_per_second;
							myYouBotBase->setBaseVelocity(vx, vy, va);
							//LOG(info) << "drive right";
							SLEEP_MILLISEC(y_move_time);
							myYouBotBase->setBaseVelocity(0, 0, 0);
							cout << "right" << endl;
						}

					}


					else if (forward == 0) //s
					{
						// backwards
						vx = -vLinear * meter_per_second;
						vy = 0 * meter_per_second;
						myYouBotBase->setBaseVelocity(vx, vy, va);
						//LOG(info) << "drive backwards";
						SLEEP_MILLISEC(x_move_time);
						myYouBotBase->setBaseVelocity(0, 0, 0);
						cout << "backward" << endl;

						if (left == 1) //a
						{
							// left
							vx = 0 * meter_per_second;
							vy = vLinear * meter_per_second;
							myYouBotBase->setBaseVelocity(vx, vy, va);
							//LOG(info) << "drive left";
							SLEEP_MILLISEC(y_move_time);
							myYouBotBase->setBaseVelocity(0, 0, 0);
							cout << "left" << endl;
						}
						else if (left == 0) //d
						{
							// right 
							vx = 0 * meter_per_second;
							vy = -vLinear * meter_per_second;
							myYouBotBase->setBaseVelocity(vx, vy, va);
							//LOG(info) << "drive right";
							SLEEP_MILLISEC(y_move_time);
							myYouBotBase->setBaseVelocity(0, 0, 0);
							cout << "right" << endl;
						}
					}

					else if (GetAsyncKeyState(0x58)) //x
					{
						// stop base 
						vx = 0 * meter_per_second;
						vy = 0 * meter_per_second;
						va = 0 * radian_per_second;
						myYouBotBase->setBaseVelocity(vx, vy, va);
						LOG(info) << "stop base";
					}


				}


			}

#pragma region ccc==0
			if (ccc == 0)
			{

				desiredJointAngle.angle = 0.011 * radian;////指定角度用radian
				myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.037 * radian;////指定角度用radian
				myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				desiredJointAngle.angle = -2.51 * radian;////指定角度用radian
				myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.85 * radian;////指定角度用radian
				myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.12 * radian;////指定角度用radian
				myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

				//desiredJointAngle.angle = 0.011 * radian;
				//myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				//desiredJointAngle.angle = 0.5905 * radian;
				//myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				//desiredJointAngle.angle = -1.2527 * radian;
				//myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				//desiredJointAngle.angle = 0.2889 * radian;
				//myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				//desiredJointAngle.angle = 0.12 * radian;
				//myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);



				//myYouBotManipulator->getArmGripper().open();
				while (1)
				{
					myYouBotManipulator->getJointData(EachJointAngle1);////獲取角度前要加der東西
					sang1 = EachJointAngle1[0].angle.value();////獲取各軸角度
					sang2 = EachJointAngle1[1].angle.value();////獲取各軸角度
					sang3 = EachJointAngle1[2].angle.value();////獲取各軸角度
					sang4 = EachJointAngle1[3].angle.value();////獲取各軸角度
					sang5 = EachJointAngle1[4].angle.value();////獲取各軸角度
					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					//temp1 = svel1;
					//temp2 = svel2;
					//temp3 = svel3;
					//temp4 = svel4;
					//temp5 = svel5;

					myYouBotManipulator->getJointData(EachJointTorque1);////獲取數值前要加der東西
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸" << sang5_cal << endl;
					//tempvel << "第一軸  " << svel1 - temp1 << "  第二軸  " << svel2 - temp2 << "  第三軸  " << svel3 - temp3 << "  第四軸  " << svel4 - temp4 << "  第五軸  " << svel5 - temp5 << endl;
					//JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸" << svel5 << endl;
					//Velocity1 << svel1 << endl;			Velocity2 << svel2 << endl;			Velocity3 << svel3 << endl;			Velocity4 << svel4 << endl;			Velocity5 << svel5 << endl;
					//Acc1 << sacc1 << endl;			Acc2 << sacc2 << endl;			Acc3 << sacc3 << endl;			Acc4 << sacc4 << endl;			Acc5 << sacc5 << endl;
					//JointAcc << "第一軸   " << sacc1 << "   第二軸  " << sacc2 << "   第三軸  " << sacc3 << "   第四軸  " << sacc4 << "   第五軸  " << sacc5 << endl;
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸  " << stau5 << endl;
					//CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;

					if (GetAsyncKeyState(0x57))	//w
					{
						return 0;
						break;
					}
				}


			}
#pragma endregion ccc==0

#pragma region ccc==1
			if (ccc == 1)
			{
				start = clock();
				while (1)
				{

#pragma region 位置控制拿數據test

					desiredJointAngle.angle = 0.4118 * radian;
					myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

					desiredJointAngle.angle = 0.73 * radian;
					myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

					desiredJointAngle.angle = -0.016 * radian;
					myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

					desiredJointAngle.angle = 0.0223 * radian;
					myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

					desiredJointAngle.angle = 1.28529 * radian;
					myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

					start_time = clock();

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();
					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					temp1 = svel1;
					temp2 = svel2;
					temp3 = svel3;
					temp4 = svel4;
					temp5 = svel5;
					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣



					for (int i = 0; i < 60; i++)
					{
						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

						//sang1 = sang1*rad2deg;
						//sang2 = sang2*rad2deg;
						//sang3 = sang3*rad2deg;
						//sang4 = sang4*rad2deg;
						//sang5 = sang5*rad2deg;

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//svel1 = svel1*rad2deg;
						//svel2 = svel2*rad2deg;
						//svel3 = svel3*rad2deg;
						//svel4 = svel4*rad2deg;
						//svel5 = svel5*rad2deg;


						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣





						ang_matrix.at<double>(0, 0) = sang1_cal;
						ang_matrix.at<double>(1, 0) = sang2_cal;
						ang_matrix.at<double>(2, 0) = sang3_cal;
						ang_matrix.at<double>(3, 0) = sang4_cal;
						ang_matrix.at<double>(4, 0) = sang5_cal;
						vel_matrix.at<double>(0, 0) = svel1;
						vel_matrix.at<double>(1, 0) = svel2;
						vel_matrix.at<double>(2, 0) = svel3;
						vel_matrix.at<double>(3, 0) = svel4;
						vel_matrix.at<double>(4, 0) = svel5;

						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
#pragma region 科氏力矩陣

						C_matrix.at<double>(0, 0) = (30399.0*svel2*c2_234) / 125000000.0 - (30399.0*svel3*c2_234) / 125000000.0 + (30399.0*svel4*c2_234) / 125000000.0 + (1864633.0*svel2*s2_2) / 50000000.0 + (2259.0*svel5*s2_5) / 4000000000.0 + (31441279.0*svel2*s2_234) / 4000000000.0 - (31441279.0*svel3*s2_234) / 4000000000.0 + (31441279.0*svel4*s2_234) / 4000000000.0 - (23343.0*svel2*c_22345) / 400000000.0 - (20331.0*svel2*c_222345) / 400000000.0 + (23343.0*svel3*c_22345) / 800000000.0 + (20331.0*svel3*c_222345) / 400000000.0 - (23343.0*svel4*c_22345) / 800000000.0 - (20331.0*svel4*c_222345) / 800000000.0 - (23343.0*svel5*c_22345) / 800000000.0 - (20331.0*svel5*c_222345) / 800000000.0 - (24849.0*svel2*s_234_5) / 2000000000.0 + (24849.0*svel3*s_234_5) / 2000000000.0 - (24849.0*svel4*s_234_5) / 2000000000.0 + (24849.0*svel5*s_234_5) / 2000000000.0 + (2536699.0*svel2*s_223) / 50000000.0 + (10013911.0*svel2*s2_23) / 500000000.0 - (2536699.0*svel3*s_223) / 100000000.0 - (10013911.0*svel3*s2_23) / 500000000.0 - (4851957.0*svel2*c_234) / 1000000000.0 + (4851957.0*svel3*c_234) / 1000000000.0 - (4851957.0*svel4*c_234) / 1000000000.0 - (23343.0*svel3*c_345) / 800000000.0 + (23343.0*svel4*c_345) / 800000000.0 - (23343.0*svel5*c_345) / 800000000.0 + (33033.0*svel2*s_234) / 500000000.0 - (33033.0*svel3*s_234) / 500000000.0 + (33033.0*svel4*s_234) / 500000000.0 + (20331.0*svel4*c_45) / 800000000.0 + (20331.0*svel5*c_45) / 800000000.0 - (23343.0*svel2*c_2234_5) / 400000000.0 - (20331.0*svel2*c_22234_5) / 400000000.0 + (23343.0*svel3*c_2234_5) / 800000000.0 - (2259.0*svel2*c_2223245) / 62500000.0 + (20331.0*svel3*c_22234_5) / 400000000.0 - (23343.0*svel4*c_2234_5) / 800000000.0 + (2259.0*svel3*c_2223245) / 62500000.0 - (20331.0*svel4*c_22234_5) / 800000000.0 + (23343.0*svel5*c_2234_5) / 800000000.0 - (2259.0*svel4*c_2223245) / 62500000.0 + (20331.0*svel5*c_22234_5) / 800000000.0 - (2259.0*svel5*c_2223245) / 125000000.0 + (753.0*svel2*s_2223245) / 1000000000.0 - (753.0*svel3*s_2223245) / 1000000000.0 + (753.0*svel4*s_2223245) / 1000000000.0 + (753.0*svel5*s_2223245) / 2000000000.0 - (347127.0*svel2*c_2) / 20000000.0 - (27027.0*svel4*c_4) / 200000000.0 + (31031.0*svel2*c_2234) / 100000000.0 + (27027.0*svel2*c_22234) / 100000000.0 - (31031.0*svel3*c_2234) / 200000000.0 - (27027.0*svel3*c_22234) / 100000000.0 + (31031.0*svel4*c_2234) / 200000000.0 - (23343.0*svel3*c_34_5) / 800000000.0 + (27027.0*svel4*c_22234) / 200000000.0 + (23343.0*svel4*c_34_5) / 800000000.0 + (23343.0*svel5*c_34_5) / 800000000.0 - (2536699.0*svel3*s_3) / 100000000.0 - (3969783.0*svel4*s_4) / 400000000.0 + (753.0*svel5*s_5) / 1000000000.0 + (4557899.0*svel2*s_2234) / 200000000.0 + (3969783.0*svel2*s_22234) / 200000000.0 - (4557899.0*svel3*s_2234) / 400000000.0 - (3969783.0*svel3*s_22234) / 200000000.0 + (4557899.0*svel4*s_2234) / 400000000.0 + (3969783.0*svel4*s_22234) / 400000000.0 - (24849.0*svel2*s_2345) / 2000000000.0 + (24849.0*svel3*s_2345) / 2000000000.0 - (24849.0*svel4*s_2345) / 2000000000.0 - (24849.0*svel5*s_2345) / 2000000000.0 - (2700357.0*svel2*c_23) / 250000000.0 + (2700357.0*svel3*c_23) / 250000000.0 + (31031.0*svel3*c_34) / 200000000.0 - (31031.0*svel4*c_34) / 200000000.0 + (20331.0*svel4*c_4_5) / 800000000.0 - (20331.0*svel5*c_4_5) / 800000000.0 - (2259.0*svel2*c_222324_5) / 62500000.0 + (2259.0*svel3*c_222324_5) / 62500000.0 - (2259.0*svel4*c_222324_5) / 62500000.0 + (2259.0*svel5*c_222324_5) / 125000000.0 - (4557899.0*svel3*s_34) / 400000000.0 + (4557899.0*svel4*s_34) / 400000000.0 + (753.0*svel2*s_222324_5) / 1000000000.0 - (2259.0*svel2*s2_234_5) / 8000000000.0 - (2259.0*svel2*s2_2345) / 8000000000.0 - (753.0*svel3*s_222324_5) / 1000000000.0 + (2259.0*svel3*s2_234_5) / 8000000000.0 + (2259.0*svel3*s2_2345) / 8000000000.0 + (753.0*svel4*s_222324_5) / 1000000000.0 - (2259.0*svel4*s2_234_5) / 8000000000.0 - (2259.0*svel4*s2_2345) / 8000000000.0 - (753.0*svel5*s_222324_5) / 2000000000.0 + (2259.0*svel5*s2_234_5) / 8000000000.0 - (2259.0*svel5*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 1) = (30399.0*svel1*c2_234) / 125000000.0 + (1864633.0*svel1*s2_2) / 50000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 400000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 400000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 + (2536699.0*svel1*s_223) / 50000000.0 + (10013911.0*svel1*s2_23) / 500000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (20331.0*svel2*c_235) / 400000000.0 - (20331.0*svel3*c_235) / 400000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel2*c_25) / 400000000.0 - (23343.0*svel1*c_2234_5) / 400000000.0 - (20331.0*svel1*c_22234_5) / 400000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (347127.0*svel1*c_2) / 20000000.0 + (31031.0*svel1*c_2234) / 100000000.0 + (27027.0*svel1*c_22234) / 100000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 + (18867.0*svel2*s_2) / 100000000.0 + (4557899.0*svel1*s_2234) / 200000000.0 + (3969783.0*svel1*s_22234) / 200000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (2700357.0*svel1*c_23) / 250000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 - (441029.0*svel2*s_23) / 500000000.0 + (441029.0*svel3*s_23) / 500000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 2) = (23343.0*svel1*c_22345) / 800000000.0 - (31441279.0*svel1*s2_234) / 4000000000.0 - (30399.0*svel1*c2_234) / 125000000.0 + (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 400000000.0 - (2259.0*svel3*c_234_5) / 62500000.0 + (2259.0*svel4*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 - (753.0*svel2*s_234_5) / 1000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 + (753.0*svel3*s_234_5) / 1000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 - (753.0*svel4*s_234_5) / 1000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (2259.0*svel5*s_234_25) / 4000000000.0 - (2259.0*svel5*s_23425) / 4000000000.0 - (2536699.0*svel1*s_223) / 100000000.0 - (10013911.0*svel1*s2_23) / 500000000.0 + (4851957.0*svel1*c_234) / 1000000000.0 - (20331.0*svel2*c_235) / 400000000.0 - (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (33033.0*svel1*s_234) / 500000000.0 - (25433.0*svel2*s_234) / 100000000.0 + (25433.0*svel3*s_234) / 100000000.0 - (25433.0*svel4*s_234) / 100000000.0 + (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 400000000.0 + (2259.0*svel1*c_2223245) / 62500000.0 - (753.0*svel1*s_2223245) / 1000000000.0 - (31031.0*svel1*c_2234) / 200000000.0 - (27027.0*svel1*c_22234) / 100000000.0 + (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 - (20331.0*svel3*c_23_5) / 400000000.0 - (2536699.0*svel1*s_3) / 100000000.0 - (4557899.0*svel1*s_2234) / 400000000.0 - (3969783.0*svel1*s_22234) / 200000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 + (2700357.0*svel1*c_23) / 250000000.0 + (31031.0*svel1*c_34) / 200000000.0 + (2259.0*svel1*c_222324_5) / 62500000.0 + (441029.0*svel2*s_23) / 500000000.0 - (4557899.0*svel1*s_34) / 400000000.0 - (441029.0*svel3*s_23) / 500000000.0 - (753.0*svel1*s_222324_5) / 1000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 3) = (30399.0*svel1*c2_234) / 125000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (27027.0*svel1*c_4) / 200000000.0 + (31031.0*svel1*c_2234) / 200000000.0 + (27027.0*svel1*c_22234) / 200000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 - (3969783.0*svel1*s_4) / 400000000.0 + (4557899.0*svel1*s_2234) / 400000000.0 + (3969783.0*svel1*s_22234) / 400000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (31031.0*svel1*c_34) / 200000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 + (4557899.0*svel1*s_34) / 400000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 4) = (2259.0*svel1*s2_5) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel5*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 - (753.0*svel5*s_234_5) / 1000000000.0 - (23343.0*svel1*c_345) / 800000000.0 - (20331.0*svel5*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel5*c_25) / 400000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 125000000.0 + (753.0*svel1*s_2223245) / 2000000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel5*c_23_5) / 400000000.0 + (753.0*svel1*s_5) / 1000000000.0 - (24849.0*svel5*s_5) / 1000000000.0 - (2259.0*svel5*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel5*s_2345) / 1000000000.0 - (20331.0*svel1*c_4_5) / 800000000.0 + (23343.0*svel5*c_2_5) / 400000000.0 + (2259.0*svel1*c_222324_5) / 125000000.0 - (753.0*svel1*s_222324_5) / 2000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(1, 0) = (svel1*((24849.0*s_2345) / 1000000000.0 + (2700357.0*c_23) / 125000000.0 + (2259.0*c_222324_5) / 31250000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (1864633.0*s2_2) / 25000000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 200000000.0 + (20331.0*c_222345) / 200000000.0 + (24849.0*s_234_5) / 1000000000.0 - (2536699.0*s_223) / 25000000.0 - (10013911.0*s2_23) / 250000000.0 + (4851957.0*c_234) / 500000000.0 - (33033.0*s_234) / 250000000.0 + (23343.0*c_2234_5) / 200000000.0 + (20331.0*c_22234_5) / 200000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (347127.0*c_2) / 10000000.0 - (31031.0*c_2234) / 50000000.0 - (27027.0*c_22234) / 50000000.0 - (4557899.0*s_2234) / 100000000.0 - (3969783.0*s_22234) / 100000000)) / 2 + (svel5*((23343.0*c_2_5) / 200000000.0 - (753.0*s_2345) / 500000000.0 + (2259.0*c_234_5) / 31250000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (23343.0*c_25) / 200000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(1, 1) = (svel4*((20331.0*c_4_5) / 200000000.0 - (31031.0*c_34) / 50000000.0 + (4557899.0*s_34) / 100000000.0 + (23343.0*c_345) / 200000000.0 + (20331.0*c_45) / 200000000.0 - (27027.0*c_4) / 50000000.0 + (23343.0*c_34_5) / 200000000.0 - (3969783.0*s_4) / 100000000)) / 2 - (svel3*((4557899.0*s_34) / 100000000.0 - (31031.0*c_34) / 50000000.0 + (23343.0*c_345) / 200000000.0 + (23343.0*c_34_5) / 200000000.0 + (2536699.0*s_3) / 25000000)) / 2 - (svel5*((2259.0*s2_5) / 1000000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000.0 + (23343.0*c_3*s_4*s_5) / 100000000.0 - (23343.0*c_4*s_3*s_5) / 100000000)) / 2;
						C_matrix.at<double>(1, 2) = (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 + (27027.0*svel4*c_4) / 100000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 + (23343.0*svel3*c_34_5) / 400000000.0 - (23343.0*svel4*c_34_5) / 400000000.0 - (23343.0*svel5*c_34_5) / 400000000.0 - (2536699.0*svel2*s_3) / 50000000.0 + (2536699.0*svel3*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_34) / 100000000.0 - (31031.0*svel3*c_34) / 100000000.0 + (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0 - (4557899.0*svel2*s_34) / 200000000.0 + (4557899.0*svel3*s_34) / 200000000.0 - (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 3) = (23343.0*svel2*c_345) / 400000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel3*c_345) / 400000000.0 + (23343.0*svel4*c_345) / 400000000.0 - (23343.0*svel5*c_345) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 - (27027.0*svel2*c_4) / 100000000.0 + (27027.0*svel3*c_4) / 100000000.0 - (27027.0*svel4*c_4) / 100000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 - (3969783.0*svel2*s_4) / 200000000.0 + (3969783.0*svel3*s_4) / 200000000.0 - (3969783.0*svel4*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_34) / 100000000.0 + (31031.0*svel3*c_34) / 100000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0 + (4557899.0*svel2*s_34) / 200000000.0 - (4557899.0*svel3*s_34) / 200000000.0 + (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (23343.0*svel1*c_25) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 + (23343.0*svel1*c_2_5) / 400000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(2, 0) = (svel1*((4557899.0*s_34) / 200000000.0 - (2700357.0*c_23) / 125000000.0 - (31031.0*c_34) / 100000000.0 - (2259.0*c_222324_5) / 31250000.0 - (24849.0*s_2345) / 1000000000.0 + (753.0*s_222324_5) / 500000000.0 - (2259.0*s2_234_5) / 4000000000.0 - (2259.0*s2_2345) / 4000000000.0 + (30399.0*c2_234) / 62500000.0 + (31441279.0*s2_234) / 2000000000.0 - (23343.0*c_22345) / 400000000.0 - (20331.0*c_222345) / 200000000.0 - (24849.0*s_234_5) / 1000000000.0 + (2536699.0*s_223) / 50000000.0 + (10013911.0*s2_23) / 250000000.0 - (4851957.0*c_234) / 500000000.0 + (23343.0*c_345) / 400000000.0 + (33033.0*s_234) / 250000000.0 - (23343.0*c_2234_5) / 400000000.0 - (20331.0*c_22234_5) / 200000000.0 - (2259.0*c_2223245) / 31250000.0 + (753.0*s_2223245) / 500000000.0 + (31031.0*c_2234) / 100000000.0 + (27027.0*c_22234) / 50000000.0 + (23343.0*c_34_5) / 400000000.0 + (2536699.0*s_3) / 50000000.0 + (4557899.0*s_2234) / 200000000.0 + (3969783.0*s_22234) / 100000000)) / 2 - (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(2, 1) = (2259.0*svel5*s2_5) / 2000000000.0 + (27027.0*svel4*c_4) / 100000000.0 + (2536699.0*svel2*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel4*c_4*c_5) / 200000000.0 - (4557899.0*svel2*c_3*s_4) / 200000000.0 + (4557899.0*svel2*c_4*s_3) / 200000000.0 - (31031.0*svel2*s_3*s_4) / 100000000.0 + (20331.0*svel5*s_4*s_5) / 200000000.0 + (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 + (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(2, 2) = -(svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 3) = (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 4) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 - (2259.0*svel5*c_5) / 31250000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(3, 0) = (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20002259.0*s_234) / 1000000000.0 + (2259.0*c_2345) / 31250000)) / 2 + (svel1*((24849.0*s_2345) / 1000000000.0 + (31031.0*c_34) / 100000000.0 - (20331.0*c_4_5) / 400000000.0 + (2259.0*c_222324_5) / 31250000.0 - (4557899.0*s_34) / 200000000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 400000000.0 + (20331.0*c_222345) / 400000000.0 + (24849.0*s_234_5) / 1000000000.0 + (4851957.0*c_234) / 500000000.0 - (23343.0*c_345) / 400000000.0 - (33033.0*s_234) / 250000000.0 - (20331.0*c_45) / 400000000.0 + (23343.0*c_2234_5) / 400000000.0 + (20331.0*c_22234_5) / 400000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (27027.0*c_4) / 100000000.0 - (31031.0*c_2234) / 100000000.0 - (27027.0*c_22234) / 100000000.0 - (23343.0*c_34_5) / 400000000.0 + (3969783.0*s_4) / 200000000.0 - (4557899.0*s_2234) / 200000000.0 - (3969783.0*s_22234) / 200000000)) / 2;
						C_matrix.at<double>(3, 1) = (27027.0*svel2*c_4) / 100000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (27027.0*svel3*c_4) / 100000000.0 + (3969783.0*svel2*s_4) / 200000000.0 - (3969783.0*svel3*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel2*c_4*c_5) / 200000000.0 + (20331.0*svel3*c_4*c_5) / 200000000.0 + (4557899.0*svel2*c_3*s_4) / 200000000.0 - (4557899.0*svel2*c_4*s_3) / 200000000.0 + (31031.0*svel2*s_3*s_4) / 100000000.0 - (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 - (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(3, 2) = (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((753.0*s_5) / 250000000.0 - (2259.0*c_5*s_5) / 500000000)) / 2;
						C_matrix.at<double>(3, 3) = -(753.0*svel5*(3.0*s2_5 - 4.0*s_5)) / 2000000000.0;
						C_matrix.at<double>(3, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 0) = (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel1*s2_5) / 4000000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (20331.0*svel2*c_235) / 400000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 - (23343.0*svel2*c_25) / 400000000.0 - (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 + (2259.0*svel1*c_2223245) / 125000000.0 - (753.0*svel1*s_2223245) / 2000000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 - (753.0*svel1*s_5) / 1000000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 125000000.0 + (753.0*svel1*s_222324_5) / 2000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(4, 1) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 + (23343.0*svel2*c_345) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (23343.0*svel1*c_25) / 400000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 - (23343.0*svel1*c_2_5) / 400000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 2) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 3) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 4) = 0.0;
#pragma endregion 科氏力矩陣
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////
#pragma region 重力矩陣


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-1;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;

#pragma endregion 重力矩陣
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////

						/////////////////////////////////////////////慣性矩陣/////////////////////////////
#pragma region 慣性矩陣

						M_matrix.at<double>(0, 0) = (4557899.0*cos(sang3_cal - sang4_cal)) / 200000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 1000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal)) / 8000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal)) / 8000000000.0 - (2700357.0*sin(sang2_cal - sang3_cal)) / 125000000.0 + (31031.0*sin(sang3_cal - sang4_cal)) / 100000000.0 + (20331.0*sin(sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 62500000.0 - (1864633.0*cos(2 * sang2_cal)) / 50000000.0 - (2259.0*cos(2 * sang5_cal)) / 4000000000.0 - (31441279.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 4000000000.0 + (30399.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 125000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (2536699.0*cos(2 * sang2_cal - sang3_cal)) / 50000000.0 - (10013911.0*cos(2 * sang2_cal - 2 * sang3_cal)) / 500000000.0 - (33033.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 250000000.0 - (4851957.0*sin(sang2_cal - sang3_cal + sang4_cal)) / 500000000.0 - (23343.0*sin(sang3_cal - sang4_cal + sang5_cal)) / 400000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 1000000000.0 + (20331.0*sin(sang4_cal + sang5_cal)) / 400000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 62500000.0 + (2536699.0*c_3) / 50000000.0 + term_3 / 200000000.0 - term_5 / 500000000.0 - (4557899.0*cos(2 * sang2_cal - sang3_cal + sang4_cal)) / 200000000.0 - (3969783.0*cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 200000000.0 - (347127.0*s_2) / 10000000.0 - term_6 / 100000000.0 + (31031.0*sin(2 * sang2_cal - sang3_cal + sang4_cal)) / 100000000.0 + (27027.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 100000000.0 - (23343.0*sin(sang3_cal - sang4_cal - sang5_cal)) / 400000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 1000000000.0 + 105207843 / 800000000.0;
						M_matrix.at<double>(0, 1) = term_1 + (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_16 / 400000000.0 + (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 - (18867.0*c_2) / 100000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 2) = term_2 - (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - term_1 - term_13 / 4000000000.0 + term_14 / 4000000000.0 + term_15 / 62500000.0 + term_17 / 100000000.0 - term_16 / 400000000.0 + (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(0, 3) = term_1 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 4) = (20002259.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 1000000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 - term_15 / 62500000.0 - term_1 - term_16 / 400000000.0 - (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 + (24849.0*c_5) / 1000000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(1, 0) = M_matrix.at<double>(0, 1);
						M_matrix.at<double>(1, 1) = (2536699.0*c_3) / 25000000.0 + term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_9 / 100000000.0 - term_10 / 50000000.0 + term_12 / 50000000.0 + term_7 / 100000000.0 + term_8 / 100000000.0 + term_4 + term_18 / 100000000.0 - term_19 / 100000000.0 + 180370741 / 1000000000.0;
						M_matrix.at<double>(1, 2) = term_5 / 250000000.0 - term_3 / 100000000.0 - (2536699.0*c_3) / 50000000.0 + term_6 / 50000000.0 - term_9 / 200000000.0 + term_10 / 100000000.0 - term_12 / 100000000.0 - term_7 / 100000000.0 - term_8 / 200000000.0 - term_4 - term_18 / 200000000.0 + term_19 / 200000000.0 - 95785421 / 1000000000.0;
						M_matrix.at<double>(1, 3) = term_3 / 200000000.0 - term_5 / 250000000.0 - term_6 / 100000000.0 + term_9 / 200000000.0 - term_10 / 100000000.0 + term_12 / 100000000.0 + term_7 / 200000000.0 + term_8 / 200000000.0 + term_4 + term_18 / 200000000.0 - term_19 / 200000000.0 + 45729777 / 1000000000.0;
						M_matrix.at<double>(1, 4) = (753.0*s_5*(155.0*cos(sang3_cal - sang4_cal) + 135.0*c_4 + 96)) / 1000000000.0;
						M_matrix.at<double>(2, 0) = M_matrix.at<double>(0, 2);
						M_matrix.at<double>(2, 1) = M_matrix.at<double>(1, 2);
						M_matrix.at<double>(2, 2) = term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_7 / 100000000.0 + term_4 + 95785421 / 1000000000.0;
						M_matrix.at<double>(2, 3) = term_5 / 250000000.0 - term_3 / 200000000.0 + term_6 / 100000000.0 - term_7 / 200000000.0 - term_4 - 45729777 / 1000000000.0;
						M_matrix.at<double>(2, 4) = -(2259.0*s_5*(45.0*c_4 + 32)) / 1000000000.0;
						M_matrix.at<double>(3, 0) = M_matrix.at<double>(0, 3);
						M_matrix.at<double>(3, 1) = M_matrix.at<double>(1, 3);
						M_matrix.at<double>(3, 2) = M_matrix.at<double>(2, 3);
						M_matrix.at<double>(3, 3) = term_4 - term_5 / 250000000.0 + 45729777.0 / 1000000000.0;
						M_matrix.at<double>(3, 4) = (2259.0*s_5) / 31250000.0;
						M_matrix.at<double>(4, 0) = M_matrix.at<double>(0, 4);
						M_matrix.at<double>(4, 1) = M_matrix.at<double>(1, 4);
						M_matrix.at<double>(4, 2) = M_matrix.at<double>(2, 4);
						M_matrix.at<double>(4, 3) = M_matrix.at<double>(3, 4);
						M_matrix.at<double>(4, 4) = 20002259.0 / 1000000000.0;
#pragma endregion 慣性矩陣
						/////////////////////////////////////////////慣性矩陣///////////////////////////////////////////////////	
					}

					end_time = clock();
					total_time = (long double)(end_time - start_time) / CLOCKS_PER_SEC;
					nowtime << total_time << endl;
					sacc1 = -(svel1 - temp1) / total_time;
					sacc2 = -(svel2 - temp2) / total_time;
					sacc3 = -(svel3 - temp3) / total_time;
					sacc4 = -(svel4 - temp4) / total_time;
					sacc5 = -(svel5 - temp5) / total_time;

					acc_matrix.at<double>(0, 0) = sacc1;
					acc_matrix.at<double>(1, 0) = sacc2;
					acc_matrix.at<double>(2, 0) = sacc3;
					acc_matrix.at<double>(3, 0) = sacc4;
					acc_matrix.at<double>(4, 0) = sacc5;

					imgtemp_torque.t1 = stau1;
					imgtemp_torque.t2 = stau2;
					imgtemp_torque.t3 = stau3;
					imgtemp_torque.t4 = stau4;
					imgtemp_torque.t5 = stau5;
					imgtest_torque.push_back(imgtemp_torque);

					/////////////////////////////////////////////力矩矩陣///////////////////////////////////////////////////
					tau_matrix = M_matrix*acc_matrix + C_matrix*vel_matrix + N_matrix;
					nowx << "第一軸   " << sang1 << "   第二軸  " << sang2 << "   第三軸  " << sang3 << "   第四軸  " << sang4 << "   第五軸" << sang5 << endl;
					tempvel << "第一軸  " << svel1 - temp1 << "  第二軸  " << svel2 - temp2 << "  第三軸  " << svel3 - temp3 << "  第四軸  " << svel4 - temp4 << "  第五軸  " << svel5 - temp5 << endl;
					JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸" << svel5 << endl;
					Velocity1 << svel1 << endl;			Velocity2 << svel2 << endl;			Velocity3 << svel3 << endl;			Velocity4 << svel4 << endl;			Velocity5 << svel5 << endl;
					Acc1 << sacc1 << endl;			Acc2 << sacc2 << endl;			Acc3 << sacc3 << endl;			Acc4 << sacc4 << endl;			Acc5 << sacc5 << endl;
					JointAcc << "第一軸   " << sacc1 << "   第二軸  " << sacc2 << "   第三軸  " << sacc3 << "   第四軸  " << sacc4 << "   第五軸  " << sacc5 << endl;
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸  " << stau5 << endl;
					CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
					if (abs(sang1 - 0.4118) < 0.01 && abs(sang2 - 0.435221) < 0.01 && abs(sang3 + 0.730734) < 0.01 && abs(sang4 - 0.692898) < 0.01	&& abs(sang5 - 1.28529) < 0.01)//w
					{

						stop = clock();
						ttime = (long double)(stop - start) / CLOCKS_PER_SEC;
						toatime << ttime;

						break;

					}
				}
			}
#pragma endregion 位置控制拿數據test
#pragma endregion ccc==1

#pragma region ccc==2

			if (ccc == 2)
			{

				/*	desiredJointAngle.angle = 2.56244 * radian;
				myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.1345 * radian;
				myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				desiredJointAngle.angle = -2.5482 * radian;
				myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.789 * radian;
				myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				desiredJointAngle.angle = 2.9234 * radian;
				myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);*/

				desiredJointAngle.angle = 0.0109 * radian;
				myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.73* radian;
				myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				desiredJointAngle.angle = -0.9 * radian;
				myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.0 * radian;//0.0223
				myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.1201 * radian;
				myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

				cin >> ttime;
				while (1)
				{
					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();

					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣

					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣


					ang_matrix.at<double>(0, 0) = sang1_cal;
					ang_matrix.at<double>(1, 0) = sang2_cal;
					ang_matrix.at<double>(2, 0) = sang3_cal;
					ang_matrix.at<double>(3, 0) = sang4_cal;
					ang_matrix.at<double>(4, 0) = sang5_cal;
					vel_matrix.at<double>(0, 0) = svel1;
					vel_matrix.at<double>(1, 0) = svel2;
					vel_matrix.at<double>(2, 0) = svel3;
					vel_matrix.at<double>(3, 0) = svel4;
					vel_matrix.at<double>(4, 0) = svel5;
					//desiredJointVelocity.angularVelocity = 0.5 * radian_per_second;
					//myYouBotManipulator->getArmJoint(1).setData(desiredJointVelocity);

					//desiredJointTorque.torque = -2.4 * newton_meter;
					//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					//LOG(info) << "torque test1";
					//desiredJointTorque.torque = -1.2 * newton_meter;
					//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//LOG(info) << "torque test1";
					start_time = clock();
					if (sang3_cal > -1.2)//第三軸保護措施 做實驗前須檢查是否符合自己所需
					{
						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-1;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						tau_matrix = N_matrix;
#pragma endregion 重力補償test



						desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						LOG(info) << "torque test1";
						desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						LOG(info) << "torque test2";
						desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						LOG(info) << "torque test3";
						desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						LOG(info) << "torque test4";
						desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						LOG(info) << "torque test5";

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣

					}
					for (int i = 0; i < 60; i++)
					{
						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

						//sang1 = sang1*rad2deg;
						//sang2 = sang2*rad2deg;
						//sang3 = sang3*rad2deg;
						//sang4 = sang4*rad2deg;
						//sang5 = sang5*rad2deg;

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//svel1 = svel1*rad2deg;
						//svel2 = svel2*rad2deg;
						//svel3 = svel3*rad2deg;
						//svel4 = svel4*rad2deg;
						//svel5 = svel5*rad2deg;


						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣





						ang_matrix.at<double>(0, 0) = sang1_cal;
						ang_matrix.at<double>(1, 0) = sang2_cal;
						ang_matrix.at<double>(2, 0) = sang3_cal;
						ang_matrix.at<double>(3, 0) = sang4_cal;
						ang_matrix.at<double>(4, 0) = sang5_cal;
						vel_matrix.at<double>(0, 0) = svel1;
						vel_matrix.at<double>(1, 0) = svel2;
						vel_matrix.at<double>(2, 0) = svel3;
						vel_matrix.at<double>(3, 0) = svel4;
						vel_matrix.at<double>(4, 0) = svel5;

						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
#pragma region 科氏力矩陣

						C_matrix.at<double>(0, 0) = (30399.0*svel2*c2_234) / 125000000.0 - (30399.0*svel3*c2_234) / 125000000.0 + (30399.0*svel4*c2_234) / 125000000.0 + (1864633.0*svel2*s2_2) / 50000000.0 + (2259.0*svel5*s2_5) / 4000000000.0 + (31441279.0*svel2*s2_234) / 4000000000.0 - (31441279.0*svel3*s2_234) / 4000000000.0 + (31441279.0*svel4*s2_234) / 4000000000.0 - (23343.0*svel2*c_22345) / 400000000.0 - (20331.0*svel2*c_222345) / 400000000.0 + (23343.0*svel3*c_22345) / 800000000.0 + (20331.0*svel3*c_222345) / 400000000.0 - (23343.0*svel4*c_22345) / 800000000.0 - (20331.0*svel4*c_222345) / 800000000.0 - (23343.0*svel5*c_22345) / 800000000.0 - (20331.0*svel5*c_222345) / 800000000.0 - (24849.0*svel2*s_234_5) / 2000000000.0 + (24849.0*svel3*s_234_5) / 2000000000.0 - (24849.0*svel4*s_234_5) / 2000000000.0 + (24849.0*svel5*s_234_5) / 2000000000.0 + (2536699.0*svel2*s_223) / 50000000.0 + (10013911.0*svel2*s2_23) / 500000000.0 - (2536699.0*svel3*s_223) / 100000000.0 - (10013911.0*svel3*s2_23) / 500000000.0 - (4851957.0*svel2*c_234) / 1000000000.0 + (4851957.0*svel3*c_234) / 1000000000.0 - (4851957.0*svel4*c_234) / 1000000000.0 - (23343.0*svel3*c_345) / 800000000.0 + (23343.0*svel4*c_345) / 800000000.0 - (23343.0*svel5*c_345) / 800000000.0 + (33033.0*svel2*s_234) / 500000000.0 - (33033.0*svel3*s_234) / 500000000.0 + (33033.0*svel4*s_234) / 500000000.0 + (20331.0*svel4*c_45) / 800000000.0 + (20331.0*svel5*c_45) / 800000000.0 - (23343.0*svel2*c_2234_5) / 400000000.0 - (20331.0*svel2*c_22234_5) / 400000000.0 + (23343.0*svel3*c_2234_5) / 800000000.0 - (2259.0*svel2*c_2223245) / 62500000.0 + (20331.0*svel3*c_22234_5) / 400000000.0 - (23343.0*svel4*c_2234_5) / 800000000.0 + (2259.0*svel3*c_2223245) / 62500000.0 - (20331.0*svel4*c_22234_5) / 800000000.0 + (23343.0*svel5*c_2234_5) / 800000000.0 - (2259.0*svel4*c_2223245) / 62500000.0 + (20331.0*svel5*c_22234_5) / 800000000.0 - (2259.0*svel5*c_2223245) / 125000000.0 + (753.0*svel2*s_2223245) / 1000000000.0 - (753.0*svel3*s_2223245) / 1000000000.0 + (753.0*svel4*s_2223245) / 1000000000.0 + (753.0*svel5*s_2223245) / 2000000000.0 - (347127.0*svel2*c_2) / 20000000.0 - (27027.0*svel4*c_4) / 200000000.0 + (31031.0*svel2*c_2234) / 100000000.0 + (27027.0*svel2*c_22234) / 100000000.0 - (31031.0*svel3*c_2234) / 200000000.0 - (27027.0*svel3*c_22234) / 100000000.0 + (31031.0*svel4*c_2234) / 200000000.0 - (23343.0*svel3*c_34_5) / 800000000.0 + (27027.0*svel4*c_22234) / 200000000.0 + (23343.0*svel4*c_34_5) / 800000000.0 + (23343.0*svel5*c_34_5) / 800000000.0 - (2536699.0*svel3*s_3) / 100000000.0 - (3969783.0*svel4*s_4) / 400000000.0 + (753.0*svel5*s_5) / 1000000000.0 + (4557899.0*svel2*s_2234) / 200000000.0 + (3969783.0*svel2*s_22234) / 200000000.0 - (4557899.0*svel3*s_2234) / 400000000.0 - (3969783.0*svel3*s_22234) / 200000000.0 + (4557899.0*svel4*s_2234) / 400000000.0 + (3969783.0*svel4*s_22234) / 400000000.0 - (24849.0*svel2*s_2345) / 2000000000.0 + (24849.0*svel3*s_2345) / 2000000000.0 - (24849.0*svel4*s_2345) / 2000000000.0 - (24849.0*svel5*s_2345) / 2000000000.0 - (2700357.0*svel2*c_23) / 250000000.0 + (2700357.0*svel3*c_23) / 250000000.0 + (31031.0*svel3*c_34) / 200000000.0 - (31031.0*svel4*c_34) / 200000000.0 + (20331.0*svel4*c_4_5) / 800000000.0 - (20331.0*svel5*c_4_5) / 800000000.0 - (2259.0*svel2*c_222324_5) / 62500000.0 + (2259.0*svel3*c_222324_5) / 62500000.0 - (2259.0*svel4*c_222324_5) / 62500000.0 + (2259.0*svel5*c_222324_5) / 125000000.0 - (4557899.0*svel3*s_34) / 400000000.0 + (4557899.0*svel4*s_34) / 400000000.0 + (753.0*svel2*s_222324_5) / 1000000000.0 - (2259.0*svel2*s2_234_5) / 8000000000.0 - (2259.0*svel2*s2_2345) / 8000000000.0 - (753.0*svel3*s_222324_5) / 1000000000.0 + (2259.0*svel3*s2_234_5) / 8000000000.0 + (2259.0*svel3*s2_2345) / 8000000000.0 + (753.0*svel4*s_222324_5) / 1000000000.0 - (2259.0*svel4*s2_234_5) / 8000000000.0 - (2259.0*svel4*s2_2345) / 8000000000.0 - (753.0*svel5*s_222324_5) / 2000000000.0 + (2259.0*svel5*s2_234_5) / 8000000000.0 - (2259.0*svel5*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 1) = (30399.0*svel1*c2_234) / 125000000.0 + (1864633.0*svel1*s2_2) / 50000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 400000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 400000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 + (2536699.0*svel1*s_223) / 50000000.0 + (10013911.0*svel1*s2_23) / 500000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (20331.0*svel2*c_235) / 400000000.0 - (20331.0*svel3*c_235) / 400000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel2*c_25) / 400000000.0 - (23343.0*svel1*c_2234_5) / 400000000.0 - (20331.0*svel1*c_22234_5) / 400000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (347127.0*svel1*c_2) / 20000000.0 + (31031.0*svel1*c_2234) / 100000000.0 + (27027.0*svel1*c_22234) / 100000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 + (18867.0*svel2*s_2) / 100000000.0 + (4557899.0*svel1*s_2234) / 200000000.0 + (3969783.0*svel1*s_22234) / 200000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (2700357.0*svel1*c_23) / 250000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 - (441029.0*svel2*s_23) / 500000000.0 + (441029.0*svel3*s_23) / 500000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 2) = (23343.0*svel1*c_22345) / 800000000.0 - (31441279.0*svel1*s2_234) / 4000000000.0 - (30399.0*svel1*c2_234) / 125000000.0 + (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 400000000.0 - (2259.0*svel3*c_234_5) / 62500000.0 + (2259.0*svel4*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 - (753.0*svel2*s_234_5) / 1000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 + (753.0*svel3*s_234_5) / 1000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 - (753.0*svel4*s_234_5) / 1000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (2259.0*svel5*s_234_25) / 4000000000.0 - (2259.0*svel5*s_23425) / 4000000000.0 - (2536699.0*svel1*s_223) / 100000000.0 - (10013911.0*svel1*s2_23) / 500000000.0 + (4851957.0*svel1*c_234) / 1000000000.0 - (20331.0*svel2*c_235) / 400000000.0 - (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (33033.0*svel1*s_234) / 500000000.0 - (25433.0*svel2*s_234) / 100000000.0 + (25433.0*svel3*s_234) / 100000000.0 - (25433.0*svel4*s_234) / 100000000.0 + (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 400000000.0 + (2259.0*svel1*c_2223245) / 62500000.0 - (753.0*svel1*s_2223245) / 1000000000.0 - (31031.0*svel1*c_2234) / 200000000.0 - (27027.0*svel1*c_22234) / 100000000.0 + (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 - (20331.0*svel3*c_23_5) / 400000000.0 - (2536699.0*svel1*s_3) / 100000000.0 - (4557899.0*svel1*s_2234) / 400000000.0 - (3969783.0*svel1*s_22234) / 200000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 + (2700357.0*svel1*c_23) / 250000000.0 + (31031.0*svel1*c_34) / 200000000.0 + (2259.0*svel1*c_222324_5) / 62500000.0 + (441029.0*svel2*s_23) / 500000000.0 - (4557899.0*svel1*s_34) / 400000000.0 - (441029.0*svel3*s_23) / 500000000.0 - (753.0*svel1*s_222324_5) / 1000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 3) = (30399.0*svel1*c2_234) / 125000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (27027.0*svel1*c_4) / 200000000.0 + (31031.0*svel1*c_2234) / 200000000.0 + (27027.0*svel1*c_22234) / 200000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 - (3969783.0*svel1*s_4) / 400000000.0 + (4557899.0*svel1*s_2234) / 400000000.0 + (3969783.0*svel1*s_22234) / 400000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (31031.0*svel1*c_34) / 200000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 + (4557899.0*svel1*s_34) / 400000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 4) = (2259.0*svel1*s2_5) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel5*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 - (753.0*svel5*s_234_5) / 1000000000.0 - (23343.0*svel1*c_345) / 800000000.0 - (20331.0*svel5*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel5*c_25) / 400000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 125000000.0 + (753.0*svel1*s_2223245) / 2000000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel5*c_23_5) / 400000000.0 + (753.0*svel1*s_5) / 1000000000.0 - (24849.0*svel5*s_5) / 1000000000.0 - (2259.0*svel5*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel5*s_2345) / 1000000000.0 - (20331.0*svel1*c_4_5) / 800000000.0 + (23343.0*svel5*c_2_5) / 400000000.0 + (2259.0*svel1*c_222324_5) / 125000000.0 - (753.0*svel1*s_222324_5) / 2000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(1, 0) = (svel1*((24849.0*s_2345) / 1000000000.0 + (2700357.0*c_23) / 125000000.0 + (2259.0*c_222324_5) / 31250000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (1864633.0*s2_2) / 25000000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 200000000.0 + (20331.0*c_222345) / 200000000.0 + (24849.0*s_234_5) / 1000000000.0 - (2536699.0*s_223) / 25000000.0 - (10013911.0*s2_23) / 250000000.0 + (4851957.0*c_234) / 500000000.0 - (33033.0*s_234) / 250000000.0 + (23343.0*c_2234_5) / 200000000.0 + (20331.0*c_22234_5) / 200000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (347127.0*c_2) / 10000000.0 - (31031.0*c_2234) / 50000000.0 - (27027.0*c_22234) / 50000000.0 - (4557899.0*s_2234) / 100000000.0 - (3969783.0*s_22234) / 100000000)) / 2 + (svel5*((23343.0*c_2_5) / 200000000.0 - (753.0*s_2345) / 500000000.0 + (2259.0*c_234_5) / 31250000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (23343.0*c_25) / 200000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(1, 1) = (svel4*((20331.0*c_4_5) / 200000000.0 - (31031.0*c_34) / 50000000.0 + (4557899.0*s_34) / 100000000.0 + (23343.0*c_345) / 200000000.0 + (20331.0*c_45) / 200000000.0 - (27027.0*c_4) / 50000000.0 + (23343.0*c_34_5) / 200000000.0 - (3969783.0*s_4) / 100000000)) / 2 - (svel3*((4557899.0*s_34) / 100000000.0 - (31031.0*c_34) / 50000000.0 + (23343.0*c_345) / 200000000.0 + (23343.0*c_34_5) / 200000000.0 + (2536699.0*s_3) / 25000000)) / 2 - (svel5*((2259.0*s2_5) / 1000000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000.0 + (23343.0*c_3*s_4*s_5) / 100000000.0 - (23343.0*c_4*s_3*s_5) / 100000000)) / 2;
						C_matrix.at<double>(1, 2) = (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 + (27027.0*svel4*c_4) / 100000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 + (23343.0*svel3*c_34_5) / 400000000.0 - (23343.0*svel4*c_34_5) / 400000000.0 - (23343.0*svel5*c_34_5) / 400000000.0 - (2536699.0*svel2*s_3) / 50000000.0 + (2536699.0*svel3*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_34) / 100000000.0 - (31031.0*svel3*c_34) / 100000000.0 + (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0 - (4557899.0*svel2*s_34) / 200000000.0 + (4557899.0*svel3*s_34) / 200000000.0 - (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 3) = (23343.0*svel2*c_345) / 400000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel3*c_345) / 400000000.0 + (23343.0*svel4*c_345) / 400000000.0 - (23343.0*svel5*c_345) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 - (27027.0*svel2*c_4) / 100000000.0 + (27027.0*svel3*c_4) / 100000000.0 - (27027.0*svel4*c_4) / 100000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 - (3969783.0*svel2*s_4) / 200000000.0 + (3969783.0*svel3*s_4) / 200000000.0 - (3969783.0*svel4*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_34) / 100000000.0 + (31031.0*svel3*c_34) / 100000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0 + (4557899.0*svel2*s_34) / 200000000.0 - (4557899.0*svel3*s_34) / 200000000.0 + (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (23343.0*svel1*c_25) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 + (23343.0*svel1*c_2_5) / 400000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(2, 0) = (svel1*((4557899.0*s_34) / 200000000.0 - (2700357.0*c_23) / 125000000.0 - (31031.0*c_34) / 100000000.0 - (2259.0*c_222324_5) / 31250000.0 - (24849.0*s_2345) / 1000000000.0 + (753.0*s_222324_5) / 500000000.0 - (2259.0*s2_234_5) / 4000000000.0 - (2259.0*s2_2345) / 4000000000.0 + (30399.0*c2_234) / 62500000.0 + (31441279.0*s2_234) / 2000000000.0 - (23343.0*c_22345) / 400000000.0 - (20331.0*c_222345) / 200000000.0 - (24849.0*s_234_5) / 1000000000.0 + (2536699.0*s_223) / 50000000.0 + (10013911.0*s2_23) / 250000000.0 - (4851957.0*c_234) / 500000000.0 + (23343.0*c_345) / 400000000.0 + (33033.0*s_234) / 250000000.0 - (23343.0*c_2234_5) / 400000000.0 - (20331.0*c_22234_5) / 200000000.0 - (2259.0*c_2223245) / 31250000.0 + (753.0*s_2223245) / 500000000.0 + (31031.0*c_2234) / 100000000.0 + (27027.0*c_22234) / 50000000.0 + (23343.0*c_34_5) / 400000000.0 + (2536699.0*s_3) / 50000000.0 + (4557899.0*s_2234) / 200000000.0 + (3969783.0*s_22234) / 100000000)) / 2 - (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(2, 1) = (2259.0*svel5*s2_5) / 2000000000.0 + (27027.0*svel4*c_4) / 100000000.0 + (2536699.0*svel2*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel4*c_4*c_5) / 200000000.0 - (4557899.0*svel2*c_3*s_4) / 200000000.0 + (4557899.0*svel2*c_4*s_3) / 200000000.0 - (31031.0*svel2*s_3*s_4) / 100000000.0 + (20331.0*svel5*s_4*s_5) / 200000000.0 + (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 + (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(2, 2) = -(svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 3) = (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 4) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 - (2259.0*svel5*c_5) / 31250000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(3, 0) = (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20002259.0*s_234) / 1000000000.0 + (2259.0*c_2345) / 31250000)) / 2 + (svel1*((24849.0*s_2345) / 1000000000.0 + (31031.0*c_34) / 100000000.0 - (20331.0*c_4_5) / 400000000.0 + (2259.0*c_222324_5) / 31250000.0 - (4557899.0*s_34) / 200000000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 400000000.0 + (20331.0*c_222345) / 400000000.0 + (24849.0*s_234_5) / 1000000000.0 + (4851957.0*c_234) / 500000000.0 - (23343.0*c_345) / 400000000.0 - (33033.0*s_234) / 250000000.0 - (20331.0*c_45) / 400000000.0 + (23343.0*c_2234_5) / 400000000.0 + (20331.0*c_22234_5) / 400000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (27027.0*c_4) / 100000000.0 - (31031.0*c_2234) / 100000000.0 - (27027.0*c_22234) / 100000000.0 - (23343.0*c_34_5) / 400000000.0 + (3969783.0*s_4) / 200000000.0 - (4557899.0*s_2234) / 200000000.0 - (3969783.0*s_22234) / 200000000)) / 2;
						C_matrix.at<double>(3, 1) = (27027.0*svel2*c_4) / 100000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (27027.0*svel3*c_4) / 100000000.0 + (3969783.0*svel2*s_4) / 200000000.0 - (3969783.0*svel3*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel2*c_4*c_5) / 200000000.0 + (20331.0*svel3*c_4*c_5) / 200000000.0 + (4557899.0*svel2*c_3*s_4) / 200000000.0 - (4557899.0*svel2*c_4*s_3) / 200000000.0 + (31031.0*svel2*s_3*s_4) / 100000000.0 - (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 - (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(3, 2) = (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((753.0*s_5) / 250000000.0 - (2259.0*c_5*s_5) / 500000000)) / 2;
						C_matrix.at<double>(3, 3) = -(753.0*svel5*(3.0*s2_5 - 4.0*s_5)) / 2000000000.0;
						C_matrix.at<double>(3, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 0) = (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel1*s2_5) / 4000000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (20331.0*svel2*c_235) / 400000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 - (23343.0*svel2*c_25) / 400000000.0 - (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 + (2259.0*svel1*c_2223245) / 125000000.0 - (753.0*svel1*s_2223245) / 2000000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 - (753.0*svel1*s_5) / 1000000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 125000000.0 + (753.0*svel1*s_222324_5) / 2000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(4, 1) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 + (23343.0*svel2*c_345) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (23343.0*svel1*c_25) / 400000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 - (23343.0*svel1*c_2_5) / 400000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 2) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 3) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 4) = 0.0;
#pragma endregion 科氏力矩陣
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////
#pragma region 重力矩陣


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-1;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;

#pragma endregion 重力矩陣
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////

						/////////////////////////////////////////////慣性矩陣/////////////////////////////
#pragma region 慣性矩陣

						M_matrix.at<double>(0, 0) = (4557899.0*cos(sang3_cal - sang4_cal)) / 200000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 1000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal)) / 8000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal)) / 8000000000.0 - (2700357.0*sin(sang2_cal - sang3_cal)) / 125000000.0 + (31031.0*sin(sang3_cal - sang4_cal)) / 100000000.0 + (20331.0*sin(sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 62500000.0 - (1864633.0*cos(2 * sang2_cal)) / 50000000.0 - (2259.0*cos(2 * sang5_cal)) / 4000000000.0 - (31441279.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 4000000000.0 + (30399.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 125000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (2536699.0*cos(2 * sang2_cal - sang3_cal)) / 50000000.0 - (10013911.0*cos(2 * sang2_cal - 2 * sang3_cal)) / 500000000.0 - (33033.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 250000000.0 - (4851957.0*sin(sang2_cal - sang3_cal + sang4_cal)) / 500000000.0 - (23343.0*sin(sang3_cal - sang4_cal + sang5_cal)) / 400000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 1000000000.0 + (20331.0*sin(sang4_cal + sang5_cal)) / 400000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 62500000.0 + (2536699.0*c_3) / 50000000.0 + term_3 / 200000000.0 - term_5 / 500000000.0 - (4557899.0*cos(2 * sang2_cal - sang3_cal + sang4_cal)) / 200000000.0 - (3969783.0*cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 200000000.0 - (347127.0*s_2) / 10000000.0 - term_6 / 100000000.0 + (31031.0*sin(2 * sang2_cal - sang3_cal + sang4_cal)) / 100000000.0 + (27027.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 100000000.0 - (23343.0*sin(sang3_cal - sang4_cal - sang5_cal)) / 400000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 1000000000.0 + 105207843 / 800000000.0;
						M_matrix.at<double>(0, 1) = term_1 + (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_16 / 400000000.0 + (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 - (18867.0*c_2) / 100000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 2) = term_2 - (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - term_1 - term_13 / 4000000000.0 + term_14 / 4000000000.0 + term_15 / 62500000.0 + term_17 / 100000000.0 - term_16 / 400000000.0 + (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(0, 3) = term_1 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 4) = (20002259.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 1000000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 - term_15 / 62500000.0 - term_1 - term_16 / 400000000.0 - (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 + (24849.0*c_5) / 1000000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(1, 0) = M_matrix.at<double>(0, 1);
						M_matrix.at<double>(1, 1) = (2536699.0*c_3) / 25000000.0 + term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_9 / 100000000.0 - term_10 / 50000000.0 + term_12 / 50000000.0 + term_7 / 100000000.0 + term_8 / 100000000.0 + term_4 + term_18 / 100000000.0 - term_19 / 100000000.0 + 180370741 / 1000000000.0;
						M_matrix.at<double>(1, 2) = term_5 / 250000000.0 - term_3 / 100000000.0 - (2536699.0*c_3) / 50000000.0 + term_6 / 50000000.0 - term_9 / 200000000.0 + term_10 / 100000000.0 - term_12 / 100000000.0 - term_7 / 100000000.0 - term_8 / 200000000.0 - term_4 - term_18 / 200000000.0 + term_19 / 200000000.0 - 95785421 / 1000000000.0;
						M_matrix.at<double>(1, 3) = term_3 / 200000000.0 - term_5 / 250000000.0 - term_6 / 100000000.0 + term_9 / 200000000.0 - term_10 / 100000000.0 + term_12 / 100000000.0 + term_7 / 200000000.0 + term_8 / 200000000.0 + term_4 + term_18 / 200000000.0 - term_19 / 200000000.0 + 45729777 / 1000000000.0;
						M_matrix.at<double>(1, 4) = (753.0*s_5*(155.0*cos(sang3_cal - sang4_cal) + 135.0*c_4 + 96)) / 1000000000.0;
						M_matrix.at<double>(2, 0) = M_matrix.at<double>(0, 2);
						M_matrix.at<double>(2, 1) = M_matrix.at<double>(1, 2);
						M_matrix.at<double>(2, 2) = term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_7 / 100000000.0 + term_4 + 95785421 / 1000000000.0;
						M_matrix.at<double>(2, 3) = term_5 / 250000000.0 - term_3 / 200000000.0 + term_6 / 100000000.0 - term_7 / 200000000.0 - term_4 - 45729777 / 1000000000.0;
						M_matrix.at<double>(2, 4) = -(2259.0*s_5*(45.0*c_4 + 32)) / 1000000000.0;
						M_matrix.at<double>(3, 0) = M_matrix.at<double>(0, 3);
						M_matrix.at<double>(3, 1) = M_matrix.at<double>(1, 3);
						M_matrix.at<double>(3, 2) = M_matrix.at<double>(2, 3);
						M_matrix.at<double>(3, 3) = term_4 - term_5 / 250000000.0 + 45729777.0 / 1000000000.0;
						M_matrix.at<double>(3, 4) = (2259.0*s_5) / 31250000.0;
						M_matrix.at<double>(4, 0) = M_matrix.at<double>(0, 4);
						M_matrix.at<double>(4, 1) = M_matrix.at<double>(1, 4);
						M_matrix.at<double>(4, 2) = M_matrix.at<double>(2, 4);
						M_matrix.at<double>(4, 3) = M_matrix.at<double>(3, 4);
						M_matrix.at<double>(4, 4) = 20002259.0 / 1000000000.0;
#pragma endregion 慣性矩陣
						/////////////////////////////////////////////慣性矩陣///////////////////////////////////////////////////	
					}

					end_time = clock();
					total_time = (long double)(end_time - start_time) / CLOCKS_PER_SEC;
					nowtime << total_time << endl;
					sacc1 = -(svel1 - temp1) / total_time;
					sacc2 = -(svel2 - temp2) / total_time;
					sacc3 = -(svel3 - temp3) / total_time;
					sacc4 = -(svel4 - temp4) / total_time;
					sacc5 = -(svel5 - temp5) / total_time;

					acc_matrix.at<double>(0, 0) = sacc1;
					acc_matrix.at<double>(1, 0) = sacc2;
					acc_matrix.at<double>(2, 0) = sacc3;
					acc_matrix.at<double>(3, 0) = sacc4;
					acc_matrix.at<double>(4, 0) = sacc5;

					imgtemp_torque.t1 = stau1;
					imgtemp_torque.t2 = stau2;
					imgtemp_torque.t3 = stau3;
					imgtemp_torque.t4 = stau4;
					imgtemp_torque.t5 = stau5;
					imgtest_torque.push_back(imgtemp_torque);

					/////////////////////////////////////////////力矩矩陣///////////////////////////////////////////////////
					tau_matrix = M_matrix*acc_matrix + C_matrix*vel_matrix + N_matrix;
					Acc1 << sacc1 << endl;			Acc2 << sacc2 << endl;			Acc3 << sacc3 << endl;			Acc4 << sacc4 << endl;			Acc5 << sacc5 << endl;
					JointAcc << "第一軸   " << sacc1 << "   第二軸  " << sacc2 << "   第三軸  " << sacc3 << "   第四軸  " << sacc4 << "   第五軸  " << sacc5 << endl;
					CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸 " << stau5 << endl;
					nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸 " << sang5_cal << endl;
					JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸 " << svel5 << endl;


					if (GetAsyncKeyState(0x57))	//w
					{
						return 0;
						break;
					}

				}

			}
#pragma endregion ccc==2

#pragma region ccc==3
			if (ccc == 3)
			{
				SLEEP_MILLISEC(3000);
				myYouBotManipulator->getArmGripper().close();
				while (1)
				{
#pragma region 重力補償test

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();

					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣

					c_2 = cos(sang2_cal);
					c_3 = cos(sang3_cal);
					c_4 = cos(sang4_cal);
					c_5 = cos(sang5_cal);

					s_2 = sin(sang2_cal);
					s_3 = sin(sang3_cal);
					s_4 = sin(sang4_cal);
					s_5 = sin(sang5_cal);
					s_23 = sin(sang2_cal - sang3_cal);
					s_34 = sin(sang3_cal - sang4_cal);
					s2_2 = sin(2 * sang2_cal);
					s2_5 = sin(2 * sang5_cal);
					c_23 = cos(sang2_cal - sang3_cal);
					c_34 = cos(sang3_cal - sang4_cal);
					c_45 = cos(sang4_cal + sang5_cal);
					c_4_5 = cos(sang4_cal - sang5_cal);
					c_25 = cos(sang2_cal + sang5_cal);
					c_2_5 = cos(sang2_cal - sang5_cal);
					s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					s_223 = sin(2 * sang2_cal - sang3_cal);
					s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

					term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					term_3 = (3969783.0*c_4);
					term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					term_5 = (753.0*c_5);
					term_6 = (27027.0*s_4);
					term_7 = (20331.0*c_5*s_4);
					term_8 = (4557899.0*s_3*s_4);
					term_9 = (4557899.0*c_3*c_4);
					term_10 = (31031.0*c_3*s_4);
					term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					term_12 = (31031.0*c_4*s_3);
					term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					term_18 = (23343.0*c_3*c_5*s_4);
					term_19 = (23343.0*c_4*c_5*s_3);


					N_matrix.at<double>(0, 0) = 0.0;
					N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(1, 0) = N_matrix.at<double>(1, 0)*0.6;
					N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
					//N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-1;
					N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0)*0.75;
					N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
					tau_matrix = N_matrix;
#pragma endregion 重力補償test



					desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//LOG(info) << "torque test1";
					desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					//LOG(info) << "torque test2";
					desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//LOG(info) << "torque test3";
					desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					//LOG(info) << "torque test4";
					desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//LOG(info) << "torque test5";

					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸" << stau5 << endl;
					Tau1 << sang1 << endl;	Tau2 << sang2 << endl; Tau3 << sang3 << endl; Tau4 << sang4 << endl; Tau5 << sang5 << endl;
					JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸 " << svel5 << endl;
					nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸 " << sang5_cal << endl;
					CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
					Jointx << "第一軸   " << sang1 << "   第二軸  " << sang2 << "   第三軸  " << sang3 << "   第四軸  " << sang4 << "   第五軸 " << sang5 << endl;
					theta1 = theta2 = theta3 = theta4 = theta5 = theta234 = 0.0;

					//cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
					printf("\n");

					sang1 = sang2 = sang3 = sang4 = sang5 = 0.0;
					//回傳算當前角度

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();
					cutang << count << ":    " << sang1 << "  " << sang2 << "  " << sang3 << "  " << sang4 << "  " << sang5 << endl;

					ang1 = sang1;
					ang2 = sang2;
					ang3 = sang3;
					ang4 = sang4;
					ang5 = sang5;

					ang1 = ang1 - theta1_zero;
					ang2 = ang2 - theta2_zero;
					ang3 = ang3 - theta3_zero;
					ang4 = ang4 - theta4_zero;
					ang5 = ang5 - theta5_zero;
					cout << "第一軸" << ang1*rad2deg << "   第二軸  " << ang2*rad2deg << "   第三軸  " << ang3*rad2deg << "   第四軸  " << ang4*rad2deg << "   第五軸 " << ang5*rad2deg << endl;


					///Because Assume opposite to real Arm.
					ang1 = -ang1;
					//ang2 = -ang2;
					//ang3 = -ang3;
					//ang4 = -ang4; 
					ang5 = -ang5;




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
					r14 = T50.at<double>(0, 3);
					r21 = T50.at<double>(1, 0);
					r22 = T50.at<double>(1, 1);
					r23 = T50.at<double>(1, 2);
					r24 = T50.at<double>(1, 3);
					r31 = T50.at<double>(2, 0);
					r32 = T50.at<double>(2, 1);
					r33 = T50.at<double>(2, 2);
					r34 = T50.at<double>(2, 3);
#pragma region 計算夾爪尤拉角zyz
					/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////
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
					/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////


#pragma region 計算夾爪尤拉角zyz
					tool_end = T50*tool;//求得夾爪的位置x y z


					end_x = tool_end.at<double>(0, 0);
					end_y = tool_end.at<double>(1, 0);
					end_z = tool_end.at<double>(2, 0);
					cout << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;


					if (GetAsyncKeyState(0x57))	//w
					{

						return 0;
						break;

					}
				}
			}
#pragma endregion ccc==3
#pragma region ccc==4
#pragma region compute torque PD控制器
			//未完成 pd參數或許是太小尚未調整 導致手臂無法移動
			if (ccc == 4)
			{


				desiredJointAngle.angle = 0.0109 * radian;
				myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.011* radian;
				myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				desiredJointAngle.angle = -0.016 * radian;
				myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.0223 * radian;
				myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.1201 * radian;
				myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

				myYouBotManipulator->getArmGripper().open();
				LOG(info) << "unfold arm";
				SLEEP_MILLISEC(3000);

				int i = 0;
				cout << "理想軌跡總共時間" << endl;
				cin >> ttime;

				while (1)
				{

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();

					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣

					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣


					ang_matrix.at<double>(0, 0) = sang1_cal;
					ang_matrix.at<double>(1, 0) = sang2_cal;
					ang_matrix.at<double>(2, 0) = sang3_cal;
					ang_matrix.at<double>(3, 0) = sang4_cal;
					ang_matrix.at<double>(4, 0) = sang5_cal;
					vel_matrix.at<double>(0, 0) = svel1;
					vel_matrix.at<double>(1, 0) = svel2;
					vel_matrix.at<double>(2, 0) = svel3;
					vel_matrix.at<double>(3, 0) = svel4;
					vel_matrix.at<double>(4, 0) = svel5;

					c_2 = cos(sang2_cal);
					c_3 = cos(sang3_cal);
					c_4 = cos(sang4_cal);
					c_5 = cos(sang5_cal);

					s_2 = sin(sang2_cal);
					s_3 = sin(sang3_cal);
					s_4 = sin(sang4_cal);
					s_5 = sin(sang5_cal);
					s_23 = sin(sang2_cal - sang3_cal);
					s_34 = sin(sang3_cal - sang4_cal);
					s2_2 = sin(2 * sang2_cal);
					s2_5 = sin(2 * sang5_cal);
					c_23 = cos(sang2_cal - sang3_cal);
					c_34 = cos(sang3_cal - sang4_cal);
					c_45 = cos(sang4_cal + sang5_cal);
					c_4_5 = cos(sang4_cal - sang5_cal);
					c_25 = cos(sang2_cal + sang5_cal);
					c_2_5 = cos(sang2_cal - sang5_cal);
					s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					s_223 = sin(2 * sang2_cal - sang3_cal);
					s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

					term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					term_3 = (3969783.0*c_4);
					term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					term_5 = (753.0*c_5);
					term_6 = (27027.0*s_4);
					term_7 = (20331.0*c_5*s_4);
					term_8 = (4557899.0*s_3*s_4);
					term_9 = (4557899.0*c_3*c_4);
					term_10 = (31031.0*c_3*s_4);
					term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					term_12 = (31031.0*c_4*s_3);
					term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					term_18 = (23343.0*c_3*c_5*s_4);
					term_19 = (23343.0*c_4*c_5*s_3);
					/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
#pragma region 科氏力矩陣

					C_matrix.at<double>(0, 0) = (30399.0*svel2*c2_234) / 125000000.0 - (30399.0*svel3*c2_234) / 125000000.0 + (30399.0*svel4*c2_234) / 125000000.0 + (1864633.0*svel2*s2_2) / 50000000.0 + (2259.0*svel5*s2_5) / 4000000000.0 + (31441279.0*svel2*s2_234) / 4000000000.0 - (31441279.0*svel3*s2_234) / 4000000000.0 + (31441279.0*svel4*s2_234) / 4000000000.0 - (23343.0*svel2*c_22345) / 400000000.0 - (20331.0*svel2*c_222345) / 400000000.0 + (23343.0*svel3*c_22345) / 800000000.0 + (20331.0*svel3*c_222345) / 400000000.0 - (23343.0*svel4*c_22345) / 800000000.0 - (20331.0*svel4*c_222345) / 800000000.0 - (23343.0*svel5*c_22345) / 800000000.0 - (20331.0*svel5*c_222345) / 800000000.0 - (24849.0*svel2*s_234_5) / 2000000000.0 + (24849.0*svel3*s_234_5) / 2000000000.0 - (24849.0*svel4*s_234_5) / 2000000000.0 + (24849.0*svel5*s_234_5) / 2000000000.0 + (2536699.0*svel2*s_223) / 50000000.0 + (10013911.0*svel2*s2_23) / 500000000.0 - (2536699.0*svel3*s_223) / 100000000.0 - (10013911.0*svel3*s2_23) / 500000000.0 - (4851957.0*svel2*c_234) / 1000000000.0 + (4851957.0*svel3*c_234) / 1000000000.0 - (4851957.0*svel4*c_234) / 1000000000.0 - (23343.0*svel3*c_345) / 800000000.0 + (23343.0*svel4*c_345) / 800000000.0 - (23343.0*svel5*c_345) / 800000000.0 + (33033.0*svel2*s_234) / 500000000.0 - (33033.0*svel3*s_234) / 500000000.0 + (33033.0*svel4*s_234) / 500000000.0 + (20331.0*svel4*c_45) / 800000000.0 + (20331.0*svel5*c_45) / 800000000.0 - (23343.0*svel2*c_2234_5) / 400000000.0 - (20331.0*svel2*c_22234_5) / 400000000.0 + (23343.0*svel3*c_2234_5) / 800000000.0 - (2259.0*svel2*c_2223245) / 62500000.0 + (20331.0*svel3*c_22234_5) / 400000000.0 - (23343.0*svel4*c_2234_5) / 800000000.0 + (2259.0*svel3*c_2223245) / 62500000.0 - (20331.0*svel4*c_22234_5) / 800000000.0 + (23343.0*svel5*c_2234_5) / 800000000.0 - (2259.0*svel4*c_2223245) / 62500000.0 + (20331.0*svel5*c_22234_5) / 800000000.0 - (2259.0*svel5*c_2223245) / 125000000.0 + (753.0*svel2*s_2223245) / 1000000000.0 - (753.0*svel3*s_2223245) / 1000000000.0 + (753.0*svel4*s_2223245) / 1000000000.0 + (753.0*svel5*s_2223245) / 2000000000.0 - (347127.0*svel2*c_2) / 20000000.0 - (27027.0*svel4*c_4) / 200000000.0 + (31031.0*svel2*c_2234) / 100000000.0 + (27027.0*svel2*c_22234) / 100000000.0 - (31031.0*svel3*c_2234) / 200000000.0 - (27027.0*svel3*c_22234) / 100000000.0 + (31031.0*svel4*c_2234) / 200000000.0 - (23343.0*svel3*c_34_5) / 800000000.0 + (27027.0*svel4*c_22234) / 200000000.0 + (23343.0*svel4*c_34_5) / 800000000.0 + (23343.0*svel5*c_34_5) / 800000000.0 - (2536699.0*svel3*s_3) / 100000000.0 - (3969783.0*svel4*s_4) / 400000000.0 + (753.0*svel5*s_5) / 1000000000.0 + (4557899.0*svel2*s_2234) / 200000000.0 + (3969783.0*svel2*s_22234) / 200000000.0 - (4557899.0*svel3*s_2234) / 400000000.0 - (3969783.0*svel3*s_22234) / 200000000.0 + (4557899.0*svel4*s_2234) / 400000000.0 + (3969783.0*svel4*s_22234) / 400000000.0 - (24849.0*svel2*s_2345) / 2000000000.0 + (24849.0*svel3*s_2345) / 2000000000.0 - (24849.0*svel4*s_2345) / 2000000000.0 - (24849.0*svel5*s_2345) / 2000000000.0 - (2700357.0*svel2*c_23) / 250000000.0 + (2700357.0*svel3*c_23) / 250000000.0 + (31031.0*svel3*c_34) / 200000000.0 - (31031.0*svel4*c_34) / 200000000.0 + (20331.0*svel4*c_4_5) / 800000000.0 - (20331.0*svel5*c_4_5) / 800000000.0 - (2259.0*svel2*c_222324_5) / 62500000.0 + (2259.0*svel3*c_222324_5) / 62500000.0 - (2259.0*svel4*c_222324_5) / 62500000.0 + (2259.0*svel5*c_222324_5) / 125000000.0 - (4557899.0*svel3*s_34) / 400000000.0 + (4557899.0*svel4*s_34) / 400000000.0 + (753.0*svel2*s_222324_5) / 1000000000.0 - (2259.0*svel2*s2_234_5) / 8000000000.0 - (2259.0*svel2*s2_2345) / 8000000000.0 - (753.0*svel3*s_222324_5) / 1000000000.0 + (2259.0*svel3*s2_234_5) / 8000000000.0 + (2259.0*svel3*s2_2345) / 8000000000.0 + (753.0*svel4*s_222324_5) / 1000000000.0 - (2259.0*svel4*s2_234_5) / 8000000000.0 - (2259.0*svel4*s2_2345) / 8000000000.0 - (753.0*svel5*s_222324_5) / 2000000000.0 + (2259.0*svel5*s2_234_5) / 8000000000.0 - (2259.0*svel5*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 1) = (30399.0*svel1*c2_234) / 125000000.0 + (1864633.0*svel1*s2_2) / 50000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 400000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 400000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 + (2536699.0*svel1*s_223) / 50000000.0 + (10013911.0*svel1*s2_23) / 500000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (20331.0*svel2*c_235) / 400000000.0 - (20331.0*svel3*c_235) / 400000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel2*c_25) / 400000000.0 - (23343.0*svel1*c_2234_5) / 400000000.0 - (20331.0*svel1*c_22234_5) / 400000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (347127.0*svel1*c_2) / 20000000.0 + (31031.0*svel1*c_2234) / 100000000.0 + (27027.0*svel1*c_22234) / 100000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 + (18867.0*svel2*s_2) / 100000000.0 + (4557899.0*svel1*s_2234) / 200000000.0 + (3969783.0*svel1*s_22234) / 200000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (2700357.0*svel1*c_23) / 250000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 - (441029.0*svel2*s_23) / 500000000.0 + (441029.0*svel3*s_23) / 500000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 2) = (23343.0*svel1*c_22345) / 800000000.0 - (31441279.0*svel1*s2_234) / 4000000000.0 - (30399.0*svel1*c2_234) / 125000000.0 + (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 400000000.0 - (2259.0*svel3*c_234_5) / 62500000.0 + (2259.0*svel4*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 - (753.0*svel2*s_234_5) / 1000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 + (753.0*svel3*s_234_5) / 1000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 - (753.0*svel4*s_234_5) / 1000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (2259.0*svel5*s_234_25) / 4000000000.0 - (2259.0*svel5*s_23425) / 4000000000.0 - (2536699.0*svel1*s_223) / 100000000.0 - (10013911.0*svel1*s2_23) / 500000000.0 + (4851957.0*svel1*c_234) / 1000000000.0 - (20331.0*svel2*c_235) / 400000000.0 - (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (33033.0*svel1*s_234) / 500000000.0 - (25433.0*svel2*s_234) / 100000000.0 + (25433.0*svel3*s_234) / 100000000.0 - (25433.0*svel4*s_234) / 100000000.0 + (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 400000000.0 + (2259.0*svel1*c_2223245) / 62500000.0 - (753.0*svel1*s_2223245) / 1000000000.0 - (31031.0*svel1*c_2234) / 200000000.0 - (27027.0*svel1*c_22234) / 100000000.0 + (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 - (20331.0*svel3*c_23_5) / 400000000.0 - (2536699.0*svel1*s_3) / 100000000.0 - (4557899.0*svel1*s_2234) / 400000000.0 - (3969783.0*svel1*s_22234) / 200000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 + (2700357.0*svel1*c_23) / 250000000.0 + (31031.0*svel1*c_34) / 200000000.0 + (2259.0*svel1*c_222324_5) / 62500000.0 + (441029.0*svel2*s_23) / 500000000.0 - (4557899.0*svel1*s_34) / 400000000.0 - (441029.0*svel3*s_23) / 500000000.0 - (753.0*svel1*s_222324_5) / 1000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 3) = (30399.0*svel1*c2_234) / 125000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (27027.0*svel1*c_4) / 200000000.0 + (31031.0*svel1*c_2234) / 200000000.0 + (27027.0*svel1*c_22234) / 200000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 - (3969783.0*svel1*s_4) / 400000000.0 + (4557899.0*svel1*s_2234) / 400000000.0 + (3969783.0*svel1*s_22234) / 400000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (31031.0*svel1*c_34) / 200000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 + (4557899.0*svel1*s_34) / 400000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 4) = (2259.0*svel1*s2_5) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel5*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 - (753.0*svel5*s_234_5) / 1000000000.0 - (23343.0*svel1*c_345) / 800000000.0 - (20331.0*svel5*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel5*c_25) / 400000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 125000000.0 + (753.0*svel1*s_2223245) / 2000000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel5*c_23_5) / 400000000.0 + (753.0*svel1*s_5) / 1000000000.0 - (24849.0*svel5*s_5) / 1000000000.0 - (2259.0*svel5*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel5*s_2345) / 1000000000.0 - (20331.0*svel1*c_4_5) / 800000000.0 + (23343.0*svel5*c_2_5) / 400000000.0 + (2259.0*svel1*c_222324_5) / 125000000.0 - (753.0*svel1*s_222324_5) / 2000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(1, 0) = (svel1*((24849.0*s_2345) / 1000000000.0 + (2700357.0*c_23) / 125000000.0 + (2259.0*c_222324_5) / 31250000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (1864633.0*s2_2) / 25000000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 200000000.0 + (20331.0*c_222345) / 200000000.0 + (24849.0*s_234_5) / 1000000000.0 - (2536699.0*s_223) / 25000000.0 - (10013911.0*s2_23) / 250000000.0 + (4851957.0*c_234) / 500000000.0 - (33033.0*s_234) / 250000000.0 + (23343.0*c_2234_5) / 200000000.0 + (20331.0*c_22234_5) / 200000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (347127.0*c_2) / 10000000.0 - (31031.0*c_2234) / 50000000.0 - (27027.0*c_22234) / 50000000.0 - (4557899.0*s_2234) / 100000000.0 - (3969783.0*s_22234) / 100000000)) / 2 + (svel5*((23343.0*c_2_5) / 200000000.0 - (753.0*s_2345) / 500000000.0 + (2259.0*c_234_5) / 31250000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (23343.0*c_25) / 200000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
					C_matrix.at<double>(1, 1) = (svel4*((20331.0*c_4_5) / 200000000.0 - (31031.0*c_34) / 50000000.0 + (4557899.0*s_34) / 100000000.0 + (23343.0*c_345) / 200000000.0 + (20331.0*c_45) / 200000000.0 - (27027.0*c_4) / 50000000.0 + (23343.0*c_34_5) / 200000000.0 - (3969783.0*s_4) / 100000000)) / 2 - (svel3*((4557899.0*s_34) / 100000000.0 - (31031.0*c_34) / 50000000.0 + (23343.0*c_345) / 200000000.0 + (23343.0*c_34_5) / 200000000.0 + (2536699.0*s_3) / 25000000)) / 2 - (svel5*((2259.0*s2_5) / 1000000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000.0 + (23343.0*c_3*s_4*s_5) / 100000000.0 - (23343.0*c_4*s_3*s_5) / 100000000)) / 2;
					C_matrix.at<double>(1, 2) = (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 + (27027.0*svel4*c_4) / 100000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 + (23343.0*svel3*c_34_5) / 400000000.0 - (23343.0*svel4*c_34_5) / 400000000.0 - (23343.0*svel5*c_34_5) / 400000000.0 - (2536699.0*svel2*s_3) / 50000000.0 + (2536699.0*svel3*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_34) / 100000000.0 - (31031.0*svel3*c_34) / 100000000.0 + (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0 - (4557899.0*svel2*s_34) / 200000000.0 + (4557899.0*svel3*s_34) / 200000000.0 - (4557899.0*svel4*s_34) / 200000000.0;
					C_matrix.at<double>(1, 3) = (23343.0*svel2*c_345) / 400000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel3*c_345) / 400000000.0 + (23343.0*svel4*c_345) / 400000000.0 - (23343.0*svel5*c_345) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 - (27027.0*svel2*c_4) / 100000000.0 + (27027.0*svel3*c_4) / 100000000.0 - (27027.0*svel4*c_4) / 100000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 - (3969783.0*svel2*s_4) / 200000000.0 + (3969783.0*svel3*s_4) / 200000000.0 - (3969783.0*svel4*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_34) / 100000000.0 + (31031.0*svel3*c_34) / 100000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0 + (4557899.0*svel2*s_34) / 200000000.0 - (4557899.0*svel3*s_34) / 200000000.0 + (4557899.0*svel4*s_34) / 200000000.0;
					C_matrix.at<double>(1, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (23343.0*svel1*c_25) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 + (23343.0*svel1*c_2_5) / 400000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0;
					C_matrix.at<double>(2, 0) = (svel1*((4557899.0*s_34) / 200000000.0 - (2700357.0*c_23) / 125000000.0 - (31031.0*c_34) / 100000000.0 - (2259.0*c_222324_5) / 31250000.0 - (24849.0*s_2345) / 1000000000.0 + (753.0*s_222324_5) / 500000000.0 - (2259.0*s2_234_5) / 4000000000.0 - (2259.0*s2_2345) / 4000000000.0 + (30399.0*c2_234) / 62500000.0 + (31441279.0*s2_234) / 2000000000.0 - (23343.0*c_22345) / 400000000.0 - (20331.0*c_222345) / 200000000.0 - (24849.0*s_234_5) / 1000000000.0 + (2536699.0*s_223) / 50000000.0 + (10013911.0*s2_23) / 250000000.0 - (4851957.0*c_234) / 500000000.0 + (23343.0*c_345) / 400000000.0 + (33033.0*s_234) / 250000000.0 - (23343.0*c_2234_5) / 400000000.0 - (20331.0*c_22234_5) / 200000000.0 - (2259.0*c_2223245) / 31250000.0 + (753.0*s_2223245) / 500000000.0 + (31031.0*c_2234) / 100000000.0 + (27027.0*c_22234) / 50000000.0 + (23343.0*c_34_5) / 400000000.0 + (2536699.0*s_3) / 50000000.0 + (4557899.0*s_2234) / 200000000.0 + (3969783.0*s_22234) / 100000000)) / 2 - (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
					C_matrix.at<double>(2, 1) = (2259.0*svel5*s2_5) / 2000000000.0 + (27027.0*svel4*c_4) / 100000000.0 + (2536699.0*svel2*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel4*c_4*c_5) / 200000000.0 - (4557899.0*svel2*c_3*s_4) / 200000000.0 + (4557899.0*svel2*c_4*s_3) / 200000000.0 - (31031.0*svel2*s_3*s_4) / 100000000.0 + (20331.0*svel5*s_4*s_5) / 200000000.0 + (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 + (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
					C_matrix.at<double>(2, 2) = -(svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
					C_matrix.at<double>(2, 3) = (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
					C_matrix.at<double>(2, 4) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 - (2259.0*svel5*c_5) / 31250000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0;
					C_matrix.at<double>(3, 0) = (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20002259.0*s_234) / 1000000000.0 + (2259.0*c_2345) / 31250000)) / 2 + (svel1*((24849.0*s_2345) / 1000000000.0 + (31031.0*c_34) / 100000000.0 - (20331.0*c_4_5) / 400000000.0 + (2259.0*c_222324_5) / 31250000.0 - (4557899.0*s_34) / 200000000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 400000000.0 + (20331.0*c_222345) / 400000000.0 + (24849.0*s_234_5) / 1000000000.0 + (4851957.0*c_234) / 500000000.0 - (23343.0*c_345) / 400000000.0 - (33033.0*s_234) / 250000000.0 - (20331.0*c_45) / 400000000.0 + (23343.0*c_2234_5) / 400000000.0 + (20331.0*c_22234_5) / 400000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (27027.0*c_4) / 100000000.0 - (31031.0*c_2234) / 100000000.0 - (27027.0*c_22234) / 100000000.0 - (23343.0*c_34_5) / 400000000.0 + (3969783.0*s_4) / 200000000.0 - (4557899.0*s_2234) / 200000000.0 - (3969783.0*s_22234) / 200000000)) / 2;
					C_matrix.at<double>(3, 1) = (27027.0*svel2*c_4) / 100000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (27027.0*svel3*c_4) / 100000000.0 + (3969783.0*svel2*s_4) / 200000000.0 - (3969783.0*svel3*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel2*c_4*c_5) / 200000000.0 + (20331.0*svel3*c_4*c_5) / 200000000.0 + (4557899.0*svel2*c_3*s_4) / 200000000.0 - (4557899.0*svel2*c_4*s_3) / 200000000.0 + (31031.0*svel2*s_3*s_4) / 100000000.0 - (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 - (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
					C_matrix.at<double>(3, 2) = (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((753.0*s_5) / 250000000.0 - (2259.0*c_5*s_5) / 500000000)) / 2;
					C_matrix.at<double>(3, 3) = -(753.0*svel5*(3.0*s2_5 - 4.0*s_5)) / 2000000000.0;
					C_matrix.at<double>(3, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0;
					C_matrix.at<double>(4, 0) = (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel1*s2_5) / 4000000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (20331.0*svel2*c_235) / 400000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 - (23343.0*svel2*c_25) / 400000000.0 - (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 + (2259.0*svel1*c_2223245) / 125000000.0 - (753.0*svel1*s_2223245) / 2000000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 - (753.0*svel1*s_5) / 1000000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 125000000.0 + (753.0*svel1*s_222324_5) / 2000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(4, 1) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 + (23343.0*svel2*c_345) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (23343.0*svel1*c_25) / 400000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 - (23343.0*svel1*c_2_5) / 400000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0;
					C_matrix.at<double>(4, 2) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0;
					C_matrix.at<double>(4, 3) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0;
					C_matrix.at<double>(4, 4) = 0.0;
#pragma endregion 科氏力矩陣
					/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
					/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////
#pragma region 重力矩陣


					N_matrix.at<double>(0, 0) = 0.0;
					N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;

#pragma endregion 重力矩陣
					/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////

					/////////////////////////////////////////////慣性矩陣/////////////////////////////
#pragma region 慣性矩陣

					M_matrix.at<double>(0, 0) = (4557899.0*cos(sang3_cal - sang4_cal)) / 200000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 1000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal)) / 8000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal)) / 8000000000.0 - (2700357.0*sin(sang2_cal - sang3_cal)) / 125000000.0 + (31031.0*sin(sang3_cal - sang4_cal)) / 100000000.0 + (20331.0*sin(sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 62500000.0 - (1864633.0*cos(2 * sang2_cal)) / 50000000.0 - (2259.0*cos(2 * sang5_cal)) / 4000000000.0 - (31441279.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 4000000000.0 + (30399.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 125000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (2536699.0*cos(2 * sang2_cal - sang3_cal)) / 50000000.0 - (10013911.0*cos(2 * sang2_cal - 2 * sang3_cal)) / 500000000.0 - (33033.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 250000000.0 - (4851957.0*sin(sang2_cal - sang3_cal + sang4_cal)) / 500000000.0 - (23343.0*sin(sang3_cal - sang4_cal + sang5_cal)) / 400000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 1000000000.0 + (20331.0*sin(sang4_cal + sang5_cal)) / 400000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 62500000.0 + (2536699.0*c_3) / 50000000.0 + term_3 / 200000000.0 - term_5 / 500000000.0 - (4557899.0*cos(2 * sang2_cal - sang3_cal + sang4_cal)) / 200000000.0 - (3969783.0*cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 200000000.0 - (347127.0*s_2) / 10000000.0 - term_6 / 100000000.0 + (31031.0*sin(2 * sang2_cal - sang3_cal + sang4_cal)) / 100000000.0 + (27027.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 100000000.0 - (23343.0*sin(sang3_cal - sang4_cal - sang5_cal)) / 400000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 1000000000.0 + 105207843 / 800000000.0;
					M_matrix.at<double>(0, 1) = term_1 + (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_16 / 400000000.0 + (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 - (18867.0*c_2) / 100000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 + term_11 / 1000000000.0;
					M_matrix.at<double>(0, 2) = term_2 - (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - term_1 - term_13 / 4000000000.0 + term_14 / 4000000000.0 + term_15 / 62500000.0 + term_17 / 100000000.0 - term_16 / 400000000.0 + (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
					M_matrix.at<double>(0, 3) = term_1 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_11 / 1000000000.0;
					M_matrix.at<double>(0, 4) = (20002259.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 1000000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 - term_15 / 62500000.0 - term_1 - term_16 / 400000000.0 - (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 + (24849.0*c_5) / 1000000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
					M_matrix.at<double>(1, 0) = M_matrix.at<double>(0, 1);
					M_matrix.at<double>(1, 1) = (2536699.0*c_3) / 25000000.0 + term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_9 / 100000000.0 - term_10 / 50000000.0 + term_12 / 50000000.0 + term_7 / 100000000.0 + term_8 / 100000000.0 + term_4 + term_18 / 100000000.0 - term_19 / 100000000.0 + 180370741 / 1000000000.0;
					M_matrix.at<double>(1, 2) = term_5 / 250000000.0 - term_3 / 100000000.0 - (2536699.0*c_3) / 50000000.0 + term_6 / 50000000.0 - term_9 / 200000000.0 + term_10 / 100000000.0 - term_12 / 100000000.0 - term_7 / 100000000.0 - term_8 / 200000000.0 - term_4 - term_18 / 200000000.0 + term_19 / 200000000.0 - 95785421 / 1000000000.0;
					M_matrix.at<double>(1, 3) = term_3 / 200000000.0 - term_5 / 250000000.0 - term_6 / 100000000.0 + term_9 / 200000000.0 - term_10 / 100000000.0 + term_12 / 100000000.0 + term_7 / 200000000.0 + term_8 / 200000000.0 + term_4 + term_18 / 200000000.0 - term_19 / 200000000.0 + 45729777 / 1000000000.0;
					M_matrix.at<double>(1, 4) = (753.0*s_5*(155.0*cos(sang3_cal - sang4_cal) + 135.0*c_4 + 96)) / 1000000000.0;
					M_matrix.at<double>(2, 0) = M_matrix.at<double>(0, 2);
					M_matrix.at<double>(2, 1) = M_matrix.at<double>(1, 2);
					M_matrix.at<double>(2, 2) = term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_7 / 100000000.0 + term_4 + 95785421 / 1000000000.0;
					M_matrix.at<double>(2, 3) = term_5 / 250000000.0 - term_3 / 200000000.0 + term_6 / 100000000.0 - term_7 / 200000000.0 - term_4 - 45729777 / 1000000000.0;
					M_matrix.at<double>(2, 4) = -(2259.0*s_5*(45.0*c_4 + 32)) / 1000000000.0;
					M_matrix.at<double>(3, 0) = M_matrix.at<double>(0, 3);
					M_matrix.at<double>(3, 1) = M_matrix.at<double>(1, 3);
					M_matrix.at<double>(3, 2) = M_matrix.at<double>(2, 3);
					M_matrix.at<double>(3, 3) = term_4 - term_5 / 250000000.0 + 45729777.0 / 1000000000.0;
					M_matrix.at<double>(3, 4) = (2259.0*s_5) / 31250000.0;
					M_matrix.at<double>(4, 0) = M_matrix.at<double>(0, 4);
					M_matrix.at<double>(4, 1) = M_matrix.at<double>(1, 4);
					M_matrix.at<double>(4, 2) = M_matrix.at<double>(2, 4);
					M_matrix.at<double>(4, 3) = M_matrix.at<double>(3, 4);
					M_matrix.at<double>(4, 4) = 20002259.0 / 1000000000.0;
#pragma endregion 慣性矩陣


					///////所有角度都先以動力學模型的角度轉換為基準
#pragma region 期望點軌跡分割
					if (count == 0)
					{
						for (total_time = 0; total_time < ttime + 1; total_time++)
						{
							//desired_ang.at<double>(0, 0) = ((-2 / pow(ttime, 3))*((-2.5378) - (-2.9387))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*pow(total_time, 2)) + (-2.9387);
							//desired_ang.at<double>(1, 0) = ((-2 / pow(ttime, 3))*((-0.699) - (-1.1235))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-0.699) - (-1.1235))*pow(total_time, 2)) + (-1.1235);
							//desired_ang.at<double>(2, 0) = ((-2 / pow(ttime, 3))*(-2.1482 - (-2.5325))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*(-2.1482 - (-2.5325))*pow(total_time, 2)) + ((-2.5325));
							//desired_ang.at<double>(3, 0) = ((-2 / pow(ttime, 3))*((-1.289) - (-1.7667))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.289) - (-1.7667))*pow(total_time, 2)) + (-1.7667);
							//desired_ang.at<double>(4, 0) = ((-2 / pow(ttime, 3))*((-1.63811) - (-2.8033))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*pow(total_time, 2)) + (-2.8033);



							//desired_vel.at<double>(0, 0) = (3 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*total_time;
							//desired_vel.at<double>(1, 0) = (3 * (-2 / pow(ttime, 3))*((-0.699) - (-1.1235)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-0.699) - (-1.1235))*total_time;
							//desired_vel.at<double>(2, 0) = (3 * (-2 / pow(ttime, 3))*(-2.1482 - (-2.5325)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*(-2.1482 - (-2.5325))*total_time;
							//desired_vel.at<double>(3, 0) = (3 * (-2 / pow(ttime, 3))*((-1.289) - (-1.7667)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.289) - (-1.7667))*total_time;
							//desired_vel.at<double>(4, 0) = (3 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*total_time;

							//desired_acc.at<double>(0, 0) = (6 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387));
							//desired_acc.at<double>(1, 0) = (6 * (-2 / pow(ttime, 3))*((-0.699) - (-1.1235)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-0.699) - (-1.1235));
							//desired_acc.at<double>(2, 0) = (6 * (-2 / pow(ttime, 3))*(-2.1482 - (-2.5325)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*(-2.1482 - (-2.5325));
							//desired_acc.at<double>(3, 0) = (6 * (-2 / pow(ttime, 3))*((-1.289) - (-1.7667)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.289) - (-1.7667));
							//desired_acc.at<double>(4, 0) = (6 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033));


							desired_ang.at<double>(0, 0) = sang1_cal;
							desired_ang.at<double>(1, 0) = ((-2 / pow(ttime, 3))*((-0.699) - (-1.1235))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-0.699) - (-1.1235))*pow(total_time, 2)) + (-1.1235);
							desired_ang.at<double>(2, 0) = sang3_cal;
							desired_ang.at<double>(3, 0) = sang4_cal;
							desired_ang.at<double>(4, 0) = sang5_cal;



							desired_vel.at<double>(0, 0) = 0;
							desired_vel.at<double>(1, 0) = (3 * (-2 / pow(ttime, 3))*((-0.699) - (-1.1235)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-0.699) - (-1.1235))*total_time;
							desired_vel.at<double>(2, 0) = 0;
							desired_vel.at<double>(3, 0) = 0;
							desired_vel.at<double>(4, 0) = 0;

							desired_acc.at<double>(0, 0) = 0;
							desired_acc.at<double>(1, 0) = (6 * (-2 / pow(ttime, 3))*((-0.699) - (-1.1235)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-0.699) - (-1.1235));
							desired_acc.at<double>(2, 0) = 0;
							desired_acc.at<double>(3, 0) = 0;
							desired_acc.at<double>(4, 0) = 0;





							temp_ang.j1 = desired_ang.at<double>(0, 0);
							temp_ang.j2 = desired_ang.at<double>(1, 0);
							temp_ang.j3 = desired_ang.at<double>(2, 0);
							temp_ang.j4 = desired_ang.at<double>(3, 0);
							temp_ang.j5 = desired_ang.at<double>(4, 0);
							temp_point.push_back(temp_ang);

							veltemp.j1 = desired_vel.at<double>(0, 0);
							veltemp.j2 = desired_vel.at<double>(1, 0);
							veltemp.j3 = desired_vel.at<double>(2, 0);
							veltemp.j4 = desired_vel.at<double>(3, 0);
							veltemp.j5 = desired_vel.at<double>(4, 0);
							vel_test.push_back(veltemp);

							acctemp.j1 = desired_acc.at<double>(0, 0);
							acctemp.j2 = desired_acc.at<double>(1, 0);
							acctemp.j3 = desired_acc.at<double>(2, 0);
							acctemp.j4 = desired_acc.at<double>(3, 0);
							acctemp.j5 = desired_acc.at<double>(4, 0);
							acc_test.push_back(acctemp);

						}
					}
					qqq = vel_test.size();

					desired_tempang.at<double>(0, 0) = temp_point[i].j1;
					desired_tempang.at<double>(1, 0) = temp_point[i].j2;
					desired_tempang.at<double>(2, 0) = temp_point[i].j3;
					desired_tempang.at<double>(3, 0) = temp_point[i].j4;
					desired_tempang.at<double>(4, 0) = temp_point[i].j5;

					desired_tempvel.at<double>(0, 0) = vel_test[i].j1;
					desired_tempvel.at<double>(1, 0) = vel_test[i].j2;
					desired_tempvel.at<double>(2, 0) = vel_test[i].j3;
					desired_tempvel.at<double>(3, 0) = vel_test[i].j4;
					desired_tempvel.at<double>(4, 0) = vel_test[i].j5;

					desired_tempacc.at<double>(0, 0) = acc_test[i].j1;
					desired_tempacc.at<double>(1, 0) = acc_test[i].j2;
					desired_tempacc.at<double>(2, 0) = acc_test[i].j3;
					desired_tempacc.at<double>(3, 0) = acc_test[i].j4;
					desired_tempacc.at<double>(4, 0) = acc_test[i].j5;


#pragma endregion 期望點軌跡分割

#pragma region KpKd
					Kp = 0;
					Kd = 0;
					Kp.at<double>(0, 0) = 10;
					Kp.at<double>(1, 1) = 10;
					Kp.at<double>(2, 2) = 10;
					Kp.at<double>(3, 3) = 10;
					Kp.at<double>(4, 4) = 10;

					Kd.at<double>(0, 0) = 6;
					Kd.at<double>(1, 1) = 6;
					Kd.at<double>(2, 2) = 6;
					Kd.at<double>(3, 3) = 6;
					Kd.at<double>(4, 4) = 6;




#pragma endregion KpKd
					joint_poserror = ang_matrix - desired_tempang;
					joint_poserror_dot = vel_matrix - desired_tempvel;

					J1poserror = joint_poserror.at<double>(0, 0);
					J2poserror = joint_poserror.at<double>(1, 0);
					J3poserror = joint_poserror.at<double>(2, 0);
					J4poserror = joint_poserror.at<double>(3, 0);
					J5poserror = joint_poserror.at<double>(4, 0);

					J1velerror = joint_poserror_dot.at<double>(0, 0);
					J2velerror = joint_poserror_dot.at<double>(1, 0);
					J3velerror = joint_poserror_dot.at<double>(2, 0);
					J4velerror = joint_poserror_dot.at<double>(3, 0);
					J5velerror = joint_poserror_dot.at<double>(4, 0);



					FF = M_matrix*(desired_tempacc)+C_matrix*desired_tempvel + N_matrix;
					FF.at<double>(2, 0) = FF.at<double>(2, 0)*-1;
					FB = M_matrix*((-Kd*joint_poserror_dot) - (Kp*joint_poserror));
					FB.at<double>(2, 0) = FB.at<double>(2, 0)*-1;
					tau_matrix = FF + FB;
					check1 = -Kd*joint_poserror_dot - Kp*joint_poserror;//檢查用



					M11 = M_matrix.at<double>(0, 0); M12 = M_matrix.at<double>(0, 1); M13 = M_matrix.at<double>(0, 2); M14 = M_matrix.at<double>(0, 3); M15 = M_matrix.at<double>(0, 4);
					M21 = M_matrix.at<double>(1, 0); M22 = M_matrix.at<double>(1, 1); M23 = M_matrix.at<double>(1, 2); M24 = M_matrix.at<double>(1, 3); M25 = M_matrix.at<double>(1, 4);
					M31 = M_matrix.at<double>(2, 0); M32 = M_matrix.at<double>(2, 1); M33 = M_matrix.at<double>(2, 2); M34 = M_matrix.at<double>(2, 3); M35 = M_matrix.at<double>(2, 4);
					M41 = M_matrix.at<double>(3, 0); M42 = M_matrix.at<double>(3, 1); M43 = M_matrix.at<double>(3, 2); M44 = M_matrix.at<double>(3, 3); M45 = M_matrix.at<double>(3, 4);
					M51 = M_matrix.at<double>(4, 0); M52 = M_matrix.at<double>(4, 1); M53 = M_matrix.at<double>(4, 2); M54 = M_matrix.at<double>(4, 3); M55 = M_matrix.at<double>(4, 4);

					FF1 = FF.at<double>(0, 0);
					FF2 = FF.at<double>(1, 0);
					FF3 = FF.at<double>(2, 0);
					FF4 = FF.at<double>(3, 0);
					FF5 = FF.at<double>(4, 0);

					FB1 = FB.at<double>(0, 0);
					FB2 = FB.at<double>(1, 0);
					FB3 = FB.at<double>(2, 0);
					FB4 = FB.at<double>(3, 0);
					FB5 = FB.at<double>(4, 0);



					tau_matrix.at<double>(0, 0) = tau_matrix.at<double>(0, 0);
					tau_matrix.at<double>(1, 0) = tau_matrix.at<double>(1, 0);
					tau_matrix.at<double>(2, 0) = tau_matrix.at<double>(2, 0);
					tau_matrix.at<double>(3, 0) = tau_matrix.at<double>(3, 0);
					tau_matrix.at<double>(4, 0) = tau_matrix.at<double>(4, 0);



					N1 = tau_matrix.at<double>(0, 0);
					N2 = tau_matrix.at<double>(1, 0);
					N3 = tau_matrix.at<double>(2, 0);
					N4 = tau_matrix.at<double>(3, 0);
					N5 = tau_matrix.at<double>(4, 0);
					cout << "N1" << N1 << endl;
					cout << "N2" << N2 << endl;
					cout << "N3" << N3 << endl;
					cout << "N4" << N4 << endl;
					cout << "N5" << N5 << endl;
#pragma region 限制力矩
					if (tau_matrix.at<double>(0, 0) > 3)
					{
						tau_matrix.at<double>(0, 0) = 3;
					}
					if (tau_matrix.at<double>(0, 0) < -3)
					{
						tau_matrix.at<double>(0, 0) = -3;
					}
					if (tau_matrix.at<double>(1, 0) > 9.5)
					{
						tau_matrix.at<double>(1, 0) = 5;
					}
					if (tau_matrix.at<double>(1, 0) < -9.5)
					{
						tau_matrix.at<double>(1, 0) = -5;
					}
					if (tau_matrix.at<double>(2, 0) > 6.0)
					{
						tau_matrix.at<double>(2, 0) = 5.0;
					}
					if (tau_matrix.at<double>(2, 0) < -6.0)
					{
						tau_matrix.at<double>(2, 0) = -5.0;
					}
					if (tau_matrix.at<double>(3, 0) > 2.0)
					{
						tau_matrix.at<double>(3, 0) = 2.0;
					}
					if (tau_matrix.at<double>(3, 0) < -2.0)
					{
						tau_matrix.at<double>(3, 0) = -2.0;
					}
					if (tau_matrix.at<double>(4, 0) > 1.0)
					{
						tau_matrix.at<double>(4, 0) = 1.0;
					}
					if (tau_matrix.at<double>(4, 0) < -1.0)
					{
						tau_matrix.at<double>(4, 0) = -1.0;
					}

					N1 = tau_matrix.at<double>(0, 0);
					N2 = tau_matrix.at<double>(1, 0);
					N3 = tau_matrix.at<double>(2, 0);
					N4 = tau_matrix.at<double>(3, 0);
					N5 = tau_matrix.at<double>(4, 0);

#pragma endregion 限制力矩

#pragma region 力矩輸出
					//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//LOG(info) << "torque test1";
					desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					LOG(info) << "torque test2";
					//desiredJointTorque.torque = -3 * newton_meter;
					//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//LOG(info) << "torque test3";
					//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					//LOG(info) << "torque test4";
					//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//LOG(info) << "torque test5";
#pragma endregion 力矩輸出

					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸" << stau5 << endl;
					nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸" << sang5_cal << endl;
					CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
					jointerror << "第一軸   " << J1poserror << "    第二軸  " << J2poserror << "    第三軸  " << J3poserror << "   第四軸  " << J4poserror << "   第五軸  " << J5poserror << endl;
					count++;
					if (abs(J1poserror) < 0.1 && abs(J2poserror) < 0.1 && abs(J3poserror) < 0.1 && abs(J4poserror) < 0.1 && abs(J5poserror) < 0.1)
					{

						if (i < qqq)
						{
							cout << i << endl;
							i++;
						}
						if (qqq == i)
						{

							return 0;
							break;
						}

					}

					if (GetAsyncKeyState(0x57))	//w
					{

						return 0;
						break;

					}

				}

			}

#pragma endregion compute torque PD控制器
#pragma endregion ccc==4

#pragma region ccc==5
#pragma region sliding-mode控制器
			if (ccc == 5)//注意第三軸速度回傳一定要有負號
			{

				//myYouBotManipulator->getArmGripper().close();
				//SLEEP_MILLISEC(3000);
				desiredJointAngle.angle = 0.0109 * radian;
				myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.8715* radian;
				myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

				desiredJointAngle.angle = -1.2 * radian;
				myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

				desiredJointAngle.angle = 1.5408 * radian;//0.0223
				myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

				desiredJointAngle.angle = 0.1201 * radian;
				myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

				//myYouBotManipulator->getArmGripper().open();
				LOG(info) << "unfold arm";
				SLEEP_MILLISEC(1000);

				int i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;
				cout << "理想軌跡總共時間" << endl;
				cin >> ttime;

				while (1)
				{

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();

					sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

					myYouBotManipulator->getJointData(EachJointVelocity1);
					svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
					svel3 = svel3*-1;//注意第三軸速度回傳一定要有負號
					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣


					ang_matrix.at<double>(0, 0) = sang1_cal;
					ang_matrix.at<double>(1, 0) = sang2_cal;
					ang_matrix.at<double>(2, 0) = sang3_cal;
					ang_matrix.at<double>(3, 0) = sang4_cal;
					ang_matrix.at<double>(4, 0) = sang5_cal;
					vel_matrix.at<double>(0, 0) = svel1;
					vel_matrix.at<double>(1, 0) = svel2;
					vel_matrix.at<double>(2, 0) = svel3;
					vel_matrix.at<double>(3, 0) = svel4;
					vel_matrix.at<double>(4, 0) = svel5;

					c_2 = cos(sang2_cal);
					c_3 = cos(sang3_cal);
					c_4 = cos(sang4_cal);
					c_5 = cos(sang5_cal);

					s_2 = sin(sang2_cal);
					s_3 = sin(sang3_cal);
					s_4 = sin(sang4_cal);
					s_5 = sin(sang5_cal);
					s_23 = sin(sang2_cal - sang3_cal);
					s_34 = sin(sang3_cal - sang4_cal);
					s2_2 = sin(2 * sang2_cal);
					s2_5 = sin(2 * sang5_cal);
					c_23 = cos(sang2_cal - sang3_cal);
					c_34 = cos(sang3_cal - sang4_cal);
					c_45 = cos(sang4_cal + sang5_cal);
					c_4_5 = cos(sang4_cal - sang5_cal);
					c_25 = cos(sang2_cal + sang5_cal);
					c_2_5 = cos(sang2_cal - sang5_cal);
					s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					s_223 = sin(2 * sang2_cal - sang3_cal);
					s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

					term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					term_3 = (3969783.0*c_4);
					term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					term_5 = (753.0*c_5);
					term_6 = (27027.0*s_4);
					term_7 = (20331.0*c_5*s_4);
					term_8 = (4557899.0*s_3*s_4);
					term_9 = (4557899.0*c_3*c_4);
					term_10 = (31031.0*c_3*s_4);
					term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					term_12 = (31031.0*c_4*s_3);
					term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					term_18 = (23343.0*c_3*c_5*s_4);
					term_19 = (23343.0*c_4*c_5*s_3);
					/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
#pragma region 科氏力矩陣

					C_matrix.at<double>(0, 0) = (30399.0*svel2*c2_234) / 125000000.0 - (30399.0*svel3*c2_234) / 125000000.0 + (30399.0*svel4*c2_234) / 125000000.0 + (1864633.0*svel2*s2_2) / 50000000.0 + (2259.0*svel5*s2_5) / 4000000000.0 + (31441279.0*svel2*s2_234) / 4000000000.0 - (31441279.0*svel3*s2_234) / 4000000000.0 + (31441279.0*svel4*s2_234) / 4000000000.0 - (23343.0*svel2*c_22345) / 400000000.0 - (20331.0*svel2*c_222345) / 400000000.0 + (23343.0*svel3*c_22345) / 800000000.0 + (20331.0*svel3*c_222345) / 400000000.0 - (23343.0*svel4*c_22345) / 800000000.0 - (20331.0*svel4*c_222345) / 800000000.0 - (23343.0*svel5*c_22345) / 800000000.0 - (20331.0*svel5*c_222345) / 800000000.0 - (24849.0*svel2*s_234_5) / 2000000000.0 + (24849.0*svel3*s_234_5) / 2000000000.0 - (24849.0*svel4*s_234_5) / 2000000000.0 + (24849.0*svel5*s_234_5) / 2000000000.0 + (2536699.0*svel2*s_223) / 50000000.0 + (10013911.0*svel2*s2_23) / 500000000.0 - (2536699.0*svel3*s_223) / 100000000.0 - (10013911.0*svel3*s2_23) / 500000000.0 - (4851957.0*svel2*c_234) / 1000000000.0 + (4851957.0*svel3*c_234) / 1000000000.0 - (4851957.0*svel4*c_234) / 1000000000.0 - (23343.0*svel3*c_345) / 800000000.0 + (23343.0*svel4*c_345) / 800000000.0 - (23343.0*svel5*c_345) / 800000000.0 + (33033.0*svel2*s_234) / 500000000.0 - (33033.0*svel3*s_234) / 500000000.0 + (33033.0*svel4*s_234) / 500000000.0 + (20331.0*svel4*c_45) / 800000000.0 + (20331.0*svel5*c_45) / 800000000.0 - (23343.0*svel2*c_2234_5) / 400000000.0 - (20331.0*svel2*c_22234_5) / 400000000.0 + (23343.0*svel3*c_2234_5) / 800000000.0 - (2259.0*svel2*c_2223245) / 62500000.0 + (20331.0*svel3*c_22234_5) / 400000000.0 - (23343.0*svel4*c_2234_5) / 800000000.0 + (2259.0*svel3*c_2223245) / 62500000.0 - (20331.0*svel4*c_22234_5) / 800000000.0 + (23343.0*svel5*c_2234_5) / 800000000.0 - (2259.0*svel4*c_2223245) / 62500000.0 + (20331.0*svel5*c_22234_5) / 800000000.0 - (2259.0*svel5*c_2223245) / 125000000.0 + (753.0*svel2*s_2223245) / 1000000000.0 - (753.0*svel3*s_2223245) / 1000000000.0 + (753.0*svel4*s_2223245) / 1000000000.0 + (753.0*svel5*s_2223245) / 2000000000.0 - (347127.0*svel2*c_2) / 20000000.0 - (27027.0*svel4*c_4) / 200000000.0 + (31031.0*svel2*c_2234) / 100000000.0 + (27027.0*svel2*c_22234) / 100000000.0 - (31031.0*svel3*c_2234) / 200000000.0 - (27027.0*svel3*c_22234) / 100000000.0 + (31031.0*svel4*c_2234) / 200000000.0 - (23343.0*svel3*c_34_5) / 800000000.0 + (27027.0*svel4*c_22234) / 200000000.0 + (23343.0*svel4*c_34_5) / 800000000.0 + (23343.0*svel5*c_34_5) / 800000000.0 - (2536699.0*svel3*s_3) / 100000000.0 - (3969783.0*svel4*s_4) / 400000000.0 + (753.0*svel5*s_5) / 1000000000.0 + (4557899.0*svel2*s_2234) / 200000000.0 + (3969783.0*svel2*s_22234) / 200000000.0 - (4557899.0*svel3*s_2234) / 400000000.0 - (3969783.0*svel3*s_22234) / 200000000.0 + (4557899.0*svel4*s_2234) / 400000000.0 + (3969783.0*svel4*s_22234) / 400000000.0 - (24849.0*svel2*s_2345) / 2000000000.0 + (24849.0*svel3*s_2345) / 2000000000.0 - (24849.0*svel4*s_2345) / 2000000000.0 - (24849.0*svel5*s_2345) / 2000000000.0 - (2700357.0*svel2*c_23) / 250000000.0 + (2700357.0*svel3*c_23) / 250000000.0 + (31031.0*svel3*c_34) / 200000000.0 - (31031.0*svel4*c_34) / 200000000.0 + (20331.0*svel4*c_4_5) / 800000000.0 - (20331.0*svel5*c_4_5) / 800000000.0 - (2259.0*svel2*c_222324_5) / 62500000.0 + (2259.0*svel3*c_222324_5) / 62500000.0 - (2259.0*svel4*c_222324_5) / 62500000.0 + (2259.0*svel5*c_222324_5) / 125000000.0 - (4557899.0*svel3*s_34) / 400000000.0 + (4557899.0*svel4*s_34) / 400000000.0 + (753.0*svel2*s_222324_5) / 1000000000.0 - (2259.0*svel2*s2_234_5) / 8000000000.0 - (2259.0*svel2*s2_2345) / 8000000000.0 - (753.0*svel3*s_222324_5) / 1000000000.0 + (2259.0*svel3*s2_234_5) / 8000000000.0 + (2259.0*svel3*s2_2345) / 8000000000.0 + (753.0*svel4*s_222324_5) / 1000000000.0 - (2259.0*svel4*s2_234_5) / 8000000000.0 - (2259.0*svel4*s2_2345) / 8000000000.0 - (753.0*svel5*s_222324_5) / 2000000000.0 + (2259.0*svel5*s2_234_5) / 8000000000.0 - (2259.0*svel5*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 1) = (30399.0*svel1*c2_234) / 125000000.0 + (1864633.0*svel1*s2_2) / 50000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 400000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 400000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 + (2536699.0*svel1*s_223) / 50000000.0 + (10013911.0*svel1*s2_23) / 500000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (20331.0*svel2*c_235) / 400000000.0 - (20331.0*svel3*c_235) / 400000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel2*c_25) / 400000000.0 - (23343.0*svel1*c_2234_5) / 400000000.0 - (20331.0*svel1*c_22234_5) / 400000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (347127.0*svel1*c_2) / 20000000.0 + (31031.0*svel1*c_2234) / 100000000.0 + (27027.0*svel1*c_22234) / 100000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 + (18867.0*svel2*s_2) / 100000000.0 + (4557899.0*svel1*s_2234) / 200000000.0 + (3969783.0*svel1*s_22234) / 200000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (2700357.0*svel1*c_23) / 250000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 - (441029.0*svel2*s_23) / 500000000.0 + (441029.0*svel3*s_23) / 500000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 2) = (23343.0*svel1*c_22345) / 800000000.0 - (31441279.0*svel1*s2_234) / 4000000000.0 - (30399.0*svel1*c2_234) / 125000000.0 + (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 400000000.0 - (2259.0*svel3*c_234_5) / 62500000.0 + (2259.0*svel4*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 - (753.0*svel2*s_234_5) / 1000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 + (753.0*svel3*s_234_5) / 1000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 - (753.0*svel4*s_234_5) / 1000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (2259.0*svel5*s_234_25) / 4000000000.0 - (2259.0*svel5*s_23425) / 4000000000.0 - (2536699.0*svel1*s_223) / 100000000.0 - (10013911.0*svel1*s2_23) / 500000000.0 + (4851957.0*svel1*c_234) / 1000000000.0 - (20331.0*svel2*c_235) / 400000000.0 - (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (33033.0*svel1*s_234) / 500000000.0 - (25433.0*svel2*s_234) / 100000000.0 + (25433.0*svel3*s_234) / 100000000.0 - (25433.0*svel4*s_234) / 100000000.0 + (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 400000000.0 + (2259.0*svel1*c_2223245) / 62500000.0 - (753.0*svel1*s_2223245) / 1000000000.0 - (31031.0*svel1*c_2234) / 200000000.0 - (27027.0*svel1*c_22234) / 100000000.0 + (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 - (20331.0*svel3*c_23_5) / 400000000.0 - (2536699.0*svel1*s_3) / 100000000.0 - (4557899.0*svel1*s_2234) / 400000000.0 - (3969783.0*svel1*s_22234) / 200000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 + (2700357.0*svel1*c_23) / 250000000.0 + (31031.0*svel1*c_34) / 200000000.0 + (2259.0*svel1*c_222324_5) / 62500000.0 + (441029.0*svel2*s_23) / 500000000.0 - (4557899.0*svel1*s_34) / 400000000.0 - (441029.0*svel3*s_23) / 500000000.0 - (753.0*svel1*s_222324_5) / 1000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 3) = (30399.0*svel1*c2_234) / 125000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (27027.0*svel1*c_4) / 200000000.0 + (31031.0*svel1*c_2234) / 200000000.0 + (27027.0*svel1*c_22234) / 200000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 - (3969783.0*svel1*s_4) / 400000000.0 + (4557899.0*svel1*s_2234) / 400000000.0 + (3969783.0*svel1*s_22234) / 400000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (31031.0*svel1*c_34) / 200000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 + (4557899.0*svel1*s_34) / 400000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(0, 4) = (2259.0*svel1*s2_5) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel5*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 - (753.0*svel5*s_234_5) / 1000000000.0 - (23343.0*svel1*c_345) / 800000000.0 - (20331.0*svel5*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel5*c_25) / 400000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 125000000.0 + (753.0*svel1*s_2223245) / 2000000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel5*c_23_5) / 400000000.0 + (753.0*svel1*s_5) / 1000000000.0 - (24849.0*svel5*s_5) / 1000000000.0 - (2259.0*svel5*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel5*s_2345) / 1000000000.0 - (20331.0*svel1*c_4_5) / 800000000.0 + (23343.0*svel5*c_2_5) / 400000000.0 + (2259.0*svel1*c_222324_5) / 125000000.0 - (753.0*svel1*s_222324_5) / 2000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(1, 0) = (svel1*((24849.0*s_2345) / 1000000000.0 + (2700357.0*c_23) / 125000000.0 + (2259.0*c_222324_5) / 31250000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (1864633.0*s2_2) / 25000000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 200000000.0 + (20331.0*c_222345) / 200000000.0 + (24849.0*s_234_5) / 1000000000.0 - (2536699.0*s_223) / 25000000.0 - (10013911.0*s2_23) / 250000000.0 + (4851957.0*c_234) / 500000000.0 - (33033.0*s_234) / 250000000.0 + (23343.0*c_2234_5) / 200000000.0 + (20331.0*c_22234_5) / 200000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (347127.0*c_2) / 10000000.0 - (31031.0*c_2234) / 50000000.0 - (27027.0*c_22234) / 50000000.0 - (4557899.0*s_2234) / 100000000.0 - (3969783.0*s_22234) / 100000000)) / 2 + (svel5*((23343.0*c_2_5) / 200000000.0 - (753.0*s_2345) / 500000000.0 + (2259.0*c_234_5) / 31250000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (23343.0*c_25) / 200000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
					C_matrix.at<double>(1, 1) = (svel4*((20331.0*c_4_5) / 200000000.0 - (31031.0*c_34) / 50000000.0 + (4557899.0*s_34) / 100000000.0 + (23343.0*c_345) / 200000000.0 + (20331.0*c_45) / 200000000.0 - (27027.0*c_4) / 50000000.0 + (23343.0*c_34_5) / 200000000.0 - (3969783.0*s_4) / 100000000)) / 2 - (svel3*((4557899.0*s_34) / 100000000.0 - (31031.0*c_34) / 50000000.0 + (23343.0*c_345) / 200000000.0 + (23343.0*c_34_5) / 200000000.0 + (2536699.0*s_3) / 25000000)) / 2 - (svel5*((2259.0*s2_5) / 1000000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000.0 + (23343.0*c_3*s_4*s_5) / 100000000.0 - (23343.0*c_4*s_3*s_5) / 100000000)) / 2;
					C_matrix.at<double>(1, 2) = (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 + (27027.0*svel4*c_4) / 100000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 + (23343.0*svel3*c_34_5) / 400000000.0 - (23343.0*svel4*c_34_5) / 400000000.0 - (23343.0*svel5*c_34_5) / 400000000.0 - (2536699.0*svel2*s_3) / 50000000.0 + (2536699.0*svel3*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_34) / 100000000.0 - (31031.0*svel3*c_34) / 100000000.0 + (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0 - (4557899.0*svel2*s_34) / 200000000.0 + (4557899.0*svel3*s_34) / 200000000.0 - (4557899.0*svel4*s_34) / 200000000.0;
					C_matrix.at<double>(1, 3) = (23343.0*svel2*c_345) / 400000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel3*c_345) / 400000000.0 + (23343.0*svel4*c_345) / 400000000.0 - (23343.0*svel5*c_345) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 - (27027.0*svel2*c_4) / 100000000.0 + (27027.0*svel3*c_4) / 100000000.0 - (27027.0*svel4*c_4) / 100000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 - (3969783.0*svel2*s_4) / 200000000.0 + (3969783.0*svel3*s_4) / 200000000.0 - (3969783.0*svel4*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_34) / 100000000.0 + (31031.0*svel3*c_34) / 100000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0 + (4557899.0*svel2*s_34) / 200000000.0 - (4557899.0*svel3*s_34) / 200000000.0 + (4557899.0*svel4*s_34) / 200000000.0;
					C_matrix.at<double>(1, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (23343.0*svel1*c_25) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 + (23343.0*svel1*c_2_5) / 400000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0;
					C_matrix.at<double>(2, 0) = (svel1*((4557899.0*s_34) / 200000000.0 - (2700357.0*c_23) / 125000000.0 - (31031.0*c_34) / 100000000.0 - (2259.0*c_222324_5) / 31250000.0 - (24849.0*s_2345) / 1000000000.0 + (753.0*s_222324_5) / 500000000.0 - (2259.0*s2_234_5) / 4000000000.0 - (2259.0*s2_2345) / 4000000000.0 + (30399.0*c2_234) / 62500000.0 + (31441279.0*s2_234) / 2000000000.0 - (23343.0*c_22345) / 400000000.0 - (20331.0*c_222345) / 200000000.0 - (24849.0*s_234_5) / 1000000000.0 + (2536699.0*s_223) / 50000000.0 + (10013911.0*s2_23) / 250000000.0 - (4851957.0*c_234) / 500000000.0 + (23343.0*c_345) / 400000000.0 + (33033.0*s_234) / 250000000.0 - (23343.0*c_2234_5) / 400000000.0 - (20331.0*c_22234_5) / 200000000.0 - (2259.0*c_2223245) / 31250000.0 + (753.0*s_2223245) / 500000000.0 + (31031.0*c_2234) / 100000000.0 + (27027.0*c_22234) / 50000000.0 + (23343.0*c_34_5) / 400000000.0 + (2536699.0*s_3) / 50000000.0 + (4557899.0*s_2234) / 200000000.0 + (3969783.0*s_22234) / 100000000)) / 2 - (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
					C_matrix.at<double>(2, 1) = (2259.0*svel5*s2_5) / 2000000000.0 + (27027.0*svel4*c_4) / 100000000.0 + (2536699.0*svel2*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel4*c_4*c_5) / 200000000.0 - (4557899.0*svel2*c_3*s_4) / 200000000.0 + (4557899.0*svel2*c_4*s_3) / 200000000.0 - (31031.0*svel2*s_3*s_4) / 100000000.0 + (20331.0*svel5*s_4*s_5) / 200000000.0 + (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 + (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
					C_matrix.at<double>(2, 2) = -(svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
					C_matrix.at<double>(2, 3) = (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
					C_matrix.at<double>(2, 4) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 - (2259.0*svel5*c_5) / 31250000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0;
					C_matrix.at<double>(3, 0) = (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20002259.0*s_234) / 1000000000.0 + (2259.0*c_2345) / 31250000)) / 2 + (svel1*((24849.0*s_2345) / 1000000000.0 + (31031.0*c_34) / 100000000.0 - (20331.0*c_4_5) / 400000000.0 + (2259.0*c_222324_5) / 31250000.0 - (4557899.0*s_34) / 200000000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 400000000.0 + (20331.0*c_222345) / 400000000.0 + (24849.0*s_234_5) / 1000000000.0 + (4851957.0*c_234) / 500000000.0 - (23343.0*c_345) / 400000000.0 - (33033.0*s_234) / 250000000.0 - (20331.0*c_45) / 400000000.0 + (23343.0*c_2234_5) / 400000000.0 + (20331.0*c_22234_5) / 400000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (27027.0*c_4) / 100000000.0 - (31031.0*c_2234) / 100000000.0 - (27027.0*c_22234) / 100000000.0 - (23343.0*c_34_5) / 400000000.0 + (3969783.0*s_4) / 200000000.0 - (4557899.0*s_2234) / 200000000.0 - (3969783.0*s_22234) / 200000000)) / 2;
					C_matrix.at<double>(3, 1) = (27027.0*svel2*c_4) / 100000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (27027.0*svel3*c_4) / 100000000.0 + (3969783.0*svel2*s_4) / 200000000.0 - (3969783.0*svel3*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel2*c_4*c_5) / 200000000.0 + (20331.0*svel3*c_4*c_5) / 200000000.0 + (4557899.0*svel2*c_3*s_4) / 200000000.0 - (4557899.0*svel2*c_4*s_3) / 200000000.0 + (31031.0*svel2*s_3*s_4) / 100000000.0 - (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 - (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
					C_matrix.at<double>(3, 2) = (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((753.0*s_5) / 250000000.0 - (2259.0*c_5*s_5) / 500000000)) / 2;
					C_matrix.at<double>(3, 3) = -(753.0*svel5*(3.0*s2_5 - 4.0*s_5)) / 2000000000.0;
					C_matrix.at<double>(3, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0;
					C_matrix.at<double>(4, 0) = (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel1*s2_5) / 4000000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (20331.0*svel2*c_235) / 400000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 - (23343.0*svel2*c_25) / 400000000.0 - (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 + (2259.0*svel1*c_2223245) / 125000000.0 - (753.0*svel1*s_2223245) / 2000000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 - (753.0*svel1*s_5) / 1000000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 125000000.0 + (753.0*svel1*s_222324_5) / 2000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
					C_matrix.at<double>(4, 1) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 + (23343.0*svel2*c_345) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (23343.0*svel1*c_25) / 400000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 - (23343.0*svel1*c_2_5) / 400000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0;
					C_matrix.at<double>(4, 2) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0;
					C_matrix.at<double>(4, 3) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0;
					C_matrix.at<double>(4, 4) = 0.0;
#pragma endregion 科氏力矩陣
					/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
					/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////
#pragma region 重力矩陣


					N_matrix.at<double>(0, 0) = 0.0;
					N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;

#pragma endregion 重力矩陣
					/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////

					/////////////////////////////////////////////慣性矩陣/////////////////////////////
#pragma region 慣性矩陣

					M_matrix.at<double>(0, 0) = (4557899.0*cos(sang3_cal - sang4_cal)) / 200000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 1000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal)) / 8000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal)) / 8000000000.0 - (2700357.0*sin(sang2_cal - sang3_cal)) / 125000000.0 + (31031.0*sin(sang3_cal - sang4_cal)) / 100000000.0 + (20331.0*sin(sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 62500000.0 - (1864633.0*cos(2 * sang2_cal)) / 50000000.0 - (2259.0*cos(2 * sang5_cal)) / 4000000000.0 - (31441279.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 4000000000.0 + (30399.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 125000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (2536699.0*cos(2 * sang2_cal - sang3_cal)) / 50000000.0 - (10013911.0*cos(2 * sang2_cal - 2 * sang3_cal)) / 500000000.0 - (33033.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 250000000.0 - (4851957.0*sin(sang2_cal - sang3_cal + sang4_cal)) / 500000000.0 - (23343.0*sin(sang3_cal - sang4_cal + sang5_cal)) / 400000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 1000000000.0 + (20331.0*sin(sang4_cal + sang5_cal)) / 400000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 62500000.0 + (2536699.0*c_3) / 50000000.0 + term_3 / 200000000.0 - term_5 / 500000000.0 - (4557899.0*cos(2 * sang2_cal - sang3_cal + sang4_cal)) / 200000000.0 - (3969783.0*cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 200000000.0 - (347127.0*s_2) / 10000000.0 - term_6 / 100000000.0 + (31031.0*sin(2 * sang2_cal - sang3_cal + sang4_cal)) / 100000000.0 + (27027.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 100000000.0 - (23343.0*sin(sang3_cal - sang4_cal - sang5_cal)) / 400000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 1000000000.0 + 105207843 / 800000000.0;
					M_matrix.at<double>(0, 1) = term_1 + (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_16 / 400000000.0 + (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 - (18867.0*c_2) / 100000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 + term_11 / 1000000000.0;
					M_matrix.at<double>(0, 2) = term_2 - (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - term_1 - term_13 / 4000000000.0 + term_14 / 4000000000.0 + term_15 / 62500000.0 + term_17 / 100000000.0 - term_16 / 400000000.0 + (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
					M_matrix.at<double>(0, 3) = term_1 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_11 / 1000000000.0;
					M_matrix.at<double>(0, 4) = (20002259.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 1000000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 - term_15 / 62500000.0 - term_1 - term_16 / 400000000.0 - (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 + (24849.0*c_5) / 1000000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
					M_matrix.at<double>(1, 0) = M_matrix.at<double>(0, 1);
					M_matrix.at<double>(1, 1) = (2536699.0*c_3) / 25000000.0 + term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_9 / 100000000.0 - term_10 / 50000000.0 + term_12 / 50000000.0 + term_7 / 100000000.0 + term_8 / 100000000.0 + term_4 + term_18 / 100000000.0 - term_19 / 100000000.0 + 180370741 / 1000000000.0;
					M_matrix.at<double>(1, 2) = term_5 / 250000000.0 - term_3 / 100000000.0 - (2536699.0*c_3) / 50000000.0 + term_6 / 50000000.0 - term_9 / 200000000.0 + term_10 / 100000000.0 - term_12 / 100000000.0 - term_7 / 100000000.0 - term_8 / 200000000.0 - term_4 - term_18 / 200000000.0 + term_19 / 200000000.0 - 95785421 / 1000000000.0;
					M_matrix.at<double>(1, 3) = term_3 / 200000000.0 - term_5 / 250000000.0 - term_6 / 100000000.0 + term_9 / 200000000.0 - term_10 / 100000000.0 + term_12 / 100000000.0 + term_7 / 200000000.0 + term_8 / 200000000.0 + term_4 + term_18 / 200000000.0 - term_19 / 200000000.0 + 45729777 / 1000000000.0;
					M_matrix.at<double>(1, 4) = (753.0*s_5*(155.0*cos(sang3_cal - sang4_cal) + 135.0*c_4 + 96)) / 1000000000.0;
					M_matrix.at<double>(2, 0) = M_matrix.at<double>(0, 2);
					M_matrix.at<double>(2, 1) = M_matrix.at<double>(1, 2);
					M_matrix.at<double>(2, 2) = term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_7 / 100000000.0 + term_4 + 95785421 / 1000000000.0;
					M_matrix.at<double>(2, 3) = term_5 / 250000000.0 - term_3 / 200000000.0 + term_6 / 100000000.0 - term_7 / 200000000.0 - term_4 - 45729777 / 1000000000.0;
					M_matrix.at<double>(2, 4) = -(2259.0*s_5*(45.0*c_4 + 32)) / 1000000000.0;
					M_matrix.at<double>(3, 0) = M_matrix.at<double>(0, 3);
					M_matrix.at<double>(3, 1) = M_matrix.at<double>(1, 3);
					M_matrix.at<double>(3, 2) = M_matrix.at<double>(2, 3);
					M_matrix.at<double>(3, 3) = term_4 - term_5 / 250000000.0 + 45729777.0 / 1000000000.0;
					M_matrix.at<double>(3, 4) = (2259.0*s_5) / 31250000.0;
					M_matrix.at<double>(4, 0) = M_matrix.at<double>(0, 4);
					M_matrix.at<double>(4, 1) = M_matrix.at<double>(1, 4);
					M_matrix.at<double>(4, 2) = M_matrix.at<double>(2, 4);
					M_matrix.at<double>(4, 3) = M_matrix.at<double>(3, 4);
					M_matrix.at<double>(4, 4) = 20002259.0 / 1000000000.0;
#pragma endregion 慣性矩陣


					///////所有角度都先以動力學模型的角度轉換為基準
#pragma region 期望點軌跡分割
					if (count == 0)
					{
						for (total_time = 0; total_time < ttime + 1; total_time++)
						{
							//desired_ang.at<double>(0, 0) = ((-2 / pow(ttime, 3))*((-2.5378) - (-2.9387))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*pow(total_time, 2)) + (-2.9387);
							//desired_ang.at<double>(1, 0) = ((-2 / pow(ttime, 3))*((-1.1235) - (-0.4045))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.1235) - (-0.4045))*pow(total_time, 2)) + (-0.4045);
							//desired_ang.at<double>(2, 0) = ((-2 / pow(ttime, 3))*(-2.2482 - (-1.6482))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*(-2.2482 - (-1.6482))*pow(total_time, 2)) + ((-1.6482));
							//desired_ang.at<double>(3, 0) = ((-2 / pow(ttime, 3))*((-1.633) - (-0.789))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.633) - (-0.789))*pow(total_time, 2)) + (-0.789);
							//desired_ang.at<double>(4, 0) = ((-2 / pow(ttime, 3))*((-1.63811) - (-2.8033))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*pow(total_time, 2)) + (-2.8033);



							//desired_vel.at<double>(0, 0) = (3 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*total_time;
							//desired_vel.at<double>(1, 0) = (3 * (-2 / pow(ttime, 3))*((-1.1235) - (-0.4045)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.1235) - (-0.4045))*total_time;
							//desired_vel.at<double>(2, 0) = (3 * (-2 / pow(ttime, 3))*(-2.2482 - (-1.6482)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*(-2.2482 - (-1.6482))*total_time;
							//desired_vel.at<double>(3, 0) = (3 * (-2 / pow(ttime, 3))*((-1.633) - (-0.789)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.633) - (-0.789))*total_time;
							//desired_vel.at<double>(4, 0) = (3 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*total_time;

							//desired_acc.at<double>(0, 0) = (6 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387));
							//desired_acc.at<double>(1, 0) = (6 * (-2 / pow(ttime, 3))*((-1.1235) - (-0.4045)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.1235) - (-0.4045));
							//desired_acc.at<double>(2, 0) = (6 * (-2 / pow(ttime, 3))*(-2.2482 - (-1.6482)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*(-2.2482 - (-1.6482));
							//desired_acc.at<double>(3, 0) = (6 * (-2 / pow(ttime, 3))*((-1.633) - (-0.789)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.633) - (-0.789));
							//desired_acc.at<double>(4, 0) = (6 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033));


							desired_ang.at<double>(0, 0) = sang1_cal;// ((-2 / pow(ttime, 3))*((-2.5378) - (-2.9387))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*pow(total_time, 2)) + (-2.9387);
							desired_ang.at<double>(1, 0) = ((-2 / pow(ttime, 3))*((-0.982) - (-0.263))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-0.982) - (-0.263))*pow(total_time, 2)) + (-0.263);
							desired_ang.at<double>(2, 0) = ((-2 / pow(ttime, 3))*(-1.948 - (-1.348))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*(-1.948 - (-1.348))*pow(total_time, 2)) + ((-1.348));
							desired_ang.at<double>(3, 0) = ((-2 / pow(ttime, 3))*((-1.092) - (-0.248))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.092) - (-0.248))*pow(total_time, 2)) + (-0.248);
							desired_ang.at<double>(4, 0) = sang5_cal; //((-2 / pow(ttime, 3))*((-1.63811) - (-2.8033))*pow(total_time, 3)) + ((3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*pow(total_time, 2)) + (-2.8033);



							desired_vel.at<double>(0, 0) = 0;// (3 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387))*total_time;
							desired_vel.at<double>(1, 0) = (3 * (-2 / pow(ttime, 3))*((-0.982) - (-0.263)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-0.982) - (-0.263))*total_time;
							desired_vel.at<double>(2, 0) = (3 * (-2 / pow(ttime, 3))*(-1.948 - (-1.348)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*(-1.948 - (-1.348))*total_time;
							desired_vel.at<double>(3, 0) = (3 * (-2 / pow(ttime, 3))*((-1.092) - (-0.248)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.092) - (-0.248))*total_time;
							desired_vel.at<double>(4, 0) = 0;// (3 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 2) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033))*total_time;

							desired_acc.at<double>(0, 0) = 0;// (6 * (-2 / pow(ttime, 3))*((-2.5378) - (-2.9387)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-2.5378) - (-2.9387));
							desired_acc.at<double>(1, 0) = (6 * (-2 / pow(ttime, 3))*((-0.982) - (-0.263)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-0.982) - (-0.263));
							desired_acc.at<double>(2, 0) = (6 * (-2 / pow(ttime, 3))*(-1.948 - (-1.348)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*(-1.948 - (-1.348));
							desired_acc.at<double>(3, 0) = (6 * (-2 / pow(ttime, 3))*((-1.092) - (-0.248)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.092) - (-0.248));
							desired_acc.at<double>(4, 0) = 0;// (6 * (-2 / pow(ttime, 3))*((-1.63811) - (-2.8033)))*pow(total_time, 1) + 2 * (3 / pow(ttime, 2))*((-1.63811) - (-2.8033));


							cutang << "第一軸   " << desired_ang.at<double>(0, 0) << "    第二軸  " << desired_ang.at<double>(1, 0) << "    第三軸  " << desired_ang.at<double>(2, 0) << "   第四軸  " << desired_ang.at<double>(3, 0) << "   第五軸  " << desired_ang.at<double>(4, 0) << endl;
							cutvel << "第一軸   " << desired_vel.at<double>(0, 0) << "    第二軸  " << desired_vel.at<double>(1, 0) << "    第三軸  " << desired_vel.at<double>(2, 0) << "   第四軸  " << desired_vel.at<double>(3, 0) << "   第五軸  " << desired_vel.at<double>(4, 0) << endl;
							cutacc << "第一軸   " << desired_acc.at<double>(0, 0) << "    第二軸  " << desired_acc.at<double>(1, 0) << "    第三軸  " << desired_acc.at<double>(2, 0) << "   第四軸  " << desired_acc.at<double>(3, 0) << "   第五軸  " << desired_acc.at<double>(4, 0) << endl;



							temp_ang.j1 = desired_ang.at<double>(0, 0);
							temp_ang.j2 = desired_ang.at<double>(1, 0);
							temp_ang.j3 = desired_ang.at<double>(2, 0);
							temp_ang.j4 = desired_ang.at<double>(3, 0);
							temp_ang.j5 = desired_ang.at<double>(4, 0);
							temp_point.push_back(temp_ang);

							veltemp.j1 = desired_vel.at<double>(0, 0);
							veltemp.j2 = desired_vel.at<double>(1, 0);
							veltemp.j3 = desired_vel.at<double>(2, 0);
							veltemp.j4 = desired_vel.at<double>(3, 0);
							veltemp.j5 = desired_vel.at<double>(4, 0);
							vel_test.push_back(veltemp);

							acctemp.j1 = desired_acc.at<double>(0, 0);
							acctemp.j2 = desired_acc.at<double>(1, 0);
							acctemp.j3 = desired_acc.at<double>(2, 0);
							acctemp.j4 = desired_acc.at<double>(3, 0);
							acctemp.j5 = desired_acc.at<double>(4, 0);
							acc_test.push_back(acctemp);

						}
					}
					qqq = vel_test.size();

					desired_tempang.at<double>(0, 0) = temp_point[i1].j1;
					desired_tempang.at<double>(1, 0) = temp_point[i2].j2;
					desired_tempang.at<double>(2, 0) = temp_point[i3].j3;
					desired_tempang.at<double>(3, 0) = temp_point[i4].j4;
					desired_tempang.at<double>(4, 0) = temp_point[i5].j5;

					desired_tempvel.at<double>(0, 0) = vel_test[i1].j1;
					desired_tempvel.at<double>(1, 0) = vel_test[i2].j2;
					desired_tempvel.at<double>(2, 0) = vel_test[i3].j3;
					desired_tempvel.at<double>(3, 0) = vel_test[i4].j4;
					desired_tempvel.at<double>(4, 0) = vel_test[i5].j5;

					desired_tempacc.at<double>(0, 0) = acc_test[i1].j1;
					desired_tempacc.at<double>(1, 0) = acc_test[i2].j2;
					desired_tempacc.at<double>(2, 0) = acc_test[i3].j3;
					desired_tempacc.at<double>(3, 0) = acc_test[i4].j4;
					desired_tempacc.at<double>(4, 0) = acc_test[i5].j5;


#pragma endregion 期望點軌跡分割

#pragma region lambda-kk-qq
					lambda = 0;
					kk = 0;
					///////////////////單軸移動
					//lambda.at<double>(0, 0) = 256;
					//lambda.at<double>(1, 1) = 33.3;//56.43//56.47//56.53//34//33.3
					//lambda.at<double>(2, 2) = 101;//128//125//135//134.5
					//lambda.at<double>(3, 3) = 63.6;//62.5
					//lambda.at<double>(4, 4) = 95;
					///////////////////////2-3-4軸移動

					lambda.at<double>(0, 0) = 256;
					lambda.at<double>(1, 1) = 53;//70最低了//75//70//40
					lambda.at<double>(2, 2) = 28;//158.702會抖//45//20//35
					lambda.at<double>(3, 3) = 20.3;//62.5//58//75//62.5//15
					lambda.at<double>(4, 4) = 95;

					//kk.at<double>(0, 0) = 0.1;//0.1
					//kk.at<double>(1, 1) = 70;//0.8//12
					//kk.at<double>(2, 2) = 25;//1//7
					//kk.at<double>(3, 3) = 1;//0.2
					//kk.at<double>(4, 4) = 0.1;//0.1



					kk.at<double>(0, 0) = 0.1;
					kk.at<double>(1, 1) = 5.5;//5.8
					kk.at<double>(2, 2) = 3;//5.8
					kk.at<double>(3, 3) = 1.67;
					kk.at<double>(4, 4) = 0.1;
					qq = 10;



#pragma endregion lambda-kk-qq
					joint_poserror = ang_matrix - desired_tempang;
					joint_poserror_dot = vel_matrix - desired_tempvel;

					J1poserror = joint_poserror.at<double>(0, 0);
					J2poserror = joint_poserror.at<double>(1, 0);
					J3poserror = joint_poserror.at<double>(2, 0);
					J4poserror = joint_poserror.at<double>(3, 0);
					J5poserror = joint_poserror.at<double>(4, 0);

					//if (abs(J1poserror) < 0.07)
					//{
					//	J1poserror = 0;
					//}
					//if (abs(J2poserror) < 0.07)
					//{
					//	J2poserror = 0;
					//}
					//if (abs(J3poserror) < 0.07)
					//{
					//	J3poserror = 0;
					//}
					//if (abs(J4poserror) < 0.07)
					//{
					//	J4poserror = 0;
					//}
					//if (abs(J5poserror) < 0.07)
					//{
					//	J5poserror = 0;
					//}

					if (sang3_cal > -1)//第三軸保護措施 做實驗前須檢查是否符合自己所需
					{
						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						tau_matrix = N_matrix;
#pragma endregion 重力補償test



						desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						LOG(info) << "torque test1";
						desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						LOG(info) << "torque test2";
						desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						LOG(info) << "torque test3";
						desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						LOG(info) << "torque test4";
						desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						LOG(info) << "torque test5";

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣

					}



					J1velerror = joint_poserror_dot.at<double>(0, 0);
					J2velerror = joint_poserror_dot.at<double>(1, 0);
					J3velerror = joint_poserror_dot.at<double>(2, 0);
					J4velerror = joint_poserror_dot.at<double>(3, 0);
					J5velerror = joint_poserror_dot.at<double>(4, 0);
					ss = joint_poserror_dot + lambda * joint_poserror;

					for (int i = 0; i < 5; i++)
					{
						qaz = ss.at<double>(i, 0);
						if (qaz > qq)
						{
							sw.at<double>(i, 0) = 1;
						}
						if (qaz < qq)
						{
							sw.at<double>(i, 0) = -1;
						}
						if (abs(qaz) <= qq)
						{
							sw.at<double>(i, 0) = qaz / qq;
						}

					}
					N_matrix.at<double>(1, 0) = N_matrix.at<double>(1, 0)*0.6;
					N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*0.75*0.68;
					N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0)*0.7;
					adj = 0;
					adj.at<double>(0, 0) = 1;//1	
					adj.at<double>(1, 1) = 1;//0.9	
					adj.at<double>(2, 2) = 1;//1.05	
					adj.at<double>(3, 3) = 1;//1.05	
					adj.at<double>(4, 4) = 1;//1
					//adj2 = 0;
					//adj2.at<double>(0, 0) = 1;//1
					//adj2.at<double>(1, 1) = 0.7;//0.7
					//adj2.at<double>(2, 2) = 1.3;//1.1
					//adj2.at<double>(3, 3) = 0.7;//0.7
					//adj2.at<double>(4, 4) = 1;//1


					//Ueq = M_matrix*desired_tempacc + adj2*C_matrix*vel_matrix + N_matrix + M_matrix* (-lambda*joint_poserror_dot);
					//Ueq = adj*(M_matrix*desired_tempacc + C_matrix*vel_matrix + N_matrix) + M_matrix* (-lambda*joint_poserror_dot);
					Ueq = adj*(M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix) + N_matrix;
					//Ueq = (M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix) + N_matrix;//基本
					Ueq.at<double>(2, 0) = Ueq.at<double>(2, 0)*-1;

					Usw = -kk *sw;
					Usw.at<double>(2, 0) = Usw.at<double>(2, 0)*-1;
					Utemp = Usw;
					tau_matrix = Ueq + Usw;
					check1 = adj*(M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix);//檢查用
					ch1 = check1.at<double>(0, 0);
					ch2 = check1.at<double>(1, 0);
					ch3 = check1.at<double>(2, 0);
					ch4 = check1.at<double>(3, 0);
					ch5 = check1.at<double>(4, 0);
					check2 = M_matrix*desired_tempacc;
					ch11 = check2.at<double>(0, 0);
					ch22 = check2.at<double>(1, 0);
					ch33 = check2.at<double>(2, 0);
					ch44 = check2.at<double>(3, 0);
					ch55 = check2.at<double>(4, 0);


					M11 = M_matrix.at<double>(0, 0); M12 = M_matrix.at<double>(0, 1); M13 = M_matrix.at<double>(0, 2); M14 = M_matrix.at<double>(0, 3); M15 = M_matrix.at<double>(0, 4);
					M21 = M_matrix.at<double>(1, 0); M22 = M_matrix.at<double>(1, 1); M23 = M_matrix.at<double>(1, 2); M24 = M_matrix.at<double>(1, 3); M25 = M_matrix.at<double>(1, 4);
					M31 = M_matrix.at<double>(2, 0); M32 = M_matrix.at<double>(2, 1); M33 = M_matrix.at<double>(2, 2); M34 = M_matrix.at<double>(2, 3); M35 = M_matrix.at<double>(2, 4);
					M41 = M_matrix.at<double>(3, 0); M42 = M_matrix.at<double>(3, 1); M43 = M_matrix.at<double>(3, 2); M44 = M_matrix.at<double>(3, 3); M45 = M_matrix.at<double>(3, 4);
					M51 = M_matrix.at<double>(4, 0); M52 = M_matrix.at<double>(4, 1); M53 = M_matrix.at<double>(4, 2); M54 = M_matrix.at<double>(4, 3); M55 = M_matrix.at<double>(4, 4);

					FF1 = Ueq.at<double>(0, 0);
					FF2 = Ueq.at<double>(1, 0);
					FF3 = Ueq.at<double>(2, 0);
					FF4 = Ueq.at<double>(3, 0);
					FF5 = Ueq.at<double>(4, 0);

					FB1 = Usw.at<double>(0, 0);
					FB2 = Usw.at<double>(1, 0);
					FB3 = Usw.at<double>(2, 0);
					FB4 = Usw.at<double>(3, 0);
					FB5 = Usw.at<double>(4, 0);



					tau_matrix.at<double>(0, 0) = tau_matrix.at<double>(0, 0);
					tau_matrix.at<double>(1, 0) = tau_matrix.at<double>(1, 0);
					tau_matrix.at<double>(2, 0) = tau_matrix.at<double>(2, 0);
					tau_matrix.at<double>(3, 0) = tau_matrix.at<double>(3, 0);
					tau_matrix.at<double>(4, 0) = tau_matrix.at<double>(4, 0);



					N1 = tau_matrix.at<double>(0, 0);
					N2 = tau_matrix.at<double>(1, 0);
					N3 = tau_matrix.at<double>(2, 0);
					N4 = tau_matrix.at<double>(3, 0);
					N5 = tau_matrix.at<double>(4, 0);
					cout << "N1" << N1 << endl;
					cout << "N2" << N2 << endl;
					cout << "N3" << N3 << endl;
					cout << "N4" << N4 << endl;
					cout << "N5" << N5 << endl;
					cout << "切割幾個" << i << endl;
#pragma region 限制力矩
					if (tau_matrix.at<double>(0, 0) > 3)
					{
						tau_matrix.at<double>(0, 0) = 3;
					}
					if (tau_matrix.at<double>(0, 0) < -3)
					{
						tau_matrix.at<double>(0, 0) = -3;
					}
					if (tau_matrix.at<double>(1, 0) > 5)
					{
						tau_matrix.at<double>(1, 0) = 5;
					}
					if (tau_matrix.at<double>(1, 0) < -5)
					{
						tau_matrix.at<double>(1, 0) = -5;
					}
					if (tau_matrix.at<double>(2, 0) > 6)
					{
						tau_matrix.at<double>(2, 0) = 6;
					}
					if (tau_matrix.at<double>(2, 0) < -6)
					{
						tau_matrix.at<double>(2, 0) = -6;
					}
					if (tau_matrix.at<double>(3, 0) > 4.0)
					{
						tau_matrix.at<double>(3, 0) = 4.0;
					}
					if (tau_matrix.at<double>(3, 0) < -4.0)
					{
						tau_matrix.at<double>(3, 0) = -4.0;
					}
					if (tau_matrix.at<double>(4, 0) > 1.0)
					{
						tau_matrix.at<double>(4, 0) = 1.0;
					}
					if (tau_matrix.at<double>(4, 0) < -1.0)
					{
						tau_matrix.at<double>(4, 0) = -1.0;
					}


					cout << "第一軸   " << ss.at<double>(0, 0) << "    第二軸  " << ss.at<double>(1, 0) << "    第三軸  " << ss.at<double>(2, 0) << "   第四軸  " << ss.at<double>(3, 0) << "   第五軸  " << ss.at<double>(4, 0) << endl;
					N1 = tau_matrix.at<double>(0, 0);
					N2 = tau_matrix.at<double>(1, 0);
					N3 = tau_matrix.at<double>(2, 0);
					N4 = tau_matrix.at<double>(3, 0);
					N5 = tau_matrix.at<double>(4, 0);
#pragma endregion 限制力矩

#pragma region 力矩輸出
					//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//LOG(info) << "torque test1";
					desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					LOG(info) << "torque test2";
					desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					LOG(info) << "torque test3";
					desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					LOG(info) << "torque test4";
					//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//LOG(info) << "torque test5";
#pragma endregion 力矩輸出

					myYouBotManipulator->getJointData(EachJointTorque1);
					stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸  " << stau5 << endl;
					Tau1 << stau1 << endl;		Tau2 << stau2 << endl;   Tau3 << stau3 << endl;   Tau4 << stau4 << endl;  Tau5 << stau5 << endl;
					nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸  " << sang5_cal << endl;
					jointdesiredvel << "第一軸   " << desired_tempvel.at<double>(0.0) << "   第二軸  " << desired_tempvel.at<double>(1.0) << "   第三軸  " << desired_tempvel.at<double>(2.0) << "   第四軸  " << desired_tempvel.at<double>(3.0) << "   第五軸  " << desired_tempvel.at<double>(4.0) << endl;
					jointdesiredang << "第一軸   " << desired_tempang.at<double>(0.0) << "   第二軸  " << desired_tempang.at<double>(1.0) << "   第三軸  " << desired_tempang.at<double>(2.0) << "   第四軸  " << desired_tempang.at<double>(3.0) << "   第五軸  " << desired_tempang.at<double>(4.0) << endl;
					Usww << "第一軸   " << Utemp.at<double>(0.0) << "   第二軸  " << Utemp.at<double>(1.0) << "   第三軸  " << Utemp.at<double>(2.0) << "   第四軸  " << Utemp.at<double>(3.0) << "   第五軸  " << Utemp.at<double>(4.0) << endl;
					Jointx << "第一軸   " << sang1 << "   第二軸  " << sang2 << "   第三軸  " << sang3 << "   第四軸  " << sang4 << "   第五軸  " << sang5 << endl;
					joint1 << sang1 << endl;	joint2 << sang2 << endl;	joint3 << sang3 << endl;	joint4 << sang4 << endl;	joint5 << sang5 << endl;
					CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
					jointerror << "第一軸   " << J1poserror << "    第二軸  " << J2poserror << "    第三軸  " << J3poserror << "   第四軸  " << J4poserror << "   第五軸  " << J5poserror << endl;
					jointvelerror << "第一軸   " << J1velerror << "    第二軸  " << J2velerror << "    第三軸  " << J3velerror << "   第四軸  " << J4velerror << "   第五軸  " << J5velerror << endl;
					JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸" << svel5 << endl;
					sliding << "第一軸   " << ss.at<double>(0, 0) << "    第二軸  " << ss.at<double>(1, 0) << "    第三軸  " << ss.at<double>(2, 0) << "   第四軸  " << ss.at<double>(3, 0) << "   第五軸  " << ss.at<double>(4, 0) << endl;
					Ueqq << "第一軸   " << check1.at<double>(0.0) << "   第二軸  " << check1.at<double>(1.0) << "   第三軸  " << check1.at<double>(2.0) << "   第四軸  " << check1.at<double>(3.0) << "   第五軸  " << check1.at<double>(4.0) << endl;
					QAZ << i4 << endl;
					count++;


					if (J1poserror < 0.01)
					{
						if (i1 < qqq - 1)
						{
							cout << i1 << endl;
							i1++;
						}
					}
					if (i1 == qqq)
					{
						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						tau_matrix = N_matrix;
#pragma endregion 重力補償test



						desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						LOG(info) << "torque test1";
						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//LOG(info) << "torque test2";
						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//LOG(info) << "torque test3";
						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//LOG(info) << "torque test4";
						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						//LOG(info) << "torque test5";

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣



					}
					if (J2poserror < 0.01)
					{

						if (i2 < qqq - 1)
						{
							cout << i2 << endl;
							i2++;
						}

					}

					//					if (i2 == qqq)
					//					{
					//
					//						myYouBotManipulator->getJointData(EachJointAngle1);
					//						sang1 = EachJointAngle1[0].angle.value();
					//						sang2 = EachJointAngle1[1].angle.value();
					//						sang3 = EachJointAngle1[2].angle.value();
					//						sang4 = EachJointAngle1[3].angle.value();
					//						sang5 = EachJointAngle1[4].angle.value();
					//
					//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					//
					//
					//						c_2 = cos(sang2_cal);
					//						c_3 = cos(sang3_cal);
					//						c_4 = cos(sang4_cal);
					//						c_5 = cos(sang5_cal);
					//
					//						s_2 = sin(sang2_cal);
					//						s_3 = sin(sang3_cal);
					//						s_4 = sin(sang4_cal);
					//						s_5 = sin(sang5_cal);
					//						s_23 = sin(sang2_cal - sang3_cal);
					//						s_34 = sin(sang3_cal - sang4_cal);
					//						s2_2 = sin(2 * sang2_cal);
					//						s2_5 = sin(2 * sang5_cal);
					//						c_23 = cos(sang2_cal - sang3_cal);
					//						c_34 = cos(sang3_cal - sang4_cal);
					//						c_45 = cos(sang4_cal + sang5_cal);
					//						c_4_5 = cos(sang4_cal - sang5_cal);
					//						c_25 = cos(sang2_cal + sang5_cal);
					//						c_2_5 = cos(sang2_cal - sang5_cal);
					//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					//						s_223 = sin(2 * sang2_cal - sang3_cal);
					//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
					//
					//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					//						term_3 = (3969783.0*c_4);
					//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					//						term_5 = (753.0*c_5);
					//						term_6 = (27027.0*s_4);
					//						term_7 = (20331.0*c_5*s_4);
					//						term_8 = (4557899.0*s_3*s_4);
					//						term_9 = (4557899.0*c_3*c_4);
					//						term_10 = (31031.0*c_3*s_4);
					//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					//						term_12 = (31031.0*c_4*s_3);
					//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					//						term_18 = (23343.0*c_3*c_5*s_4);
					//						term_19 = (23343.0*c_4*c_5*s_3);
					//
					//
					//						N_matrix.at<double>(0, 0) = 0.0;
					//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
					//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
					//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
					//						tau_matrix = N_matrix;
					//#pragma endregion 重力補償test
					//
					//
					//
					//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//						//LOG(info) << "torque test1";
					//						desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					//						myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					//						LOG(info) << "torque test2";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//						//LOG(info) << "torque test3";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					//						//LOG(info) << "torque test4";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//						//LOG(info) << "torque test5";
					//
					//						myYouBotManipulator->getJointData(EachJointVelocity1);
					//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
					//						myYouBotManipulator->getJointData(EachJointTorque1);
					//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					//					}

					if (J3poserror < 0.01)
					{

						if (i3 < qqq - 1)
						{
							cout << i3 << endl;
							i3++;
						}

					}
					//					if (i3 == qqq-1)
					//					{
					//
					//						myYouBotManipulator->getJointData(EachJointAngle1);
					//						sang1 = EachJointAngle1[0].angle.value();
					//						sang2 = EachJointAngle1[1].angle.value();
					//						sang3 = EachJointAngle1[2].angle.value();
					//						sang4 = EachJointAngle1[3].angle.value();
					//						sang5 = EachJointAngle1[4].angle.value();
					//
					//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					//
					//
					//						c_2 = cos(sang2_cal);
					//						c_3 = cos(sang3_cal);
					//						c_4 = cos(sang4_cal);
					//						c_5 = cos(sang5_cal);
					//
					//						s_2 = sin(sang2_cal);
					//						s_3 = sin(sang3_cal);
					//						s_4 = sin(sang4_cal);
					//						s_5 = sin(sang5_cal);
					//						s_23 = sin(sang2_cal - sang3_cal);
					//						s_34 = sin(sang3_cal - sang4_cal);
					//						s2_2 = sin(2 * sang2_cal);
					//						s2_5 = sin(2 * sang5_cal);
					//						c_23 = cos(sang2_cal - sang3_cal);
					//						c_34 = cos(sang3_cal - sang4_cal);
					//						c_45 = cos(sang4_cal + sang5_cal);
					//						c_4_5 = cos(sang4_cal - sang5_cal);
					//						c_25 = cos(sang2_cal + sang5_cal);
					//						c_2_5 = cos(sang2_cal - sang5_cal);
					//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					//						s_223 = sin(2 * sang2_cal - sang3_cal);
					//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
					//
					//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					//						term_3 = (3969783.0*c_4);
					//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					//						term_5 = (753.0*c_5);
					//						term_6 = (27027.0*s_4);
					//						term_7 = (20331.0*c_5*s_4);
					//						term_8 = (4557899.0*s_3*s_4);
					//						term_9 = (4557899.0*c_3*c_4);
					//						term_10 = (31031.0*c_3*s_4);
					//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					//						term_12 = (31031.0*c_4*s_3);
					//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					//						term_18 = (23343.0*c_3*c_5*s_4);
					//						term_19 = (23343.0*c_4*c_5*s_3);
					//
					//
					//						N_matrix.at<double>(0, 0) = 0.0;
					//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
					//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
					//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
					//						tau_matrix = N_matrix;
					//#pragma endregion 重力補償test
					//
					//
					//
					//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//						//LOG(info) << "torque test1";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					//						//LOG(info) << "torque test2";
					//						desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
					//						myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//						LOG(info) << "torque test3";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					//						//LOG(info) << "torque test4";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//						//LOG(info) << "torque test5";
					//
					//						myYouBotManipulator->getJointData(EachJointVelocity1);
					//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
					//						myYouBotManipulator->getJointData(EachJointTorque1);
					//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					//
					//
					//
					//					}


					if (J4poserror < 0.01)
					{

						if (i4 < qqq - 1)
						{
							cout << i4 << endl;
							i4++;
						}

					}

					//					if (i4 == qqq-1)
					//					{
					//
					//						myYouBotManipulator->getJointData(EachJointAngle1);
					//						sang1 = EachJointAngle1[0].angle.value();
					//						sang2 = EachJointAngle1[1].angle.value();
					//						sang3 = EachJointAngle1[2].angle.value();
					//						sang4 = EachJointAngle1[3].angle.value();
					//						sang5 = EachJointAngle1[4].angle.value();
					//
					//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
					//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
					//
					//
					//						c_2 = cos(sang2_cal);
					//						c_3 = cos(sang3_cal);
					//						c_4 = cos(sang4_cal);
					//						c_5 = cos(sang5_cal);
					//
					//						s_2 = sin(sang2_cal);
					//						s_3 = sin(sang3_cal);
					//						s_4 = sin(sang4_cal);
					//						s_5 = sin(sang5_cal);
					//						s_23 = sin(sang2_cal - sang3_cal);
					//						s_34 = sin(sang3_cal - sang4_cal);
					//						s2_2 = sin(2 * sang2_cal);
					//						s2_5 = sin(2 * sang5_cal);
					//						c_23 = cos(sang2_cal - sang3_cal);
					//						c_34 = cos(sang3_cal - sang4_cal);
					//						c_45 = cos(sang4_cal + sang5_cal);
					//						c_4_5 = cos(sang4_cal - sang5_cal);
					//						c_25 = cos(sang2_cal + sang5_cal);
					//						c_2_5 = cos(sang2_cal - sang5_cal);
					//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
					//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
					//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
					//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
					//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
					//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
					//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
					//						s_223 = sin(2 * sang2_cal - sang3_cal);
					//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
					//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
					//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
					//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
					//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
					//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
					//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
					//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
					//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
					//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
					//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
					//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
					//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
					//
					//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
					//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
					//						term_3 = (3969783.0*c_4);
					//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
					//						term_5 = (753.0*c_5);
					//						term_6 = (27027.0*s_4);
					//						term_7 = (20331.0*c_5*s_4);
					//						term_8 = (4557899.0*s_3*s_4);
					//						term_9 = (4557899.0*c_3*c_4);
					//						term_10 = (31031.0*c_3*s_4);
					//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
					//						term_12 = (31031.0*c_4*s_3);
					//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
					//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
					//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
					//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
					//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
					//						term_18 = (23343.0*c_3*c_5*s_4);
					//						term_19 = (23343.0*c_4*c_5*s_3);
					//
					//
					//						N_matrix.at<double>(0, 0) = 0.0;
					//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
					//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
					//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
					//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
					//						tau_matrix = N_matrix;
					//#pragma endregion 重力補償test
					//
					//
					//
					//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
					//						//LOG(info) << "torque test1";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
					//						//LOG(info) << "torque test2";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
					//						//LOG(info) << "torque test3";
					//						desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
					//						myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
					//						LOG(info) << "torque test4";
					//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
					//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
					//						//LOG(info) << "torque test5";
					//
					//						myYouBotManipulator->getJointData(EachJointVelocity1);
					//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
					//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
					//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
					//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
					//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
					//						myYouBotManipulator->getJointData(EachJointTorque1);
					//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
					//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
					//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
					//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
					//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
					//
					//
					//
					//					}

					if (J5poserror < 0.01)
					{

						if (i5 < qqq - 1)
						{
							cout << i5 << endl;
							i5++;
						}

					}

					if (i5 == qqq - 1)
					{

						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						tau_matrix = N_matrix;
#pragma endregion 重力補償test



						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						//LOG(info) << "torque test1";
						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//LOG(info) << "torque test2";
						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//LOG(info) << "torque test3";
						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//LOG(info) << "torque test4";
						desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						LOG(info) << "torque test5";

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣



					}



					//if (abs(ss.at<double>(0, 0)) < 10 && abs(ss.at<double>(1, 0)) < 10 && abs(ss.at<double>(2, 0)) < 10 && abs(ss.at<double>(3, 0)) < 10 && abs(ss.at<double>(4, 0)) < 10)
					//
					//{

					//	if (i < qqq)
					//	{
					//		cout << i << endl;
					//		i++;
					//	}
					//	
					//}

					if (GetAsyncKeyState(0x57))	//w
					{

						return 0;
						break;

					}

				}

			}

#pragma endregion sliding-mode控制器
#pragma endregion ccc==5

#pragma region ccc==6
			if (ccc == 6)
			{
				SLEEP_MILLISEC(3000);
				myYouBotManipulator->getArmGripper().close();
				while (1)
				{
					if (count == 0)
					{
#pragma region 輸入初始角度
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

#pragma endregion 輸入初始角度

#pragma region 角度換算
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


#pragma endregion 角度換算
#pragma region 順向運動學
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


#pragma endregion 順向運動學



#pragma region youbot角度整理
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
						//ang2 = -ang2;
						//ang3 = -ang3;
						//ang4 = -ang4;
						ang5 = -ang5;
						//論文Benjamin_Keiser_Torque_Control_2013補充有寫到角度換算問題

						ang1 = ang1 + theta1_zero;
						ang2 = ang2 + theta2_zero;
						ang3 = ang3 + theta3_zero;
						ang4 = ang4 + theta4_zero;
						ang5 = ang5 + theta5_zero;

						cout << "T10" << T10 << endl;
						cout << "T20" << T20 << endl;
						cout << "T30" << T30 << endl;
						cout << "T40" << T40 << endl;
						cout << "T50" << T50 << endl;
						cout << "給youbot角度 " << ang1 << " " << ang2 << "" << ang3 << " " << ang4 << " " << ang5 << endl;
						///Test Move
						cout << "Test Forward Move " << endl;

#pragma endregion youbot角度整理
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

						//myYouBotManipulator->getArmGripper().open();
						LOG(info) << "unfold arm";
						SLEEP_MILLISEC(3000);
#pragma region 計算夾爪尤拉角zyz
						/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////
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
						/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////


#pragma region 計算夾爪尤拉角zyz

#pragma region 計算夾爪位置xyz
						tool_end = T50*tool;//求得夾爪的位置x y z


						end_x = tool_end.at<double>(0, 0);
						end_y = tool_end.at<double>(1, 0);
						end_z = tool_end.at<double>(2, 0);
						cout << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
						printf("\n");
#pragma endregion 計算夾爪位置xyz
#pragma region 計算期望由拉角z-y-z
						/////////////////////////////之前40給的夾爪與電池的轉換關係///////////////////////////////
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
						/////////////////////////////之前40給的夾爪與電池的轉換關係///////////////////////////////
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
						cout << "exp_x:" << endl;
						cin >> exp_x;
						cout << "exp_y:" << endl;
						cin >> exp_y;
						cout << "exp_z:" << endl;
						cin >> exp_z;
						cout << "exp_gamma:" << endl;
						cin >> exp_gamma;
						cout << "exp_beta:" << endl;
						cin >> exp_beta;
						cout << "exp_alpha:" << endl;
						cin >> exp_alpha;
#pragma endregion 計算期望由拉角z-y-z

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
					}
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
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();
						cutang << count << ":    " << sang1 << "  " << sang2 << "  " << sang3 << "  " << sang4 << "  " << sang5 << endl;

						ang1 = sang1;
						ang2 = sang2;
						ang3 = sang3;
						ang4 = sang4;
						ang5 = sang5;

						ang1 = ang1 - theta1_zero;
						ang2 = ang2 - theta2_zero;
						ang3 = ang3 - theta3_zero;
						ang4 = ang4 - theta4_zero;
						ang5 = ang5 - theta5_zero;

						///Because Assume opposite to real Arm.
						ang1 = -ang1;
						//ang2 = -ang2;
						//ang3 = -ang3;
						//ang4 = -ang4; 
						ang5 = -ang5;
						//////////////為了讓坐標系比較好看




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
						/////////////懶得驗證///////////////////
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


						tool_end = T50*tool;
						end_x = tool_end.at<double>(0, 0);//夾爪當前的x y z
						end_y = tool_end.at<double>(1, 0);
						end_z = tool_end.at<double>(2, 0);
						cout << "期望位置: " << exp_x << " " << exp_y << " " << exp_z << " " << exp_gamma << " " << exp_beta << " " << exp_alpha << "  " << " 正確的位置" << endl;
						cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << "  " << " 正確的位置" << endl;
						printf("\n");




						Mat Jacob(3, 3, CV_64FC1);//jacobian關係 角速度與各軸變化輛
						Jacob.at<double>(0, 0) = cos(alpha*deg2rad)*sin(beta*deg2rad);	Jacob.at<double>(0, 1) = -sin(alpha* deg2rad);	Jacob.at<double>(0, 2) = 0.0;
						Jacob.at<double>(1, 0) = sin(alpha*deg2rad)*cos(beta*deg2rad);	Jacob.at<double>(1, 1) = cos(alpha* deg2rad);	Jacob.at<double>(1, 2) = 0.0;
						Jacob.at<double>(2, 0) = cos(beta*deg2rad);							Jacob.at<double>(2, 1) = 0.0;						Jacob.at<double>(2, 2) = 1.0;


						//============================

						Mat Q_Jacob(3, 3, CV_64FC1);
						Q_Jacob.at<double>(0, 0) = 0.0;		Q_Jacob.at<double>(0, 1) = -end_z;	Q_Jacob.at<double>(0, 2) = end_y;
						Q_Jacob.at<double>(1, 0) = end_z;	Q_Jacob.at<double>(1, 1) = 0.0;		Q_Jacob.at<double>(1, 2) = -end_x;
						Q_Jacob.at<double>(2, 0) = -end_y;	Q_Jacob.at<double>(2, 1) = end_x;	Q_Jacob.at<double>(2, 2) = 0.0;
						Q_Jacob = -Q_Jacob;//CROSS反過來

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


						nowx << end_x << endl;
						nowy << end_y << endl;
						nowz << end_z << endl;
						nowr << gamma << endl;
						nowb << beta << endl;
						nowa << alpha << endl;

#pragma endregion 角度wx,wy,wz	
						IDentity.at<double>(0, 0) = 1; IDentity.at<double>(0, 1) = 0; IDentity.at<double>(0, 2) = 0;
						IDentity.at<double>(1, 0) = 0; IDentity.at<double>(1, 1) = 1; IDentity.at<double>(1, 2) = 0;
						IDentity.at<double>(2, 0) = 0; IDentity.at<double>(2, 1) = 0; IDentity.at<double>(2, 2) = 1;
#pragma region u=-lambda*De

						Mat vel(3, 1, CV_64FC1);////相對基底速度命令
						Mat velw(3, 1, CV_64FC1);////相對基底速度命令
						Mat QQ(3, 1, CV_64FC1);
						//QQ = Q_Jacob*(w_error);


						//vel = -0.3 * ((r - rd) - Q_Jacob*(wr - wrd));
						vel = -lamda_v * ((r - rd) + Q_Jacob*Jacob*(wr - wrd));
						//vel = -0.3 * ((r - rd) - Q_Jacob*(wr - wrd));
						velw = -lamda_w * (0 + Jacob * (wr - wrd));

#pragma endregion u=-lambda*De

						Mat Error_dot = Mat(6, 1, CV_64FC1);//加角度是6
						Mat T(6, 6, CV_64FC1);

						double vx, vy, vz, wx, wy, wz;
						////原本
						Error_dot.at<double>(0, 0) = vel.at<double>(0, 0);
						Error_dot.at<double>(1, 0) = vel.at<double>(1, 0);
						Error_dot.at<double>(2, 0) = vel.at<double>(2, 0);
						Error_dot.at<double>(3, 0) = velw.at<double>(0, 0);
						Error_dot.at<double>(4, 0) = velw.at<double>(1, 0);
						Error_dot.at<double>(5, 0) = velw.at<double>(2, 0);



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
						Error_dot.at<double>(4, 0) = Error_dot.at<double>(4, 0);
						Error_dot.at<double>(5, 0) = Error_dot.at<double>(5, 0);

						//限制各軸速度極限
						vx = Error_dot.at<double>(0, 0);
						vy = Error_dot.at<double>(1, 0);
						vz = Error_dot.at<double>(2, 0);
						wx = Error_dot.at<double>(3, 0);
						wy = Error_dot.at<double>(4, 0);
						wz = Error_dot.at<double>(5, 0);
						if (vx > 3)
							Error_dot.at<double>(0, 0) = 3;
						if (vx < -3)
							Error_dot.at<double>(0, 0) = -3;
						if (vy > 3)
							Error_dot.at<double>(1, 0) = 3;
						if (vy < -3)
							Error_dot.at<double>(1, 0) = -3;
						if (vz > 3)
							Error_dot.at<double>(2, 0) = 3;
						if (vz < -3)
							Error_dot.at<double>(2, 0) = -3;
						if (wx > 3)
							Error_dot.at<double>(3, 0) = 3;
						if (wx < -3)
							Error_dot.at<double>(3, 0) = -3;
						if (wy > 3)
							Error_dot.at<double>(4, 0) = 3;
						if (wy < -3)
							Error_dot.at<double>(4, 0) = -3;
						if (wz > 3)
							Error_dot.at<double>(5, 0) = 3;
						if (wz < -3)
							Error_dot.at<double>(5, 0) = -3;

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


						Mat Theta_dot;
						Mat JacobianInv;


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

						if (GetAsyncKeyState(0x57))	//w
						{

							return 0;
							break;

						}
						count++;
					}

					while (abs(error_x) > 0.4 || abs(error_y) > 0.4 || abs(error_z) > 0.4 || abs(error_ThetaZ) > 0.12 || abs(error_ThetaY) > 0.12 || abs(error_ThetaX) > 0.12);
#pragma endregion 速度控制

					if (abs(error_x) < 0.4 && abs(error_y) < 0.4 && abs(error_z) < 0.4 && abs(error_ThetaZ) < 0.12 && abs(error_ThetaY) < 0.12 && abs(error_ThetaX) < 0.12)
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

						cout << "給期望點按1 拍照按2 夾爪放開按3 開車按7 新姿態9 夾爪關閉10 " << endl;
						cin >> bomb;


						while (bomb == 1 || bomb == 10 || bomb == 5 || bomb == 2 || bomb == 3 || bomb == 9 || bomb == 7)//while迴圈為了vector設計的
						{
							if (bomb == 10)
							{
								SLEEP_MILLISEC(4000);
								myYouBotManipulator->getArmGripper().close();
								SLEEP_MILLISEC(4000);//setpointBar1.setpointBar2.barEncoder = 500;
								cout << "給期望點按1 由拍照給新期望點按2 夾爪放開按3 開車按7 新姿態9 夾爪10 " << endl;
								cin >> bomb;
							}

#pragma region 新期望點
							if (bomb == 1)
							{

								cout << "目前位置:" << end_x << "  " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;



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

								cout << "exp_x:" << endl;
								cin >> exp_x;
								cout << "exp_y:" << endl;
								cin >> exp_y;
								cout << "exp_z:" << endl;
								cin >> exp_z;
								cout << "exp_gamma:" << endl;
								cin >> exp_gamma;
								cout << "exp_beta:" << endl;
								cin >> exp_beta;
								cout << "exp_alpha:" << endl;
								cin >> exp_alpha;


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


#pragma endregion 角度換算
#pragma region 順向運動學
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


#pragma endregion 順向運動學



#pragma region youbot角度整理
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
								//ang2 = -ang2;
								//ang3 = -ang3;
								//ang4 = -ang4;
								ang5 = -ang5;
								//論文Benjamin_Keiser_Torque_Control_2013補充有寫到角度換算問題

								ang1 = ang1 + theta1_zero;
								ang2 = ang2 + theta2_zero;
								ang3 = ang3 + theta3_zero;
								ang4 = ang4 + theta4_zero;
								ang5 = ang5 + theta5_zero;

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

					//////////////////中斷////////////////
					if (GetAsyncKeyState(0x57))	//w
					{
						return 0;
						break;
					}
					////////////////////////////////////////
				}
			}
#pragma endregion ccc==6

			///////////////////////////////////////////////////////////////////////////////////////////////////////// 
			if (ccc == 7)
			{
				if (count == 0)
				{
#pragma region 輸入初始角度
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

#pragma endregion 輸入初始角度

#pragma region 角度換算
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


#pragma endregion 角度換算
#pragma region 順向運動學
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


#pragma endregion 順向運動學



#pragma region youbot角度整理
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
					//ang2 = -ang2;
					//ang3 = -ang3;
					//ang4 = -ang4;
					ang5 = -ang5;
					//論文Benjamin_Keiser_Torque_Control_2013補充有寫到角度換算問題

					ang1 = ang1 + theta1_zero;
					ang2 = ang2 + theta2_zero;
					ang3 = ang3 + theta3_zero;
					ang4 = ang4 + theta4_zero;
					ang5 = ang5 + theta5_zero;

					cout << "T10" << T10 << endl;
					cout << "T20" << T20 << endl;
					cout << "T30" << T30 << endl;
					cout << "T40" << T40 << endl;
					cout << "T50" << T50 << endl;
					cout << "給youbot角度 " << ang1 << " " << ang2 << "" << ang3 << " " << ang4 << " " << ang5 << endl;
					///Test Move
					cout << "Test Forward Move " << endl;

#pragma endregion youbot角度整理
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

					//myYouBotManipulator->getArmGripper().open();
					LOG(info) << "unfold arm";
					SLEEP_MILLISEC(3000);
#pragma region 計算夾爪尤拉角zyz
					/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////
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
					/////////////////////////////算z-y-z  aplha-beta-gamma///////////////////////////////


#pragma region 計算夾爪尤拉角zyz

#pragma region 計算夾爪位置xyz
					tool_end = T50*tool;//求得夾爪的位置x y z


					end_x = tool_end.at<double>(0, 0);
					end_y = tool_end.at<double>(1, 0);
					end_z = tool_end.at<double>(2, 0);
					cout << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
					printf("\n");
#pragma endregion 計算夾爪位置xyz
#pragma region 計算期望由拉角z-y-z
					/////////////////////////////之前40給的夾爪與電池的轉換關係///////////////////////////////
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
					/////////////////////////////之前40給的夾爪與電池的轉換關係///////////////////////////////
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
					cout << "exp_x:" << endl;
					cin >> exp_x;
					cout << "exp_y:" << endl;
					cin >> exp_y;
					cout << "exp_z:" << endl;
					cin >> exp_z;
					cout << "exp_gamma:" << endl;
					cin >> exp_gamma;
					cout << "exp_beta:" << endl;
					cin >> exp_beta;
					cout << "exp_alpha:" << endl;
					cin >> exp_alpha;
#pragma endregion 計算期望由拉角z-y-z

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
				}
				do
				{

					theta1 = theta2 = theta3 = theta4 = theta5 = theta234 = 0.0;

					//cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << endl;
					printf("\n");

					sang1 = sang2 = sang3 = sang4 = sang5 = 0.0;
					//回傳算當前角度

					myYouBotManipulator->getJointData(EachJointAngle1);
					sang1 = EachJointAngle1[0].angle.value();
					sang2 = EachJointAngle1[1].angle.value();
					sang3 = EachJointAngle1[2].angle.value();
					sang4 = EachJointAngle1[3].angle.value();
					sang5 = EachJointAngle1[4].angle.value();
					cutang << count << ":    " << sang1 << "  " << sang2 << "  " << sang3 << "  " << sang4 << "  " << sang5 << endl;

					ang1 = sang1;
					ang2 = sang2;
					ang3 = sang3;
					ang4 = sang4;
					ang5 = sang5;

					ang1 = ang1 - theta1_zero;
					ang2 = ang2 - theta2_zero;
					ang3 = ang3 - theta3_zero;
					ang4 = ang4 - theta4_zero;
					ang5 = ang5 - theta5_zero;

					///Because Assume opposite to real Arm.
					ang1 = -ang1;
					//ang2 = -ang2;
					//ang3 = -ang3;
					//ang4 = -ang4; 
					ang5 = -ang5;
					//////////////為了讓坐標系比較好看




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
					/////////////懶得驗證///////////////////
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


					tool_end = T50*tool;
					end_x = tool_end.at<double>(0, 0);//夾爪當前的x y z
					end_y = tool_end.at<double>(1, 0);
					end_z = tool_end.at<double>(2, 0);
					cout << "期望位置: " << exp_x << " " << exp_y << " " << exp_z << " " << exp_gamma << " " << exp_beta << " " << exp_alpha << "  " << " 正確的位置" << endl;
					cout << "當前位置: " << end_x << " " << end_y << " " << end_z << " " << gamma << " " << beta << " " << alpha << "  " << " 正確的位置" << endl;
					printf("\n");




					Mat Jacob(3, 3, CV_64FC1);//jacobian關係 角速度與各軸變化輛
					Jacob.at<double>(0, 0) = cos(alpha*deg2rad)*sin(beta*deg2rad);	Jacob.at<double>(0, 1) = -sin(alpha* deg2rad);	Jacob.at<double>(0, 2) = 0.0;
					Jacob.at<double>(1, 0) = sin(alpha*deg2rad)*cos(beta*deg2rad);	Jacob.at<double>(1, 1) = cos(alpha* deg2rad);	Jacob.at<double>(1, 2) = 0.0;
					Jacob.at<double>(2, 0) = cos(beta*deg2rad);							Jacob.at<double>(2, 1) = 0.0;						Jacob.at<double>(2, 2) = 1.0;


					//============================

					Mat Q_Jacob(3, 3, CV_64FC1);
					Q_Jacob.at<double>(0, 0) = 0.0;		Q_Jacob.at<double>(0, 1) = -end_z;	Q_Jacob.at<double>(0, 2) = end_y;
					Q_Jacob.at<double>(1, 0) = end_z;	Q_Jacob.at<double>(1, 1) = 0.0;		Q_Jacob.at<double>(1, 2) = -end_x;
					Q_Jacob.at<double>(2, 0) = -end_y;	Q_Jacob.at<double>(2, 1) = end_x;	Q_Jacob.at<double>(2, 2) = 0.0;
					Q_Jacob = -Q_Jacob;//CROSS反過來

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


					nowx << end_x << endl;
					nowy << end_y << endl;
					nowz << end_z << endl;
					nowr << gamma << endl;
					nowb << beta << endl;
					nowa << alpha << endl;

#pragma endregion 角度wx,wy,wz	
					IDentity.at<double>(0, 0) = 1; IDentity.at<double>(0, 1) = 0; IDentity.at<double>(0, 2) = 0;
					IDentity.at<double>(1, 0) = 0; IDentity.at<double>(1, 1) = 1; IDentity.at<double>(1, 2) = 0;
					IDentity.at<double>(2, 0) = 0; IDentity.at<double>(2, 1) = 0; IDentity.at<double>(2, 2) = 1;
#pragma region u=-lambda*De

					Mat vel(3, 1, CV_64FC1);////相對基底速度命令
					Mat velw(3, 1, CV_64FC1);////相對基底速度命令
					Mat QQ(3, 1, CV_64FC1);
					//QQ = Q_Jacob*(w_error);


					//vel = -0.3 * ((r - rd) - Q_Jacob*(wr - wrd));
					vel = -lamda_v * ((r - rd) + Q_Jacob*Jacob*(wr - wrd));
					//vel = -0.3 * ((r - rd) - Q_Jacob*(wr - wrd));
					velw = -lamda_w * (0 + Jacob * (wr - wrd));

#pragma endregion u=-lambda*De

					Mat Error_dot = Mat(6, 1, CV_64FC1);//加角度是6
					Mat T(6, 6, CV_64FC1);

					double vx, vy, vz, wx, wy, wz;
					////原本
					Error_dot.at<double>(0, 0) = vel.at<double>(0, 0);
					Error_dot.at<double>(1, 0) = vel.at<double>(1, 0);
					Error_dot.at<double>(2, 0) = vel.at<double>(2, 0);
					Error_dot.at<double>(3, 0) = velw.at<double>(0, 0);
					Error_dot.at<double>(4, 0) = velw.at<double>(1, 0);
					Error_dot.at<double>(5, 0) = velw.at<double>(2, 0);



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
					Error_dot.at<double>(4, 0) = Error_dot.at<double>(4, 0);
					Error_dot.at<double>(5, 0) = Error_dot.at<double>(5, 0);

					//限制各軸速度極限
					vx = Error_dot.at<double>(0, 0);
					vy = Error_dot.at<double>(1, 0);
					vz = Error_dot.at<double>(2, 0);
					wx = Error_dot.at<double>(3, 0);
					wy = Error_dot.at<double>(4, 0);
					wz = Error_dot.at<double>(5, 0);
					if (vx > 5)
						Error_dot.at<double>(0, 0) = 5;
					if (vx < -5)
						Error_dot.at<double>(0, 0) = -5;
					if (vy > 5)
						Error_dot.at<double>(1, 0) = 5;
					if (vy < -5)
						Error_dot.at<double>(1, 0) = -5;
					if (vz > 5)
						Error_dot.at<double>(2, 0) = 5;
					if (vz < -5)
						Error_dot.at<double>(2, 0) = -5;
					if (wx > 5)
						Error_dot.at<double>(3, 0) = 5;
					if (wx < -5)
						Error_dot.at<double>(3, 0) = -5;
					if (wy > 5)
						Error_dot.at<double>(4, 0) = 5;
					if (wy < -5)
						Error_dot.at<double>(4, 0) = -5;
					if (wz > 5)
						Error_dot.at<double>(5, 0) = 5;
					if (wz < -5)
						Error_dot.at<double>(5, 0) = -5;

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


					Mat Theta_dot;
					Mat JacobianInv;


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
					count++;

					int i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;
					cout << "理想軌跡總共時間" << endl;
					cin >> ttime;

					cout << "第一軸理想位置" << endl;
					cin >> desired_tempang.at<double>(0, 0);
					cout << "第二軸理想位置" << endl;
					cin >> desired_tempang.at<double>(0, 0);
					cout << "第三軸理想位置" << endl;
					cin >> desired_tempang.at<double>(0, 0);
					cout << "第四軸理想位置" << endl;
					cin >> desired_tempang.at<double>(0, 0);
					cout << "第五軸理想位置" << endl;
					cin >> desired_tempang.at<double>(0, 0);

					desired_tempang.at<double>(0, 0) = desired_tempang.at<double>(0, 0) - 2.9496;
					desired_tempang.at<double>(1, 0) = desired_tempang.at<double>(1, 0) - 1.1345;
					desired_tempang.at<double>(2, 0) = (desired_tempang.at<double>(2, 0)*-1) - 2.5482;
					desired_tempang.at<double>(3, 0) = desired_tempang.at<double>(3, 0) - 1.789;
					desired_tempang.at<double>(4, 0) = desired_tempang.at<double>(4, 0) - 2.9234;
					while (1)
					{

						myYouBotManipulator->getJointData(EachJointAngle1);
						sang1 = EachJointAngle1[0].angle.value();
						sang2 = EachJointAngle1[1].angle.value();
						sang3 = EachJointAngle1[2].angle.value();
						sang4 = EachJointAngle1[3].angle.value();
						sang5 = EachJointAngle1[4].angle.value();

						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型

						myYouBotManipulator->getJointData(EachJointVelocity1);
						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						svel3 = svel3*-1;//注意第三軸速度回傳一定要有負號
						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣


						ang_matrix.at<double>(0, 0) = sang1_cal;
						ang_matrix.at<double>(1, 0) = sang2_cal;
						ang_matrix.at<double>(2, 0) = sang3_cal;
						ang_matrix.at<double>(3, 0) = sang4_cal;
						ang_matrix.at<double>(4, 0) = sang5_cal;
						vel_matrix.at<double>(0, 0) = svel1;
						vel_matrix.at<double>(1, 0) = svel2;
						vel_matrix.at<double>(2, 0) = svel3;
						vel_matrix.at<double>(3, 0) = svel4;
						vel_matrix.at<double>(4, 0) = svel5;

						c_2 = cos(sang2_cal);
						c_3 = cos(sang3_cal);
						c_4 = cos(sang4_cal);
						c_5 = cos(sang5_cal);

						s_2 = sin(sang2_cal);
						s_3 = sin(sang3_cal);
						s_4 = sin(sang4_cal);
						s_5 = sin(sang5_cal);
						s_23 = sin(sang2_cal - sang3_cal);
						s_34 = sin(sang3_cal - sang4_cal);
						s2_2 = sin(2 * sang2_cal);
						s2_5 = sin(2 * sang5_cal);
						c_23 = cos(sang2_cal - sang3_cal);
						c_34 = cos(sang3_cal - sang4_cal);
						c_45 = cos(sang4_cal + sang5_cal);
						c_4_5 = cos(sang4_cal - sang5_cal);
						c_25 = cos(sang2_cal + sang5_cal);
						c_2_5 = cos(sang2_cal - sang5_cal);
						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						s_223 = sin(2 * sang2_cal - sang3_cal);
						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						term_3 = (3969783.0*c_4);
						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						term_5 = (753.0*c_5);
						term_6 = (27027.0*s_4);
						term_7 = (20331.0*c_5*s_4);
						term_8 = (4557899.0*s_3*s_4);
						term_9 = (4557899.0*c_3*c_4);
						term_10 = (31031.0*c_3*s_4);
						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						term_12 = (31031.0*c_4*s_3);
						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						term_18 = (23343.0*c_3*c_5*s_4);
						term_19 = (23343.0*c_4*c_5*s_3);
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
#pragma region 科氏力矩陣

						C_matrix.at<double>(0, 0) = (30399.0*svel2*c2_234) / 125000000.0 - (30399.0*svel3*c2_234) / 125000000.0 + (30399.0*svel4*c2_234) / 125000000.0 + (1864633.0*svel2*s2_2) / 50000000.0 + (2259.0*svel5*s2_5) / 4000000000.0 + (31441279.0*svel2*s2_234) / 4000000000.0 - (31441279.0*svel3*s2_234) / 4000000000.0 + (31441279.0*svel4*s2_234) / 4000000000.0 - (23343.0*svel2*c_22345) / 400000000.0 - (20331.0*svel2*c_222345) / 400000000.0 + (23343.0*svel3*c_22345) / 800000000.0 + (20331.0*svel3*c_222345) / 400000000.0 - (23343.0*svel4*c_22345) / 800000000.0 - (20331.0*svel4*c_222345) / 800000000.0 - (23343.0*svel5*c_22345) / 800000000.0 - (20331.0*svel5*c_222345) / 800000000.0 - (24849.0*svel2*s_234_5) / 2000000000.0 + (24849.0*svel3*s_234_5) / 2000000000.0 - (24849.0*svel4*s_234_5) / 2000000000.0 + (24849.0*svel5*s_234_5) / 2000000000.0 + (2536699.0*svel2*s_223) / 50000000.0 + (10013911.0*svel2*s2_23) / 500000000.0 - (2536699.0*svel3*s_223) / 100000000.0 - (10013911.0*svel3*s2_23) / 500000000.0 - (4851957.0*svel2*c_234) / 1000000000.0 + (4851957.0*svel3*c_234) / 1000000000.0 - (4851957.0*svel4*c_234) / 1000000000.0 - (23343.0*svel3*c_345) / 800000000.0 + (23343.0*svel4*c_345) / 800000000.0 - (23343.0*svel5*c_345) / 800000000.0 + (33033.0*svel2*s_234) / 500000000.0 - (33033.0*svel3*s_234) / 500000000.0 + (33033.0*svel4*s_234) / 500000000.0 + (20331.0*svel4*c_45) / 800000000.0 + (20331.0*svel5*c_45) / 800000000.0 - (23343.0*svel2*c_2234_5) / 400000000.0 - (20331.0*svel2*c_22234_5) / 400000000.0 + (23343.0*svel3*c_2234_5) / 800000000.0 - (2259.0*svel2*c_2223245) / 62500000.0 + (20331.0*svel3*c_22234_5) / 400000000.0 - (23343.0*svel4*c_2234_5) / 800000000.0 + (2259.0*svel3*c_2223245) / 62500000.0 - (20331.0*svel4*c_22234_5) / 800000000.0 + (23343.0*svel5*c_2234_5) / 800000000.0 - (2259.0*svel4*c_2223245) / 62500000.0 + (20331.0*svel5*c_22234_5) / 800000000.0 - (2259.0*svel5*c_2223245) / 125000000.0 + (753.0*svel2*s_2223245) / 1000000000.0 - (753.0*svel3*s_2223245) / 1000000000.0 + (753.0*svel4*s_2223245) / 1000000000.0 + (753.0*svel5*s_2223245) / 2000000000.0 - (347127.0*svel2*c_2) / 20000000.0 - (27027.0*svel4*c_4) / 200000000.0 + (31031.0*svel2*c_2234) / 100000000.0 + (27027.0*svel2*c_22234) / 100000000.0 - (31031.0*svel3*c_2234) / 200000000.0 - (27027.0*svel3*c_22234) / 100000000.0 + (31031.0*svel4*c_2234) / 200000000.0 - (23343.0*svel3*c_34_5) / 800000000.0 + (27027.0*svel4*c_22234) / 200000000.0 + (23343.0*svel4*c_34_5) / 800000000.0 + (23343.0*svel5*c_34_5) / 800000000.0 - (2536699.0*svel3*s_3) / 100000000.0 - (3969783.0*svel4*s_4) / 400000000.0 + (753.0*svel5*s_5) / 1000000000.0 + (4557899.0*svel2*s_2234) / 200000000.0 + (3969783.0*svel2*s_22234) / 200000000.0 - (4557899.0*svel3*s_2234) / 400000000.0 - (3969783.0*svel3*s_22234) / 200000000.0 + (4557899.0*svel4*s_2234) / 400000000.0 + (3969783.0*svel4*s_22234) / 400000000.0 - (24849.0*svel2*s_2345) / 2000000000.0 + (24849.0*svel3*s_2345) / 2000000000.0 - (24849.0*svel4*s_2345) / 2000000000.0 - (24849.0*svel5*s_2345) / 2000000000.0 - (2700357.0*svel2*c_23) / 250000000.0 + (2700357.0*svel3*c_23) / 250000000.0 + (31031.0*svel3*c_34) / 200000000.0 - (31031.0*svel4*c_34) / 200000000.0 + (20331.0*svel4*c_4_5) / 800000000.0 - (20331.0*svel5*c_4_5) / 800000000.0 - (2259.0*svel2*c_222324_5) / 62500000.0 + (2259.0*svel3*c_222324_5) / 62500000.0 - (2259.0*svel4*c_222324_5) / 62500000.0 + (2259.0*svel5*c_222324_5) / 125000000.0 - (4557899.0*svel3*s_34) / 400000000.0 + (4557899.0*svel4*s_34) / 400000000.0 + (753.0*svel2*s_222324_5) / 1000000000.0 - (2259.0*svel2*s2_234_5) / 8000000000.0 - (2259.0*svel2*s2_2345) / 8000000000.0 - (753.0*svel3*s_222324_5) / 1000000000.0 + (2259.0*svel3*s2_234_5) / 8000000000.0 + (2259.0*svel3*s2_2345) / 8000000000.0 + (753.0*svel4*s_222324_5) / 1000000000.0 - (2259.0*svel4*s2_234_5) / 8000000000.0 - (2259.0*svel4*s2_2345) / 8000000000.0 - (753.0*svel5*s_222324_5) / 2000000000.0 + (2259.0*svel5*s2_234_5) / 8000000000.0 - (2259.0*svel5*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 1) = (30399.0*svel1*c2_234) / 125000000.0 + (1864633.0*svel1*s2_2) / 50000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 400000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 400000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 + (2536699.0*svel1*s_223) / 50000000.0 + (10013911.0*svel1*s2_23) / 500000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (20331.0*svel2*c_235) / 400000000.0 - (20331.0*svel3*c_235) / 400000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel2*c_25) / 400000000.0 - (23343.0*svel1*c_2234_5) / 400000000.0 - (20331.0*svel1*c_22234_5) / 400000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (347127.0*svel1*c_2) / 20000000.0 + (31031.0*svel1*c_2234) / 100000000.0 + (27027.0*svel1*c_22234) / 100000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 + (18867.0*svel2*s_2) / 100000000.0 + (4557899.0*svel1*s_2234) / 200000000.0 + (3969783.0*svel1*s_22234) / 200000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (2700357.0*svel1*c_23) / 250000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 - (441029.0*svel2*s_23) / 500000000.0 + (441029.0*svel3*s_23) / 500000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 2) = (23343.0*svel1*c_22345) / 800000000.0 - (31441279.0*svel1*s2_234) / 4000000000.0 - (30399.0*svel1*c2_234) / 125000000.0 + (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 400000000.0 - (2259.0*svel3*c_234_5) / 62500000.0 + (2259.0*svel4*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 - (753.0*svel2*s_234_5) / 1000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 + (753.0*svel3*s_234_5) / 1000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 - (753.0*svel4*s_234_5) / 1000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (2259.0*svel5*s_234_25) / 4000000000.0 - (2259.0*svel5*s_23425) / 4000000000.0 - (2536699.0*svel1*s_223) / 100000000.0 - (10013911.0*svel1*s2_23) / 500000000.0 + (4851957.0*svel1*c_234) / 1000000000.0 - (20331.0*svel2*c_235) / 400000000.0 - (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (33033.0*svel1*s_234) / 500000000.0 - (25433.0*svel2*s_234) / 100000000.0 + (25433.0*svel3*s_234) / 100000000.0 - (25433.0*svel4*s_234) / 100000000.0 + (20002259.0*svel5*s_234) / 2000000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 400000000.0 + (2259.0*svel1*c_2223245) / 62500000.0 - (753.0*svel1*s_2223245) / 1000000000.0 - (31031.0*svel1*c_2234) / 200000000.0 - (27027.0*svel1*c_22234) / 100000000.0 + (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 - (20331.0*svel3*c_23_5) / 400000000.0 - (2536699.0*svel1*s_3) / 100000000.0 - (4557899.0*svel1*s_2234) / 400000000.0 - (3969783.0*svel1*s_22234) / 200000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 + (2700357.0*svel1*c_23) / 250000000.0 + (31031.0*svel1*c_34) / 200000000.0 + (2259.0*svel1*c_222324_5) / 62500000.0 + (441029.0*svel2*s_23) / 500000000.0 - (4557899.0*svel1*s_34) / 400000000.0 - (441029.0*svel3*s_23) / 500000000.0 - (753.0*svel1*s_222324_5) / 1000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 3) = (30399.0*svel1*c2_234) / 125000000.0 + (31441279.0*svel1*s2_234) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 + (2259.0*svel5*s_234_25) / 4000000000.0 + (2259.0*svel5*s_23425) / 4000000000.0 - (4851957.0*svel1*c_234) / 1000000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (33033.0*svel1*s_234) / 500000000.0 + (25433.0*svel2*s_234) / 100000000.0 - (25433.0*svel3*s_234) / 100000000.0 + (25433.0*svel4*s_234) / 100000000.0 - (20002259.0*svel5*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 62500000.0 + (753.0*svel1*s_2223245) / 1000000000.0 - (27027.0*svel1*c_4) / 200000000.0 + (31031.0*svel1*c_2234) / 200000000.0 + (27027.0*svel1*c_22234) / 200000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 - (3969783.0*svel1*s_4) / 400000000.0 + (4557899.0*svel1*s_2234) / 400000000.0 + (3969783.0*svel1*s_22234) / 400000000.0 + (2259.0*svel2*c_2345) / 62500000.0 - (2259.0*svel3*c_2345) / 62500000.0 + (2259.0*svel4*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 - (753.0*svel2*s_2345) / 1000000000.0 + (753.0*svel3*s_2345) / 1000000000.0 - (753.0*svel4*s_2345) / 1000000000.0 - (31031.0*svel1*c_34) / 200000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 62500000.0 + (4557899.0*svel1*s_34) / 400000000.0 + (753.0*svel1*s_222324_5) / 1000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(0, 4) = (2259.0*svel1*s2_5) / 4000000000.0 - (23343.0*svel1*c_22345) / 800000000.0 - (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel5*c_234_5) / 62500000.0 + (24849.0*svel1*s_234_5) / 2000000000.0 + (2259.0*svel2*s_234_25) / 4000000000.0 + (2259.0*svel2*s_23425) / 4000000000.0 - (2259.0*svel3*s_234_25) / 4000000000.0 - (2259.0*svel3*s_23425) / 4000000000.0 + (2259.0*svel4*s_234_25) / 4000000000.0 + (2259.0*svel4*s_23425) / 4000000000.0 - (753.0*svel5*s_234_5) / 1000000000.0 - (23343.0*svel1*c_345) / 800000000.0 - (20331.0*svel5*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 + (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel5*c_25) / 400000000.0 + (23343.0*svel1*c_2234_5) / 800000000.0 + (20331.0*svel1*c_22234_5) / 800000000.0 - (2259.0*svel1*c_2223245) / 125000000.0 + (753.0*svel1*s_2223245) / 2000000000.0 + (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel5*c_23_5) / 400000000.0 + (753.0*svel1*s_5) / 1000000000.0 - (24849.0*svel5*s_5) / 1000000000.0 - (2259.0*svel5*c_2345) / 62500000.0 - (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel5*s_2345) / 1000000000.0 - (20331.0*svel1*c_4_5) / 800000000.0 + (23343.0*svel5*c_2_5) / 400000000.0 + (2259.0*svel1*c_222324_5) / 125000000.0 - (753.0*svel1*s_222324_5) / 2000000000.0 + (2259.0*svel1*s2_234_5) / 8000000000.0 - (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(1, 0) = (svel1*((24849.0*s_2345) / 1000000000.0 + (2700357.0*c_23) / 125000000.0 + (2259.0*c_222324_5) / 31250000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (1864633.0*s2_2) / 25000000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 200000000.0 + (20331.0*c_222345) / 200000000.0 + (24849.0*s_234_5) / 1000000000.0 - (2536699.0*s_223) / 25000000.0 - (10013911.0*s2_23) / 250000000.0 + (4851957.0*c_234) / 500000000.0 - (33033.0*s_234) / 250000000.0 + (23343.0*c_2234_5) / 200000000.0 + (20331.0*c_22234_5) / 200000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (347127.0*c_2) / 10000000.0 - (31031.0*c_2234) / 50000000.0 - (27027.0*c_22234) / 50000000.0 - (4557899.0*s_2234) / 100000000.0 - (3969783.0*s_22234) / 100000000)) / 2 + (svel5*((23343.0*c_2_5) / 200000000.0 - (753.0*s_2345) / 500000000.0 + (2259.0*c_234_5) / 31250000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (23343.0*c_25) / 200000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(1, 1) = (svel4*((20331.0*c_4_5) / 200000000.0 - (31031.0*c_34) / 50000000.0 + (4557899.0*s_34) / 100000000.0 + (23343.0*c_345) / 200000000.0 + (20331.0*c_45) / 200000000.0 - (27027.0*c_4) / 50000000.0 + (23343.0*c_34_5) / 200000000.0 - (3969783.0*s_4) / 100000000)) / 2 - (svel3*((4557899.0*s_34) / 100000000.0 - (31031.0*c_34) / 50000000.0 + (23343.0*c_345) / 200000000.0 + (23343.0*c_34_5) / 200000000.0 + (2536699.0*s_3) / 25000000)) / 2 - (svel5*((2259.0*s2_5) / 1000000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000.0 + (23343.0*c_3*s_4*s_5) / 100000000.0 - (23343.0*c_4*s_3*s_5) / 100000000)) / 2;
						C_matrix.at<double>(1, 2) = (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 + (27027.0*svel4*c_4) / 100000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 + (23343.0*svel3*c_34_5) / 400000000.0 - (23343.0*svel4*c_34_5) / 400000000.0 - (23343.0*svel5*c_34_5) / 400000000.0 - (2536699.0*svel2*s_3) / 50000000.0 + (2536699.0*svel3*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_34) / 100000000.0 - (31031.0*svel3*c_34) / 100000000.0 + (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0 - (4557899.0*svel2*s_34) / 200000000.0 + (4557899.0*svel3*s_34) / 200000000.0 - (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 3) = (23343.0*svel2*c_345) / 400000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (23343.0*svel3*c_345) / 400000000.0 + (23343.0*svel4*c_345) / 400000000.0 - (23343.0*svel5*c_345) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 - (27027.0*svel2*c_4) / 100000000.0 + (27027.0*svel3*c_4) / 100000000.0 - (27027.0*svel4*c_4) / 100000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 - (3969783.0*svel2*s_4) / 200000000.0 + (3969783.0*svel3*s_4) / 200000000.0 - (3969783.0*svel4*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_34) / 100000000.0 + (31031.0*svel3*c_34) / 100000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (31031.0*svel4*c_34) / 100000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0 + (4557899.0*svel2*s_34) / 200000000.0 - (4557899.0*svel3*s_34) / 200000000.0 + (4557899.0*svel4*s_34) / 200000000.0;
						C_matrix.at<double>(1, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 - (23343.0*svel2*c_345) / 400000000.0 + (23343.0*svel3*c_345) / 400000000.0 - (23343.0*svel4*c_345) / 400000000.0 + (23343.0*svel5*c_345) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (23343.0*svel1*c_25) / 400000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel4*c_45) / 400000000.0 + (20331.0*svel5*c_45) / 400000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (23343.0*svel2*c_34_5) / 400000000.0 - (23343.0*svel3*c_34_5) / 400000000.0 + (23343.0*svel4*c_34_5) / 400000000.0 + (23343.0*svel5*c_34_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 + (23343.0*svel1*c_2_5) / 400000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0 - (20331.0*svel4*c_4_5) / 400000000.0 + (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(2, 0) = (svel1*((4557899.0*s_34) / 200000000.0 - (2700357.0*c_23) / 125000000.0 - (31031.0*c_34) / 100000000.0 - (2259.0*c_222324_5) / 31250000.0 - (24849.0*s_2345) / 1000000000.0 + (753.0*s_222324_5) / 500000000.0 - (2259.0*s2_234_5) / 4000000000.0 - (2259.0*s2_2345) / 4000000000.0 + (30399.0*c2_234) / 62500000.0 + (31441279.0*s2_234) / 2000000000.0 - (23343.0*c_22345) / 400000000.0 - (20331.0*c_222345) / 200000000.0 - (24849.0*s_234_5) / 1000000000.0 + (2536699.0*s_223) / 50000000.0 + (10013911.0*s2_23) / 250000000.0 - (4851957.0*c_234) / 500000000.0 + (23343.0*c_345) / 400000000.0 + (33033.0*s_234) / 250000000.0 - (23343.0*c_2234_5) / 400000000.0 - (20331.0*c_22234_5) / 200000000.0 - (2259.0*c_2223245) / 31250000.0 + (753.0*s_2223245) / 500000000.0 + (31031.0*c_2234) / 100000000.0 + (27027.0*c_22234) / 50000000.0 + (23343.0*c_34_5) / 400000000.0 + (2536699.0*s_3) / 50000000.0 + (4557899.0*s_2234) / 200000000.0 + (3969783.0*s_22234) / 100000000)) / 2 - (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20331.0*c_235) / 200000000.0 + (20002259.0*s_234) / 1000000000.0 + (20331.0*c_23_5) / 200000000.0 + (2259.0*c_2345) / 31250000)) / 2;
						C_matrix.at<double>(2, 1) = (2259.0*svel5*s2_5) / 2000000000.0 + (27027.0*svel4*c_4) / 100000000.0 + (2536699.0*svel2*s_3) / 50000000.0 + (3969783.0*svel4*s_4) / 200000000.0 - (753.0*svel5*s_5) / 500000000.0 - (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel4*c_4*c_5) / 200000000.0 - (4557899.0*svel2*c_3*s_4) / 200000000.0 + (4557899.0*svel2*c_4*s_3) / 200000000.0 - (31031.0*svel2*s_3*s_4) / 100000000.0 + (20331.0*svel5*s_4*s_5) / 200000000.0 + (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 + (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(2, 2) = -(svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 3) = (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel4*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 + (svel5*((2259.0*c_5*s_5) / 500000000.0 - (753.0*s_5) / 250000000.0 + (20331.0*s_4*s_5) / 100000000)) / 2;
						C_matrix.at<double>(2, 4) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel4*c_45) / 400000000.0 - (20331.0*svel5*c_45) / 400000000.0 - (2259.0*svel5*c_5) / 31250000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0 + (20331.0*svel4*c_4_5) / 400000000.0 - (20331.0*svel5*c_4_5) / 400000000.0;
						C_matrix.at<double>(3, 0) = (svel5*((2259.0*c_234_5) / 31250000.0 - (753.0*s_2345) / 500000000.0 - (753.0*s_234_5) / 500000000.0 + (2259.0*s_234_25) / 2000000000.0 + (2259.0*s_23425) / 2000000000.0 + (20002259.0*s_234) / 1000000000.0 + (2259.0*c_2345) / 31250000)) / 2 + (svel1*((24849.0*s_2345) / 1000000000.0 + (31031.0*c_34) / 100000000.0 - (20331.0*c_4_5) / 400000000.0 + (2259.0*c_222324_5) / 31250000.0 - (4557899.0*s_34) / 200000000.0 - (753.0*s_222324_5) / 500000000.0 + (2259.0*s2_234_5) / 4000000000.0 + (2259.0*s2_2345) / 4000000000.0 - (30399.0*c2_234) / 62500000.0 - (31441279.0*s2_234) / 2000000000.0 + (23343.0*c_22345) / 400000000.0 + (20331.0*c_222345) / 400000000.0 + (24849.0*s_234_5) / 1000000000.0 + (4851957.0*c_234) / 500000000.0 - (23343.0*c_345) / 400000000.0 - (33033.0*s_234) / 250000000.0 - (20331.0*c_45) / 400000000.0 + (23343.0*c_2234_5) / 400000000.0 + (20331.0*c_22234_5) / 400000000.0 + (2259.0*c_2223245) / 31250000.0 - (753.0*s_2223245) / 500000000.0 + (27027.0*c_4) / 100000000.0 - (31031.0*c_2234) / 100000000.0 - (27027.0*c_22234) / 100000000.0 - (23343.0*c_34_5) / 400000000.0 + (3969783.0*s_4) / 200000000.0 - (4557899.0*s_2234) / 200000000.0 - (3969783.0*s_22234) / 200000000)) / 2;
						C_matrix.at<double>(3, 1) = (27027.0*svel2*c_4) / 100000000.0 - (2259.0*svel5*s2_5) / 2000000000.0 - (27027.0*svel3*c_4) / 100000000.0 + (3969783.0*svel2*s_4) / 200000000.0 - (3969783.0*svel3*s_4) / 200000000.0 + (753.0*svel5*s_5) / 500000000.0 + (31031.0*svel2*c_3*c_4) / 100000000.0 - (20331.0*svel2*c_4*c_5) / 200000000.0 + (20331.0*svel3*c_4*c_5) / 200000000.0 + (4557899.0*svel2*c_3*s_4) / 200000000.0 - (4557899.0*svel2*c_4*s_3) / 200000000.0 + (31031.0*svel2*s_3*s_4) / 100000000.0 - (23343.0*svel2*c_3*c_4*c_5) / 200000000.0 - (23343.0*svel2*c_5*s_3*s_4) / 200000000.0;
						C_matrix.at<double>(3, 2) = (svel3*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel2*((27027.0*c_4) / 50000000.0 + (3969783.0*s_4) / 100000000.0 - (20331.0*c_4*c_5) / 100000000)) / 2 - (svel5*((753.0*s_5) / 250000000.0 - (2259.0*c_5*s_5) / 500000000)) / 2;
						C_matrix.at<double>(3, 3) = -(753.0*svel5*(3.0*s2_5 - 4.0*s_5)) / 2000000000.0;
						C_matrix.at<double>(3, 4) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (2259.0*svel5*c_5) / 31250000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 0) = (23343.0*svel1*c_22345) / 800000000.0 - (2259.0*svel1*s2_5) / 4000000000.0 - (2259.0*svel2*c_234_5) / 62500000.0 + (20331.0*svel1*c_222345) / 800000000.0 + (2259.0*svel3*c_234_5) / 62500000.0 - (2259.0*svel4*c_234_5) / 62500000.0 - (24849.0*svel1*s_234_5) / 2000000000.0 + (753.0*svel2*s_234_5) / 1000000000.0 - (2259.0*svel2*s_234_25) / 4000000000.0 - (2259.0*svel2*s_23425) / 4000000000.0 - (753.0*svel3*s_234_5) / 1000000000.0 + (2259.0*svel3*s_234_25) / 4000000000.0 + (2259.0*svel3*s_23425) / 4000000000.0 + (753.0*svel4*s_234_5) / 1000000000.0 - (2259.0*svel4*s_234_25) / 4000000000.0 - (2259.0*svel4*s_23425) / 4000000000.0 - (20331.0*svel2*c_235) / 400000000.0 + (23343.0*svel1*c_345) / 800000000.0 + (20331.0*svel3*c_235) / 400000000.0 - (20002259.0*svel2*s_234) / 2000000000.0 + (20002259.0*svel3*s_234) / 2000000000.0 - (20002259.0*svel4*s_234) / 2000000000.0 - (23343.0*svel2*c_25) / 400000000.0 - (20331.0*svel1*c_45) / 800000000.0 - (23343.0*svel1*c_2234_5) / 800000000.0 - (20331.0*svel1*c_22234_5) / 800000000.0 + (2259.0*svel1*c_2223245) / 125000000.0 - (753.0*svel1*s_2223245) / 2000000000.0 - (20331.0*svel2*c_23_5) / 400000000.0 - (23343.0*svel1*c_34_5) / 800000000.0 + (20331.0*svel3*c_23_5) / 400000000.0 - (753.0*svel1*s_5) / 1000000000.0 - (2259.0*svel2*c_2345) / 62500000.0 + (2259.0*svel3*c_2345) / 62500000.0 - (2259.0*svel4*c_2345) / 62500000.0 + (24849.0*svel1*s_2345) / 2000000000.0 + (753.0*svel2*s_2345) / 1000000000.0 - (753.0*svel3*s_2345) / 1000000000.0 + (753.0*svel4*s_2345) / 1000000000.0 - (23343.0*svel2*c_2_5) / 400000000.0 + (20331.0*svel1*c_4_5) / 800000000.0 - (2259.0*svel1*c_222324_5) / 125000000.0 + (753.0*svel1*s_222324_5) / 2000000000.0 - (2259.0*svel1*s2_234_5) / 8000000000.0 + (2259.0*svel1*s2_2345) / 8000000000.0;
						C_matrix.at<double>(4, 1) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20331.0*svel1*c_235) / 400000000.0 + (23343.0*svel2*c_345) / 400000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (23343.0*svel1*c_25) / 400000000.0 - (20331.0*svel2*c_45) / 400000000.0 + (20331.0*svel3*c_45) / 400000000.0 - (20331.0*svel1*c_23_5) / 400000000.0 - (23343.0*svel2*c_34_5) / 400000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0 - (23343.0*svel1*c_2_5) / 400000000.0 + (20331.0*svel2*c_4_5) / 400000000.0 - (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 2) = (2259.0*svel3*s2_5) / 2000000000.0 - (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel4*s2_5) / 2000000000.0 + (2259.0*svel1*c_234_5) / 62500000.0 - (753.0*svel1*s_234_5) / 1000000000.0 + (2259.0*svel1*s_234_25) / 4000000000.0 + (2259.0*svel1*s_23425) / 4000000000.0 + (20331.0*svel1*c_235) / 400000000.0 + (20002259.0*svel1*s_234) / 2000000000.0 + (20331.0*svel2*c_45) / 400000000.0 - (20331.0*svel3*c_45) / 400000000.0 + (20331.0*svel1*c_23_5) / 400000000.0 + (753.0*svel2*s_5) / 500000000.0 - (753.0*svel3*s_5) / 500000000.0 + (753.0*svel4*s_5) / 500000000.0 + (2259.0*svel1*c_2345) / 62500000.0 - (753.0*svel1*s_2345) / 1000000000.0 - (20331.0*svel2*c_4_5) / 400000000.0 + (20331.0*svel3*c_4_5) / 400000000.0;
						C_matrix.at<double>(4, 3) = (2259.0*svel2*s2_5) / 2000000000.0 - (2259.0*svel3*s2_5) / 2000000000.0 + (2259.0*svel4*s2_5) / 2000000000.0 - (2259.0*svel1*c_234_5) / 62500000.0 + (753.0*svel1*s_234_5) / 1000000000.0 - (2259.0*svel1*s_234_25) / 4000000000.0 - (2259.0*svel1*s_23425) / 4000000000.0 - (20002259.0*svel1*s_234) / 2000000000.0 - (753.0*svel2*s_5) / 500000000.0 + (753.0*svel3*s_5) / 500000000.0 - (753.0*svel4*s_5) / 500000000.0 - (2259.0*svel1*c_2345) / 62500000.0 + (753.0*svel1*s_2345) / 1000000000.0;
						C_matrix.at<double>(4, 4) = 0.0;
#pragma endregion 科氏力矩陣
						/////////////////////////////////////////////科氏力矩陣///////////////////////////////////////////////////
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////
#pragma region 重力矩陣


						N_matrix.at<double>(0, 0) = 0.0;
						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;

#pragma endregion 重力矩陣
						/////////////////////////////////////////////重力矩陣///////////////////////////////////////////////////

						/////////////////////////////////////////////慣性矩陣/////////////////////////////
#pragma region 慣性矩陣

						M_matrix.at<double>(0, 0) = (4557899.0*cos(sang3_cal - sang4_cal)) / 200000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 1000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal)) / 8000000000.0 + (2259.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal)) / 8000000000.0 - (2700357.0*sin(sang2_cal - sang3_cal)) / 125000000.0 + (31031.0*sin(sang3_cal - sang4_cal)) / 100000000.0 + (20331.0*sin(sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal)) / 62500000.0 - (1864633.0*cos(2 * sang2_cal)) / 50000000.0 - (2259.0*cos(2 * sang5_cal)) / 4000000000.0 - (31441279.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 4000000000.0 + (30399.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal)) / 125000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal)) / 400000000.0 - (2536699.0*cos(2 * sang2_cal - sang3_cal)) / 50000000.0 - (10013911.0*cos(2 * sang2_cal - 2 * sang3_cal)) / 500000000.0 - (33033.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 250000000.0 - (4851957.0*sin(sang2_cal - sang3_cal + sang4_cal)) / 500000000.0 - (23343.0*sin(sang3_cal - sang4_cal + sang5_cal)) / 400000000.0 - (753.0*cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 1000000000.0 + (20331.0*sin(sang4_cal + sang5_cal)) / 400000000.0 - (23343.0*sin(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (20331.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal)) / 400000000.0 - (2259.0*sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal)) / 62500000.0 + (2536699.0*c_3) / 50000000.0 + term_3 / 200000000.0 - term_5 / 500000000.0 - (4557899.0*cos(2 * sang2_cal - sang3_cal + sang4_cal)) / 200000000.0 - (3969783.0*cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 200000000.0 - (347127.0*s_2) / 10000000.0 - term_6 / 100000000.0 + (31031.0*sin(2 * sang2_cal - sang3_cal + sang4_cal)) / 100000000.0 + (27027.0*sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal)) / 100000000.0 - (23343.0*sin(sang3_cal - sang4_cal - sang5_cal)) / 400000000.0 + (24849.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 1000000000.0 + 105207843 / 800000000.0;
						M_matrix.at<double>(0, 1) = term_1 + (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_16 / 400000000.0 + (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 - (18867.0*c_2) / 100000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 2) = term_2 - (441029.0*cos(sang2_cal - sang3_cal)) / 500000000.0 - term_1 - term_13 / 4000000000.0 + term_14 / 4000000000.0 + term_15 / 62500000.0 + term_17 / 100000000.0 - term_16 / 400000000.0 + (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(0, 3) = term_1 - term_2 + term_13 / 4000000000.0 - term_14 / 4000000000.0 - term_15 / 62500000.0 - term_17 / 100000000.0 + term_11 / 1000000000.0;
						M_matrix.at<double>(0, 4) = (20002259.0*cos(sang2_cal - sang3_cal + sang4_cal)) / 1000000000.0 - (23343.0*sin(sang2_cal - sang5_cal)) / 400000000.0 - term_2 - term_15 / 62500000.0 - term_1 - term_16 / 400000000.0 - (23343.0*sin(sang2_cal + sang5_cal)) / 400000000.0 + (24849.0*c_5) / 1000000000.0 - (20331.0*sin(sang2_cal - sang3_cal - sang5_cal)) / 400000000.0 - term_11 / 1000000000.0;
						M_matrix.at<double>(1, 0) = M_matrix.at<double>(0, 1);
						M_matrix.at<double>(1, 1) = (2536699.0*c_3) / 25000000.0 + term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_9 / 100000000.0 - term_10 / 50000000.0 + term_12 / 50000000.0 + term_7 / 100000000.0 + term_8 / 100000000.0 + term_4 + term_18 / 100000000.0 - term_19 / 100000000.0 + 180370741 / 1000000000.0;
						M_matrix.at<double>(1, 2) = term_5 / 250000000.0 - term_3 / 100000000.0 - (2536699.0*c_3) / 50000000.0 + term_6 / 50000000.0 - term_9 / 200000000.0 + term_10 / 100000000.0 - term_12 / 100000000.0 - term_7 / 100000000.0 - term_8 / 200000000.0 - term_4 - term_18 / 200000000.0 + term_19 / 200000000.0 - 95785421 / 1000000000.0;
						M_matrix.at<double>(1, 3) = term_3 / 200000000.0 - term_5 / 250000000.0 - term_6 / 100000000.0 + term_9 / 200000000.0 - term_10 / 100000000.0 + term_12 / 100000000.0 + term_7 / 200000000.0 + term_8 / 200000000.0 + term_4 + term_18 / 200000000.0 - term_19 / 200000000.0 + 45729777 / 1000000000.0;
						M_matrix.at<double>(1, 4) = (753.0*s_5*(155.0*cos(sang3_cal - sang4_cal) + 135.0*c_4 + 96)) / 1000000000.0;
						M_matrix.at<double>(2, 0) = M_matrix.at<double>(0, 2);
						M_matrix.at<double>(2, 1) = M_matrix.at<double>(1, 2);
						M_matrix.at<double>(2, 2) = term_3 / 100000000.0 - term_5 / 250000000.0 - term_6 / 50000000.0 + term_7 / 100000000.0 + term_4 + 95785421 / 1000000000.0;
						M_matrix.at<double>(2, 3) = term_5 / 250000000.0 - term_3 / 200000000.0 + term_6 / 100000000.0 - term_7 / 200000000.0 - term_4 - 45729777 / 1000000000.0;
						M_matrix.at<double>(2, 4) = -(2259.0*s_5*(45.0*c_4 + 32)) / 1000000000.0;
						M_matrix.at<double>(3, 0) = M_matrix.at<double>(0, 3);
						M_matrix.at<double>(3, 1) = M_matrix.at<double>(1, 3);
						M_matrix.at<double>(3, 2) = M_matrix.at<double>(2, 3);
						M_matrix.at<double>(3, 3) = term_4 - term_5 / 250000000.0 + 45729777.0 / 1000000000.0;
						M_matrix.at<double>(3, 4) = (2259.0*s_5) / 31250000.0;
						M_matrix.at<double>(4, 0) = M_matrix.at<double>(0, 4);
						M_matrix.at<double>(4, 1) = M_matrix.at<double>(1, 4);
						M_matrix.at<double>(4, 2) = M_matrix.at<double>(2, 4);
						M_matrix.at<double>(4, 3) = M_matrix.at<double>(3, 4);
						M_matrix.at<double>(4, 4) = 20002259.0 / 1000000000.0;
#pragma endregion 慣性矩陣


						///////所有角度都先以動力學模型的角度轉換為基準


#pragma region lambda-kk-qq
						lambda = 0;
						kk = 0;
						///////////////////單軸移動
						//lambda.at<double>(0, 0) = 256;
						//lambda.at<double>(1, 1) = 33.3;//56.43//56.47//56.53//34//33.3
						//lambda.at<double>(2, 2) = 101;//128//125//135//134.5
						//lambda.at<double>(3, 3) = 63.6;//62.5
						//lambda.at<double>(4, 4) = 95;
						///////////////////////2-3-4軸移動

						lambda.at<double>(0, 0) = 256;
						lambda.at<double>(1, 1) = 53;//70最低了//75//70//40
						lambda.at<double>(2, 2) = 28;//158.702會抖//45//20//35
						lambda.at<double>(3, 3) = 20.3;//62.5//58//75//62.5//15
						lambda.at<double>(4, 4) = 95;

						//kk.at<double>(0, 0) = 0.1;//0.1
						//kk.at<double>(1, 1) = 70;//0.8//12
						//kk.at<double>(2, 2) = 25;//1//7
						//kk.at<double>(3, 3) = 1;//0.2
						//kk.at<double>(4, 4) = 0.1;//0.1



						kk.at<double>(0, 0) = 0.1;
						kk.at<double>(1, 1) = 5.5;//5.8
						kk.at<double>(2, 2) = 3;//5.8
						kk.at<double>(3, 3) = 1.67;
						kk.at<double>(4, 4) = 0.1;
						qq = 10;

						desired_tempvel.at<double>(0, 0) = Theta_dot.at<double>(0, 0);
						desired_tempvel.at<double>(1, 0) = Theta_dot.at<double>(1, 0);
						desired_tempvel.at<double>(2, 0) = -Theta_dot.at<double>(2, 0);
						desired_tempvel.at<double>(3, 0) = Theta_dot.at<double>(3, 0);
						desired_tempvel.at<double>(4, 0) = Theta_dot.at<double>(4, 0);
#pragma endregion lambda-kk-qq
						joint_poserror = ang_matrix - desired_tempang;
						joint_poserror_dot = vel_matrix - desired_tempvel;

						J1poserror = joint_poserror.at<double>(0, 0);
						J2poserror = joint_poserror.at<double>(1, 0);
						J3poserror = joint_poserror.at<double>(2, 0);
						J4poserror = joint_poserror.at<double>(3, 0);
						J5poserror = joint_poserror.at<double>(4, 0);



						//						if (sang3_cal > -1)//第三軸保護措施 做實驗前須檢查是否符合自己所需
						//						{
						//							myYouBotManipulator->getJointData(EachJointAngle1);
						//							sang1 = EachJointAngle1[0].angle.value();
						//							sang2 = EachJointAngle1[1].angle.value();
						//							sang3 = EachJointAngle1[2].angle.value();
						//							sang4 = EachJointAngle1[3].angle.value();
						//							sang5 = EachJointAngle1[4].angle.value();
						//
						//							sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						//							sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						//							sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						//							sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						//							sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
						//
						//
						//							c_2 = cos(sang2_cal);
						//							c_3 = cos(sang3_cal);
						//							c_4 = cos(sang4_cal);
						//							c_5 = cos(sang5_cal);
						//
						//							s_2 = sin(sang2_cal);
						//							s_3 = sin(sang3_cal);
						//							s_4 = sin(sang4_cal);
						//							s_5 = sin(sang5_cal);
						//							s_23 = sin(sang2_cal - sang3_cal);
						//							s_34 = sin(sang3_cal - sang4_cal);
						//							s2_2 = sin(2 * sang2_cal);
						//							s2_5 = sin(2 * sang5_cal);
						//							c_23 = cos(sang2_cal - sang3_cal);
						//							c_34 = cos(sang3_cal - sang4_cal);
						//							c_45 = cos(sang4_cal + sang5_cal);
						//							c_4_5 = cos(sang4_cal - sang5_cal);
						//							c_25 = cos(sang2_cal + sang5_cal);
						//							c_2_5 = cos(sang2_cal - sang5_cal);
						//							s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						//							c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						//							c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						//							c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						//							c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						//							c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						//							s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						//							s_223 = sin(2 * sang2_cal - sang3_cal);
						//							s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//							s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						//							c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						//							c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//							c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//							c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//							c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//							s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//							s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//							c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//							s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						//							s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						//							s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//							c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//							c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						//							c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						//							s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//							s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//							c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//							c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//							s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						//							s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
						//
						//							term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						//							term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						//							term_3 = (3969783.0*c_4);
						//							term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						//							term_5 = (753.0*c_5);
						//							term_6 = (27027.0*s_4);
						//							term_7 = (20331.0*c_5*s_4);
						//							term_8 = (4557899.0*s_3*s_4);
						//							term_9 = (4557899.0*c_3*c_4);
						//							term_10 = (31031.0*c_3*s_4);
						//							term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						//							term_12 = (31031.0*c_4*s_3);
						//							term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						//							term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						//							term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						//							term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						//							term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						//							term_18 = (23343.0*c_3*c_5*s_4);
						//							term_19 = (23343.0*c_4*c_5*s_3);
						//
						//
						//							N_matrix.at<double>(0, 0) = 0.0;
						//							N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//							N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//							N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						//							N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//							N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						//							N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						//							tau_matrix = N_matrix;
						//#pragma endregion 重力補償test
						//
						//
						//
						//							desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						//							myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						//							LOG(info) << "torque test1";
						//							desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//							myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//							LOG(info) << "torque test2";
						//							desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//							myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//							LOG(info) << "torque test3";
						//							desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//							myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//							LOG(info) << "torque test4";
						//							desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						//							myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						//							LOG(info) << "torque test5";
						//
						//							myYouBotManipulator->getJointData(EachJointVelocity1);
						//							svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						//							svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						//							svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						//							svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						//							svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//							myYouBotManipulator->getJointData(EachJointTorque1);
						//							stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						//							stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						//							stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						//							stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						//							stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
						//
						//						}

						J1velerror = joint_poserror_dot.at<double>(0, 0);
						J2velerror = joint_poserror_dot.at<double>(1, 0);
						J3velerror = joint_poserror_dot.at<double>(2, 0);
						J4velerror = joint_poserror_dot.at<double>(3, 0);
						J5velerror = joint_poserror_dot.at<double>(4, 0);
						ss = joint_poserror_dot + lambda * joint_poserror;

						for (int i = 0; i < 5; i++)
						{
							qaz = ss.at<double>(i, 0);
							if (qaz > qq)
							{
								sw.at<double>(i, 0) = 1;
							}
							if (qaz < qq)
							{
								sw.at<double>(i, 0) = -1;
							}
							if (abs(qaz) <= qq)
							{
								sw.at<double>(i, 0) = qaz / qq;
							}

						}
						N_matrix.at<double>(1, 0) = N_matrix.at<double>(1, 0)*0.6;
						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*0.75*0.68;
						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0)*0.7;
						adj = 0;
						adj.at<double>(0, 0) = 1;//1	
						adj.at<double>(1, 1) = 1;//0.9	
						adj.at<double>(2, 2) = 1;//1.05	
						adj.at<double>(3, 3) = 1;//1.05	
						adj.at<double>(4, 4) = 1;//1
						//adj2 = 0;
						//adj2.at<double>(0, 0) = 1;//1
						//adj2.at<double>(1, 1) = 0.7;//0.7
						//adj2.at<double>(2, 2) = 1.3;//1.1
						//adj2.at<double>(3, 3) = 0.7;//0.7
						//adj2.at<double>(4, 4) = 1;//1


						//Ueq = M_matrix*desired_tempacc + adj2*C_matrix*vel_matrix + N_matrix + M_matrix* (-lambda*joint_poserror_dot);
						//Ueq = adj*(M_matrix*desired_tempacc + C_matrix*vel_matrix + N_matrix) + M_matrix* (-lambda*joint_poserror_dot);
						Ueq = adj*(M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix) + N_matrix;
						//Ueq = (M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix) + N_matrix;//基本
						Ueq.at<double>(2, 0) = Ueq.at<double>(2, 0)*-1;

						Usw = -kk *sw;
						Usw.at<double>(2, 0) = Usw.at<double>(2, 0)*-1;
						Utemp = Usw;
						tau_matrix = Ueq + Usw;
						check1 = adj*(M_matrix*(desired_tempacc - lambda*joint_poserror_dot) + C_matrix*vel_matrix);//檢查用
						ch1 = check1.at<double>(0, 0);
						ch2 = check1.at<double>(1, 0);
						ch3 = check1.at<double>(2, 0);
						ch4 = check1.at<double>(3, 0);
						ch5 = check1.at<double>(4, 0);
						check2 = M_matrix*desired_tempacc;
						ch11 = check2.at<double>(0, 0);
						ch22 = check2.at<double>(1, 0);
						ch33 = check2.at<double>(2, 0);
						ch44 = check2.at<double>(3, 0);
						ch55 = check2.at<double>(4, 0);


						M11 = M_matrix.at<double>(0, 0); M12 = M_matrix.at<double>(0, 1); M13 = M_matrix.at<double>(0, 2); M14 = M_matrix.at<double>(0, 3); M15 = M_matrix.at<double>(0, 4);
						M21 = M_matrix.at<double>(1, 0); M22 = M_matrix.at<double>(1, 1); M23 = M_matrix.at<double>(1, 2); M24 = M_matrix.at<double>(1, 3); M25 = M_matrix.at<double>(1, 4);
						M31 = M_matrix.at<double>(2, 0); M32 = M_matrix.at<double>(2, 1); M33 = M_matrix.at<double>(2, 2); M34 = M_matrix.at<double>(2, 3); M35 = M_matrix.at<double>(2, 4);
						M41 = M_matrix.at<double>(3, 0); M42 = M_matrix.at<double>(3, 1); M43 = M_matrix.at<double>(3, 2); M44 = M_matrix.at<double>(3, 3); M45 = M_matrix.at<double>(3, 4);
						M51 = M_matrix.at<double>(4, 0); M52 = M_matrix.at<double>(4, 1); M53 = M_matrix.at<double>(4, 2); M54 = M_matrix.at<double>(4, 3); M55 = M_matrix.at<double>(4, 4);

						FF1 = Ueq.at<double>(0, 0);
						FF2 = Ueq.at<double>(1, 0);
						FF3 = Ueq.at<double>(2, 0);
						FF4 = Ueq.at<double>(3, 0);
						FF5 = Ueq.at<double>(4, 0);

						FB1 = Usw.at<double>(0, 0);
						FB2 = Usw.at<double>(1, 0);
						FB3 = Usw.at<double>(2, 0);
						FB4 = Usw.at<double>(3, 0);
						FB5 = Usw.at<double>(4, 0);



						tau_matrix.at<double>(0, 0) = tau_matrix.at<double>(0, 0);
						tau_matrix.at<double>(1, 0) = tau_matrix.at<double>(1, 0);
						tau_matrix.at<double>(2, 0) = tau_matrix.at<double>(2, 0);
						tau_matrix.at<double>(3, 0) = tau_matrix.at<double>(3, 0);
						tau_matrix.at<double>(4, 0) = tau_matrix.at<double>(4, 0);



						N1 = tau_matrix.at<double>(0, 0);
						N2 = tau_matrix.at<double>(1, 0);
						N3 = tau_matrix.at<double>(2, 0);
						N4 = tau_matrix.at<double>(3, 0);
						N5 = tau_matrix.at<double>(4, 0);
						cout << "N1" << N1 << endl;
						cout << "N2" << N2 << endl;
						cout << "N3" << N3 << endl;
						cout << "N4" << N4 << endl;
						cout << "N5" << N5 << endl;
						cout << "切割幾個" << i << endl;
#pragma region 限制力矩
						if (tau_matrix.at<double>(0, 0) > 3)
						{
							tau_matrix.at<double>(0, 0) = 3;
						}
						if (tau_matrix.at<double>(0, 0) < -3)
						{
							tau_matrix.at<double>(0, 0) = -3;
						}
						if (tau_matrix.at<double>(1, 0) > 5)
						{
							tau_matrix.at<double>(1, 0) = 5;
						}
						if (tau_matrix.at<double>(1, 0) < -5)
						{
							tau_matrix.at<double>(1, 0) = -5;
						}
						if (tau_matrix.at<double>(2, 0) > 6)
						{
							tau_matrix.at<double>(2, 0) = 6;
						}
						if (tau_matrix.at<double>(2, 0) < -6)
						{
							tau_matrix.at<double>(2, 0) = -6;
						}
						if (tau_matrix.at<double>(3, 0) > 4.0)
						{
							tau_matrix.at<double>(3, 0) = 4.0;
						}
						if (tau_matrix.at<double>(3, 0) < -4.0)
						{
							tau_matrix.at<double>(3, 0) = -4.0;
						}
						if (tau_matrix.at<double>(4, 0) > 1.0)
						{
							tau_matrix.at<double>(4, 0) = 1.0;
						}
						if (tau_matrix.at<double>(4, 0) < -1.0)
						{
							tau_matrix.at<double>(4, 0) = -1.0;
						}


						cout << "第一軸   " << ss.at<double>(0, 0) << "    第二軸  " << ss.at<double>(1, 0) << "    第三軸  " << ss.at<double>(2, 0) << "   第四軸  " << ss.at<double>(3, 0) << "   第五軸  " << ss.at<double>(4, 0) << endl;
						N1 = tau_matrix.at<double>(0, 0);
						N2 = tau_matrix.at<double>(1, 0);
						N3 = tau_matrix.at<double>(2, 0);
						N4 = tau_matrix.at<double>(3, 0);
						N5 = tau_matrix.at<double>(4, 0);
#pragma endregion 限制力矩

#pragma region 力矩輸出
						desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						LOG(info) << "torque test1";
						desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						LOG(info) << "torque test2";
						desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						LOG(info) << "torque test3";
						desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						LOG(info) << "torque test4";
						desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						LOG(info) << "torque test5";
#pragma endregion 力矩輸出

						myYouBotManipulator->getJointData(EachJointTorque1);
						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
						JointTau << "第一軸   " << stau1 << "   第二軸  " << stau2 << "   第三軸  " << stau3 << "   第四軸  " << stau4 << "   第五軸  " << stau5 << endl;
						Tau1 << stau1 << endl;		Tau2 << stau2 << endl;   Tau3 << stau3 << endl;   Tau4 << stau4 << endl;  Tau5 << stau5 << endl;
						nowx << "第一軸   " << sang1_cal << "   第二軸  " << sang2_cal << "   第三軸  " << sang3_cal << "   第四軸  " << sang4_cal << "   第五軸  " << sang5_cal << endl;
						jointdesiredvel << "第一軸   " << desired_tempvel.at<double>(0.0) << "   第二軸  " << desired_tempvel.at<double>(1.0) << "   第三軸  " << desired_tempvel.at<double>(2.0) << "   第四軸  " << desired_tempvel.at<double>(3.0) << "   第五軸  " << desired_tempvel.at<double>(4.0) << endl;
						jointdesiredang << "第一軸   " << desired_tempang.at<double>(0.0) << "   第二軸  " << desired_tempang.at<double>(1.0) << "   第三軸  " << desired_tempang.at<double>(2.0) << "   第四軸  " << desired_tempang.at<double>(3.0) << "   第五軸  " << desired_tempang.at<double>(4.0) << endl;
						Usww << "第一軸   " << Utemp.at<double>(0.0) << "   第二軸  " << Utemp.at<double>(1.0) << "   第三軸  " << Utemp.at<double>(2.0) << "   第四軸  " << Utemp.at<double>(3.0) << "   第五軸  " << Utemp.at<double>(4.0) << endl;
						Jointx << "第一軸   " << sang1 << "   第二軸  " << sang2 << "   第三軸  " << sang3 << "   第四軸  " << sang4 << "   第五軸  " << sang5 << endl;
						joint1 << sang1 << endl;	joint2 << sang2 << endl;	joint3 << sang3 << endl;	joint4 << sang4 << endl;	joint5 << sang5 << endl;
						CalJointTau << "第一軸   " << tau_matrix.at<double>(0, 0) << "    第二軸  " << tau_matrix.at<double>(1, 0) << "    第三軸  " << tau_matrix.at<double>(2, 0) << "   第四軸  " << tau_matrix.at<double>(3, 0) << "   第五軸  " << tau_matrix.at<double>(4, 0) << endl;
						jointerror << "第一軸   " << J1poserror << "    第二軸  " << J2poserror << "    第三軸  " << J3poserror << "   第四軸  " << J4poserror << "   第五軸  " << J5poserror << endl;
						jointvelerror << "第一軸   " << J1velerror << "    第二軸  " << J2velerror << "    第三軸  " << J3velerror << "   第四軸  " << J4velerror << "   第五軸  " << J5velerror << endl;
						JointVelocity << "第一軸   " << svel1 << "   第二軸  " << svel2 << "   第三軸  " << svel3 << "   第四軸  " << svel4 << "   第五軸" << svel5 << endl;
						sliding << "第一軸   " << ss.at<double>(0, 0) << "    第二軸  " << ss.at<double>(1, 0) << "    第三軸  " << ss.at<double>(2, 0) << "   第四軸  " << ss.at<double>(3, 0) << "   第五軸  " << ss.at<double>(4, 0) << endl;
						Ueqq << "第一軸   " << check1.at<double>(0.0) << "   第二軸  " << check1.at<double>(1.0) << "   第三軸  " << check1.at<double>(2.0) << "   第四軸  " << check1.at<double>(3.0) << "   第五軸  " << check1.at<double>(4.0) << endl;
						QAZ << i4 << endl;
						count++;


						if (J1poserror < 0.01)
						{
							if (i1 < qqq - 1)
							{
								cout << i1 << endl;
								i1++;
							}
						}
						if (i1 == qqq)
						{
							myYouBotManipulator->getJointData(EachJointAngle1);
							sang1 = EachJointAngle1[0].angle.value();
							sang2 = EachJointAngle1[1].angle.value();
							sang3 = EachJointAngle1[2].angle.value();
							sang4 = EachJointAngle1[3].angle.value();
							sang5 = EachJointAngle1[4].angle.value();

							sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
							sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
							sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
							sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
							sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


							c_2 = cos(sang2_cal);
							c_3 = cos(sang3_cal);
							c_4 = cos(sang4_cal);
							c_5 = cos(sang5_cal);

							s_2 = sin(sang2_cal);
							s_3 = sin(sang3_cal);
							s_4 = sin(sang4_cal);
							s_5 = sin(sang5_cal);
							s_23 = sin(sang2_cal - sang3_cal);
							s_34 = sin(sang3_cal - sang4_cal);
							s2_2 = sin(2 * sang2_cal);
							s2_5 = sin(2 * sang5_cal);
							c_23 = cos(sang2_cal - sang3_cal);
							c_34 = cos(sang3_cal - sang4_cal);
							c_45 = cos(sang4_cal + sang5_cal);
							c_4_5 = cos(sang4_cal - sang5_cal);
							c_25 = cos(sang2_cal + sang5_cal);
							c_2_5 = cos(sang2_cal - sang5_cal);
							s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
							c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
							c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
							c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
							c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
							c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
							s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
							s_223 = sin(2 * sang2_cal - sang3_cal);
							s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
							s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
							c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
							c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
							c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
							s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
							s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
							c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
							c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
							c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
							s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
							s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
							c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
							c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
							s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
							s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

							term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
							term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
							term_3 = (3969783.0*c_4);
							term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
							term_5 = (753.0*c_5);
							term_6 = (27027.0*s_4);
							term_7 = (20331.0*c_5*s_4);
							term_8 = (4557899.0*s_3*s_4);
							term_9 = (4557899.0*c_3*c_4);
							term_10 = (31031.0*c_3*s_4);
							term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
							term_12 = (31031.0*c_4*s_3);
							term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
							term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
							term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
							term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
							term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
							term_18 = (23343.0*c_3*c_5*s_4);
							term_19 = (23343.0*c_4*c_5*s_3);


							N_matrix.at<double>(0, 0) = 0.0;
							N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
							N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
							N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
							tau_matrix = N_matrix;
#pragma endregion 重力補償test



							desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
							myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
							LOG(info) << "torque test1";
							//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
							//LOG(info) << "torque test2";
							//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
							//LOG(info) << "torque test3";
							//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
							//LOG(info) << "torque test4";
							//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
							//LOG(info) << "torque test5";

							myYouBotManipulator->getJointData(EachJointVelocity1);
							svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
							svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
							svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
							svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
							svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
							myYouBotManipulator->getJointData(EachJointTorque1);
							stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
							stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
							stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
							stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
							stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣



						}
						if (J2poserror < 0.01)
						{

							if (i2 < qqq - 1)
							{
								cout << i2 << endl;
								i2++;
							}

						}

						//					if (i2 == qqq)
						//					{
						//
						//						myYouBotManipulator->getJointData(EachJointAngle1);
						//						sang1 = EachJointAngle1[0].angle.value();
						//						sang2 = EachJointAngle1[1].angle.value();
						//						sang3 = EachJointAngle1[2].angle.value();
						//						sang4 = EachJointAngle1[3].angle.value();
						//						sang5 = EachJointAngle1[4].angle.value();
						//
						//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
						//
						//
						//						c_2 = cos(sang2_cal);
						//						c_3 = cos(sang3_cal);
						//						c_4 = cos(sang4_cal);
						//						c_5 = cos(sang5_cal);
						//
						//						s_2 = sin(sang2_cal);
						//						s_3 = sin(sang3_cal);
						//						s_4 = sin(sang4_cal);
						//						s_5 = sin(sang5_cal);
						//						s_23 = sin(sang2_cal - sang3_cal);
						//						s_34 = sin(sang3_cal - sang4_cal);
						//						s2_2 = sin(2 * sang2_cal);
						//						s2_5 = sin(2 * sang5_cal);
						//						c_23 = cos(sang2_cal - sang3_cal);
						//						c_34 = cos(sang3_cal - sang4_cal);
						//						c_45 = cos(sang4_cal + sang5_cal);
						//						c_4_5 = cos(sang4_cal - sang5_cal);
						//						c_25 = cos(sang2_cal + sang5_cal);
						//						c_2_5 = cos(sang2_cal - sang5_cal);
						//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						//						s_223 = sin(2 * sang2_cal - sang3_cal);
						//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
						//
						//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						//						term_3 = (3969783.0*c_4);
						//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						//						term_5 = (753.0*c_5);
						//						term_6 = (27027.0*s_4);
						//						term_7 = (20331.0*c_5*s_4);
						//						term_8 = (4557899.0*s_3*s_4);
						//						term_9 = (4557899.0*c_3*c_4);
						//						term_10 = (31031.0*c_3*s_4);
						//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						//						term_12 = (31031.0*c_4*s_3);
						//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						//						term_18 = (23343.0*c_3*c_5*s_4);
						//						term_19 = (23343.0*c_4*c_5*s_3);
						//
						//
						//						N_matrix.at<double>(0, 0) = 0.0;
						//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						//						tau_matrix = N_matrix;
						//#pragma endregion 重力補償test
						//
						//
						//
						//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						//						//LOG(info) << "torque test1";
						//						desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//						myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//						LOG(info) << "torque test2";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//						//LOG(info) << "torque test3";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//						//LOG(info) << "torque test4";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						//						//LOG(info) << "torque test5";
						//
						//						myYouBotManipulator->getJointData(EachJointVelocity1);
						//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//						myYouBotManipulator->getJointData(EachJointTorque1);
						//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
						//					}

						if (J3poserror < 0.01)
						{

							if (i3 < qqq - 1)
							{
								cout << i3 << endl;
								i3++;
							}

						}
						//					if (i3 == qqq-1)
						//					{
						//
						//						myYouBotManipulator->getJointData(EachJointAngle1);
						//						sang1 = EachJointAngle1[0].angle.value();
						//						sang2 = EachJointAngle1[1].angle.value();
						//						sang3 = EachJointAngle1[2].angle.value();
						//						sang4 = EachJointAngle1[3].angle.value();
						//						sang5 = EachJointAngle1[4].angle.value();
						//
						//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
						//
						//
						//						c_2 = cos(sang2_cal);
						//						c_3 = cos(sang3_cal);
						//						c_4 = cos(sang4_cal);
						//						c_5 = cos(sang5_cal);
						//
						//						s_2 = sin(sang2_cal);
						//						s_3 = sin(sang3_cal);
						//						s_4 = sin(sang4_cal);
						//						s_5 = sin(sang5_cal);
						//						s_23 = sin(sang2_cal - sang3_cal);
						//						s_34 = sin(sang3_cal - sang4_cal);
						//						s2_2 = sin(2 * sang2_cal);
						//						s2_5 = sin(2 * sang5_cal);
						//						c_23 = cos(sang2_cal - sang3_cal);
						//						c_34 = cos(sang3_cal - sang4_cal);
						//						c_45 = cos(sang4_cal + sang5_cal);
						//						c_4_5 = cos(sang4_cal - sang5_cal);
						//						c_25 = cos(sang2_cal + sang5_cal);
						//						c_2_5 = cos(sang2_cal - sang5_cal);
						//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						//						s_223 = sin(2 * sang2_cal - sang3_cal);
						//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
						//
						//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						//						term_3 = (3969783.0*c_4);
						//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						//						term_5 = (753.0*c_5);
						//						term_6 = (27027.0*s_4);
						//						term_7 = (20331.0*c_5*s_4);
						//						term_8 = (4557899.0*s_3*s_4);
						//						term_9 = (4557899.0*c_3*c_4);
						//						term_10 = (31031.0*c_3*s_4);
						//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						//						term_12 = (31031.0*c_4*s_3);
						//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						//						term_18 = (23343.0*c_3*c_5*s_4);
						//						term_19 = (23343.0*c_4*c_5*s_3);
						//
						//
						//						N_matrix.at<double>(0, 0) = 0.0;
						//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						//						tau_matrix = N_matrix;
						//#pragma endregion 重力補償test
						//
						//
						//
						//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						//						//LOG(info) << "torque test1";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//						//LOG(info) << "torque test2";
						//						desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//						myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//						LOG(info) << "torque test3";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//						//LOG(info) << "torque test4";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						//						//LOG(info) << "torque test5";
						//
						//						myYouBotManipulator->getJointData(EachJointVelocity1);
						//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//						myYouBotManipulator->getJointData(EachJointTorque1);
						//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
						//
						//
						//
						//					}


						if (J4poserror < 0.01)
						{

							if (i4 < qqq - 1)
							{
								cout << i4 << endl;
								i4++;
							}

						}

						//					if (i4 == qqq-1)
						//					{
						//
						//						myYouBotManipulator->getJointData(EachJointAngle1);
						//						sang1 = EachJointAngle1[0].angle.value();
						//						sang2 = EachJointAngle1[1].angle.value();
						//						sang3 = EachJointAngle1[2].angle.value();
						//						sang4 = EachJointAngle1[3].angle.value();
						//						sang5 = EachJointAngle1[4].angle.value();
						//
						//						sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
						//						sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型
						//
						//
						//						c_2 = cos(sang2_cal);
						//						c_3 = cos(sang3_cal);
						//						c_4 = cos(sang4_cal);
						//						c_5 = cos(sang5_cal);
						//
						//						s_2 = sin(sang2_cal);
						//						s_3 = sin(sang3_cal);
						//						s_4 = sin(sang4_cal);
						//						s_5 = sin(sang5_cal);
						//						s_23 = sin(sang2_cal - sang3_cal);
						//						s_34 = sin(sang3_cal - sang4_cal);
						//						s2_2 = sin(2 * sang2_cal);
						//						s2_5 = sin(2 * sang5_cal);
						//						c_23 = cos(sang2_cal - sang3_cal);
						//						c_34 = cos(sang3_cal - sang4_cal);
						//						c_45 = cos(sang4_cal + sang5_cal);
						//						c_4_5 = cos(sang4_cal - sang5_cal);
						//						c_25 = cos(sang2_cal + sang5_cal);
						//						c_2_5 = cos(sang2_cal - sang5_cal);
						//						s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
						//						c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
						//						c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
						//						c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
						//						c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
						//						c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
						//						s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
						//						s_223 = sin(2 * sang2_cal - sang3_cal);
						//						s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
						//						c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
						//						c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
						//						s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
						//						s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
						//						s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
						//						s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
						//						c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
						//						c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
						//						s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
						//						c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
						//						s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
						//						s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);
						//
						//						term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
						//						term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
						//						term_3 = (3969783.0*c_4);
						//						term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
						//						term_5 = (753.0*c_5);
						//						term_6 = (27027.0*s_4);
						//						term_7 = (20331.0*c_5*s_4);
						//						term_8 = (4557899.0*s_3*s_4);
						//						term_9 = (4557899.0*c_3*c_4);
						//						term_10 = (31031.0*c_3*s_4);
						//						term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
						//						term_12 = (31031.0*c_4*s_3);
						//						term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
						//						term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
						//						term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
						//						term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
						//						term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
						//						term_18 = (23343.0*c_3*c_5*s_4);
						//						term_19 = (23343.0*c_4*c_5*s_3);
						//
						//
						//						N_matrix.at<double>(0, 0) = 0.0;
						//						N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
						//						N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
						//						N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
						//						N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
						//						tau_matrix = N_matrix;
						//#pragma endregion 重力補償test
						//
						//
						//
						//						//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
						//						//LOG(info) << "torque test1";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
						//						//LOG(info) << "torque test2";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
						//						//LOG(info) << "torque test3";
						//						desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
						//						myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
						//						LOG(info) << "torque test4";
						//						//desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
						//						//myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
						//						//LOG(info) << "torque test5";
						//
						//						myYouBotManipulator->getJointData(EachJointVelocity1);
						//						svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
						//						svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
						//						svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
						//						svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
						//						svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
						//						myYouBotManipulator->getJointData(EachJointTorque1);
						//						stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
						//						stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
						//						stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
						//						stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
						//						stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣
						//
						//
						//
						//					}

						if (J5poserror < 0.01)
						{

							if (i5 < qqq - 1)
							{
								cout << i5 << endl;
								i5++;
							}

						}

						if (i5 == qqq - 1)
						{

							myYouBotManipulator->getJointData(EachJointAngle1);
							sang1 = EachJointAngle1[0].angle.value();
							sang2 = EachJointAngle1[1].angle.value();
							sang3 = EachJointAngle1[2].angle.value();
							sang4 = EachJointAngle1[3].angle.value();
							sang5 = EachJointAngle1[4].angle.value();

							sang1_cal = sang1 - 2.9496;//youbot角度需要經過轉換才能帶入動力學模型
							sang2_cal = sang2 - 1.1345;//youbot角度需要經過轉換才能帶入動力學模型
							sang3_cal = (-sang3) - 2.5482;//youbot角度需要經過轉換才能帶入動力學模型
							sang4_cal = sang4 - 1.789;//youbot角度需要經過轉換才能帶入動力學模型
							sang5_cal = sang5 - 2.9234;//youbot角度需要經過轉換才能帶入動力學模型


							c_2 = cos(sang2_cal);
							c_3 = cos(sang3_cal);
							c_4 = cos(sang4_cal);
							c_5 = cos(sang5_cal);

							s_2 = sin(sang2_cal);
							s_3 = sin(sang3_cal);
							s_4 = sin(sang4_cal);
							s_5 = sin(sang5_cal);
							s_23 = sin(sang2_cal - sang3_cal);
							s_34 = sin(sang3_cal - sang4_cal);
							s2_2 = sin(2 * sang2_cal);
							s2_5 = sin(2 * sang5_cal);
							c_23 = cos(sang2_cal - sang3_cal);
							c_34 = cos(sang3_cal - sang4_cal);
							c_45 = cos(sang4_cal + sang5_cal);
							c_4_5 = cos(sang4_cal - sang5_cal);
							c_25 = cos(sang2_cal + sang5_cal);
							c_2_5 = cos(sang2_cal - sang5_cal);
							s2_23 = sin(2 * sang2_cal - 2 * sang3_cal);
							c_345 = cos(sang3_cal - sang4_cal + sang5_cal);
							c_34_5 = cos(sang3_cal - sang4_cal - sang5_cal);
							c_235 = cos(sang2_cal - sang3_cal + sang5_cal);
							c_23_5 = cos(sang2_cal - sang3_cal - sang5_cal);
							c_234 = cos(sang2_cal - sang3_cal + sang4_cal);
							s_234 = sin(sang2_cal - sang3_cal + sang4_cal);
							s_223 = sin(2 * sang2_cal - sang3_cal);
							s2_234 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
							s_2234 = sin(2 * sang2_cal - sang3_cal + sang4_cal);
							c_2234 = cos(2 * sang2_cal - sang3_cal + sang4_cal);
							c2_234 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal);
							c_22345 = cos(2 * sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							c_2234_5 = cos(2 * sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							c_2345 = cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							s_2345 = sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal);
							s_234_5 = sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							c_234_5 = cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal);
							s_234_25 = sin(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal);
							s_23425 = sin(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal);
							s_22234 = sin(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
							c_22234 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal);
							c_22234_5 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal - sang5_cal);
							c_222345 = cos(2 * sang2_cal - 2 * sang3_cal + sang4_cal + sang5_cal);
							s_2223245 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
							s_222324_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
							c_222324_5 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - sang5_cal);
							c_2223245 = cos(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + sang5_cal);
							s2_2345 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal + 2 * sang5_cal);
							s2_234_5 = sin(2 * sang2_cal - 2 * sang3_cal + 2 * sang4_cal - 2 * sang5_cal);

							term_1 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal + sang5_cal)) / 62500000.0;
							term_2 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal - sang5_cal)) / 1000000000.0;
							term_3 = (3969783.0*c_4);
							term_4 = (2259.0*(pow(c_5, 2))) / 1000000000.0;
							term_5 = (753.0*c_5);
							term_6 = (27027.0*s_4);
							term_7 = (20331.0*c_5*s_4);
							term_8 = (4557899.0*s_3*s_4);
							term_9 = (4557899.0*c_3*c_4);
							term_10 = (31031.0*c_3*s_4);
							term_11 = (753.0*cos(sang2_cal - sang3_cal + sang4_cal + sang5_cal));
							term_12 = (31031.0*c_4*s_3);
							term_13 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal - 2 * sang5_cal));
							term_14 = (2259.0*cos(sang2_cal - sang3_cal + sang4_cal + 2 * sang5_cal));
							term_15 = (2259.0*sin(sang2_cal - sang3_cal + sang4_cal - sang5_cal));
							term_16 = (20331.0*sin(sang2_cal - sang3_cal + sang5_cal));
							term_17 = (25433.0*cos(sang2_cal - sang3_cal + sang4_cal));
							term_18 = (23343.0*c_3*c_5*s_4);
							term_19 = (23343.0*c_4*c_5*s_3);


							N_matrix.at<double>(0, 0) = 0.0;
							N_matrix.at<double>(1, 0) = (25767062667123676997.0*s_2) / 11258999068426240000.0 - (88517464797832128003.0*c_2) / 56294995342131200000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (294008933574591813.0*c_2*c_3) / 562949953421312000.0 - (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0 - 57 / 1000.0)) / 56294995342131200.0 + (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 - (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((57.0*s_3) / 1000.0 - (27.0*c_3) / 100.0 + 27 / 100.0)) / 56294995342131200.0 + (294008933574591813.0*s_2*s_3) / 562949953421312000.0 + (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 + (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(2, 0) = (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 - (294008933574591813.0*c_2*c_3) / 562949953421312000.0 + (1552937978909335579.0*c_2*((57.0*c_3) / 1000.0 + (27.0*s_3) / 100.0)) / 56294995342131200.0 - (1800159961711097241.0*c_2*s_3) / 562949953421312000.0 + (1800159961711097241.0*c_3*s_2) / 562949953421312000.0 - (1552937978909335579.0*s_2*((27.0*c_3) / 100.0 - (57.0*s_3) / 1000.0)) / 56294995342131200.0 - (294008933574591813.0*s_2*s_3) / 562949953421312000.0 - (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 - (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 + (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0 - 81.0 / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((81.0*s_4) / 200.0 - (57.0*c_4) / 1000.0 + 57 / 1000.0)) / 56294995342131200.0 + (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 - (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(2, 0) = N_matrix.at<double>(2, 0)*-0.75;
							N_matrix.at<double>(3, 0) = (3082018589152638507.0*c_5*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))) / 5629499534213120000.0 - (552806158210613379.0*(c_4*(c_2*c_3 + s_2*s_3) + s_4*(c_2*s_3 - c_3*s_2))*((11.0*c_5) / 200.0 - 11.0 / 200.0)) / 56294995342131200.0 + (276066204223679841.0*c_4*(c_2*c_3 + s_2*s_3)) / 562949953421312000.0 + (100247227274517223483.0*c_4*(c_2*s_3 - c_3*s_2)) / 11258999068426240000.0 - (1037132832287244679.0*(c_2*c_3 + s_2*s_3)*((57.0*c_4) / 1000.0 - (81.0*s_4) / 200.0)) / 56294995342131200.0 - (1037132832287244679.0*(c_2*s_3 - c_3*s_2)*((81.0*c_4) / 200.0 + (57.0*s_4) / 1000.0)) / 56294995342131200.0 - (100247227274517223483.0*s_4*(c_2*c_3 + s_2*s_3)) / 11258999068426240000.0 + (276066204223679841.0*s_4*(c_2*s_3 - c_3*s_2)) / 562949953421312000.0;
							N_matrix.at<double>(3, 0) = N_matrix.at<double>(3, 0);
							N_matrix.at<double>(4, 0) = (16633887597705969.0*s_5*(c_4*(c_2*s_3 - c_3*s_2) - s_4*(c_2*c_3 + s_2*s_3))) / 2251799813685248000.0;
							tau_matrix = N_matrix;
#pragma endregion 重力補償test



							//desiredJointTorque.torque = tau_matrix.at<double>(0, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(1).setData(desiredJointTorque);
							//LOG(info) << "torque test1";
							//desiredJointTorque.torque = tau_matrix.at<double>(1, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(2).setData(desiredJointTorque);
							//LOG(info) << "torque test2";
							//desiredJointTorque.torque = tau_matrix.at<double>(2, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(3).setData(desiredJointTorque);
							//LOG(info) << "torque test3";
							//desiredJointTorque.torque = tau_matrix.at<double>(3, 0) * newton_meter;
							//myYouBotManipulator->getArmJoint(4).setData(desiredJointTorque);
							//LOG(info) << "torque test4";
							desiredJointTorque.torque = tau_matrix.at<double>(4, 0) * newton_meter;
							myYouBotManipulator->getArmJoint(5).setData(desiredJointTorque);
							LOG(info) << "torque test5";

							myYouBotManipulator->getJointData(EachJointVelocity1);
							svel1 = EachJointVelocity1[0].angularVelocity.value();/////回傳速度成為矩陣
							svel2 = EachJointVelocity1[1].angularVelocity.value();/////回傳速度成為矩陣
							svel3 = EachJointVelocity1[2].angularVelocity.value();/////回傳速度成為矩陣
							svel4 = EachJointVelocity1[3].angularVelocity.value();/////回傳速度成為矩陣
							svel5 = EachJointVelocity1[4].angularVelocity.value();/////回傳速度成為矩陣
							myYouBotManipulator->getJointData(EachJointTorque1);
							stau1 = EachJointTorque1[0].torque.value();/////回傳力矩成為矩陣
							stau2 = EachJointTorque1[1].torque.value();/////回傳力矩成為矩陣
							stau3 = EachJointTorque1[2].torque.value();/////回傳力矩成為矩陣
							stau4 = EachJointTorque1[3].torque.value();/////回傳力矩成為矩陣
							stau5 = EachJointTorque1[4].torque.value();/////回傳力矩成為矩陣



						}



						//if (abs(ss.at<double>(0, 0)) < 10 && abs(ss.at<double>(1, 0)) < 10 && abs(ss.at<double>(2, 0)) < 10 && abs(ss.at<double>(3, 0)) < 10 && abs(ss.at<double>(4, 0)) < 10)
						//
						//{

						//	if (i < qqq)
						//	{
						//		cout << i << endl;
						//		i++;
						//	}
						//	
						//}

						if (GetAsyncKeyState(0x57))	//w
						{

							return 0;
							break;

						}

					}


				}

				while (abs(error_x) > 1 || abs(error_y) > 1 || abs(error_z) > 1 || abs(error_ThetaZ) > 0.12 || abs(error_ThetaY) > 0.12 || abs(error_ThetaX) > 0.12);
#pragma endregion 速度控制

			}



		}
	}

}





