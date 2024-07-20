#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>

// オドメトリから得られる現在の位置
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::PoseStamped goal;

geometry_msgs::Twist twist; // 指令する速度、角速度
// 初期速度
double v0 = 0.0;
// 初期角速度
double w0 = 0.0;

std::map<std::string, std::string> params_;

// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// 直線追従をするために指令する速度を計算してtwistに格納
void follow_line(double x, double y, double th)
{
	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	// 直線追従のゲイン
	double k_eta = 900.00000000 / 10000;
	double k_phai = 400.00000000 / 10000;
	double k_w = 300.00000000 / 10000;

	// ロボットの最大速度、最大角速度
	double v_max = 1.00000000;
	double w_max = 6.28000000;

	// ロボットと直線の距離(実際には一定以上の距離でクリップ)
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(robot_x - x);
    else if (th == -M_PI / 2.0)
        eta = robot_x - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    if (eta > 0.40000000)
        eta = 0.40000000;
    else if (eta < -0.40000000)
        eta = -0.40000000;

	// 直線に対するロボットの向き(-M_PIからM_PIに収まるように処理)
	double phai = yaw - th;
	while (phai <= -M_PI || M_PI <= phai)
	{
		if (phai <= -M_PI)
			phai = phai + 2 * M_PI;
		else
			phai = phai - 2 * M_PI;
	}

	// 目標となるロボットの角速度と現在の角速度の差
	double w_diff = w0;

	// 角速度
	double w = w0 + (-k_eta * eta - k_phai * phai - k_w* w_diff);
	if (w > w_max)
		w = w_max;
	else if (w < -w_max)
		w = -w_max;

	// 並進速度
	double v = v0;
//	if (v0 != 0)
//		v = v0 - v0 / abs(v0) * abs(w0);
	if (v > v_max)
		v = v_max;
	else if (v < -v_max)
		v = -v_max;

	twist.linear.x = v;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w;

	// 現在の角速度を次の時間の計算に使用
	w0 = twist.angular.z;

	// デバック用のプリント
	// std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	// std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;
	std::cout << "(x,y) = (" << robot_x << "," << robot_y << ")" << std::endl;
	// std::cout << "------------------------------" << std::endl;

}

int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.03);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_line_ver2");
	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 100);

	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	// std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;

	ros::Rate loop_rate(100);

	// 速度
	v0 = 0.3;
	// 現在の角速度
	w0 = twist.angular.z;
	
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;

	goal.pose.position.x = 10.0;
    goal.pose.position.y = 0.0;

	while (ros::ok())
	{
		ros::spinOnce();
		follow_line(x, y, theta);

		if (near_position(goal))
        {
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
        }
		twist_pub.publish(twist);
		loop_rate.sleep();
	}

	return 0;
}