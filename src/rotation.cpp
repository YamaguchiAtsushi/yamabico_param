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

double robot_x, robot_y;
double roll, pitch, yaw;
int flag=0,count=0;
geometry_msgs::Quaternion robot_r;
geometry_msgs::Twist twist; // 指令する速度、角速度

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
void rotation()
{
	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.6;

	// デバック用のプリント
	// std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	// std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;
	// std::cout << "(x,y) = (" << robot_x << "," << robot_y << ")" << std::endl;
	// std::cout << "------------------------------" << std::endl;
	if(-0.1<yaw && yaw<-0.01){
		flag=1;

	}else if(-0.01<=yaw && flag==1){
		flag=0;
		count++;	// 回転数
	}

	if(count>=10){
		twist.angular.z=0.0;
	}

	printf("yaw:%.2f count:%d\n",yaw,count);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rotation");
	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("/ypspur_ros/odom", 100, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 100);

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

	while (ros::ok())
	{
		ros::spinOnce();
		rotation();
		twist_pub.publish(twist);
		loop_rate.sleep();
	}

	return 0;
}