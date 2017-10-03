#include <iostream>
//#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

#define pi	3.14
#define N	3

ros::Publisher r0_cmd_vel_pub;
ros::Publisher r1_cmd_vel_pub;
ros::Publisher r2_cmd_vel_pub;
ros::Publisher r3_cmd_vel_pub;

ros::Subscriber r0_pose_sub;
ros::Subscriber r1_pose_sub;
ros::Subscriber r2_pose_sub;
ros::Subscriber r3_pose_sub;

geometry_msgs::Twist cmd_vel_msg;

double r0X, r0Y, r0O, r0O_;
double r1X, r1Y, r1O, r1O_;
double r2X, r2Y, r2O, r2O_;
double r3X, r3Y, r3O, r3O_;

double r0uX,r1uX,r2uX;
double r0uY,r1uY,r2uY;

void vel_linear(unsigned char  r, double vx, double vy)		
{
	if(r==0)
    {
		cmd_vel_msg.linear.x=vx;
		cmd_vel_msg.linear.y=vy;
        r0_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==1)
    {
		cmd_vel_msg.linear.x=vx;
		cmd_vel_msg.linear.y=vy;
        r1_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==2)
    {
		cmd_vel_msg.linear.x=vx;
		cmd_vel_msg.linear.y=vy;
        r2_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==3)
    {
		cmd_vel_msg.linear.x=vx;
		cmd_vel_msg.linear.y=vy;
        r3_cmd_vel_pub.publish(cmd_vel_msg);
    }
}

void r0_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r0X=msg->pose.pose.position.x;
    r0Y=msg->pose.pose.position.y;

    r0O_=tf::getYaw(msg->pose.pose.orientation);

    r0O_=r0O_*180/pi;

    r0O=r0O_+90;

    if(r0O>180)
    {
		r0O=r0O-360;
    }
}

void r1_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r1X=msg->pose.pose.position.x;
    r1Y=msg->pose.pose.position.y;

    r1O_=tf::getYaw(msg->pose.pose.orientation);

    r1O_=r1O_*180/pi;

    r1O=r1O_+90;

    if(r1O>180)
    {
		r1O=r1O-360;
    }
}

void r2_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r2X=msg->pose.pose.position.x;
    r2Y=msg->pose.pose.position.y;

    r2O_=tf::getYaw(msg->pose.pose.orientation);

    r2O_=r2O_*180/pi;

    r2O=r2O_+90;

    if(r2O>180)
    {
		r2O=r2O-360;
    }
}

void r3_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r3X=msg->pose.pose.position.x;
    r3Y=msg->pose.pose.position.y;

    r3O_=tf::getYaw(msg->pose.pose.orientation);

    r3O_=r3O_*180/pi;

    r3O=r3O_+90;

    if(r3O>180)
    {
		r3O=r3O-360;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "consenso0");
    ros::NodeHandle nh;

	r0_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);
	r1_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);
	r2_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 10);
	r3_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 10);

	r0_pose_sub = nh.subscribe("/robot_0/base_pose_ground_truth", 10, r0_pose);
	r1_pose_sub = nh.subscribe("/robot_1/base_pose_ground_truth", 10, r1_pose);
	r2_pose_sub = nh.subscribe("/robot_2/base_pose_ground_truth", 10, r2_pose);
	r3_pose_sub = nh.subscribe("/robot_3/base_pose_ground_truth", 10, r3_pose);
	MatrixXd L(3,3);
	MatrixXd R(2,2);
	L << 2, -1, -1, -1, 2, -1, -1, -1, 2;
//	std::cout << L << std::endl;
	double d=0.1;

	ros::Rate loop_rate(10);
	while(ros::ok())
    {
		ros::spinOnce();

		MatrixXd diff(3,2);
		diff << r0X, r0Y, r1X, r1Y, r2X, r2Y;

		MatrixXd bias(3,2);
		bias << 2, 1, 1, 3, 3, 3;
		diff -= bias;


//		R << cos(r00_), sin(r00_), cos(r00_)/d, -sin(r00_)/d;

		diff = -L*diff;

		vel_linear(0,diff(0,0),diff(0,1));
		vel_linear(1,diff(1,0),diff(1,1));
		vel_linear(2,diff(2,0),diff(2,1));
		
		loop_rate.sleep();
	}
}
