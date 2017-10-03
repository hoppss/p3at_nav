#include <iostream>
//#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

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
double r0uv,r1uv,r2uv;
double r0uw,r1uw,r2uw;

double r0dX=2;
double r1dX=1;
double r2dX=3;

double r0dY=3;
double r1dY=1;
double r2dY=1;

void cmd_vel(unsigned char  r, double vl, double va)		
{
	if(r==0)
    {
		cmd_vel_msg.linear.x=vl;
		cmd_vel_msg.angular.z=va;
        r0_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==1)
    {
		cmd_vel_msg.linear.x=vl;
		cmd_vel_msg.angular.z=va;
        r1_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==2)
    {
		cmd_vel_msg.linear.x=vl;
		cmd_vel_msg.angular.z=va;
        r2_cmd_vel_pub.publish(cmd_vel_msg);
    }
    if(r==3)
    {
		cmd_vel_msg.linear.x=vl;
		cmd_vel_msg.angular.z=va;
        r3_cmd_vel_pub.publish(cmd_vel_msg);
    }
}

void r0_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r0X=msg->pose.pose.position.x;
    r0Y=msg->pose.pose.position.y;

    r0O_=tf::getYaw(msg->pose.pose.orientation);

    r0O=r0O_*180/pi;

    if(r0O<0)
    {
		r0O=360-abs(r0O);
    }
	r0O=r0O*pi/180;
}

void r1_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r1X=msg->pose.pose.position.x;
    r1Y=msg->pose.pose.position.y;

    r1O_=tf::getYaw(msg->pose.pose.orientation);

    r1O=r1O_*180/pi;

    if(r1O<0)
    {
		r1O=360-abs(r1O);
    }

	r1O=r1O*pi/180;
}

void r2_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r2X=msg->pose.pose.position.x;
    r2Y=msg->pose.pose.position.y;

    r2O_=tf::getYaw(msg->pose.pose.orientation);

    r2O=r2O_*180/pi;

    if(r2O<0)
    {
		r2O=360-abs(r2O);
    }

	r2O=r2O*pi/180;
}

void r3_pose (const nav_msgs::Odometry::ConstPtr& msg)
{
    r3X=msg->pose.pose.position.x;
    r3Y=msg->pose.pose.position.y;

    r3O_=tf::getYaw(msg->pose.pose.orientation);

    r3O=r3O_*180/pi;

    if(r3O<0)
    {
		r3O=360-abs(r3O);
    }

	r3O=r3O*pi/180;
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

	ros::Rate loop_rate(10);
	while(ros::ok())
    {
		ros::spinOnce();

		r0uX = (r1X-r0X)+(r2X-r0X)+(r0dX-r1dX)+(r0dX-r2dX);
		r1uX = (r0X-r1X)+(r2X-r1X)+(r1dX-r0dX)+(r1dX-r2dX);
		r2uX = (r0X-r2X)+(r1X-r2X)+(r2dX-r0dX)+(r2dX-r1dX);

		r0uY = (r1Y-r0Y)+(r2Y-r0Y)+(r0dY-r1dY)+(r0dY-r2dY);
		r1uY = (r0Y-r1Y)+(r2Y-r1Y)+(r1dY-r0dY)+(r1dY-r2dY);
		r2uY = (r0Y-r2Y)+(r1Y-r2Y)+(r2dY-r0dY)+(r2dY-r1dY);			

		r0uv = (r0uX*cos(r0O_)) + (r0uY*sin(r0O_));
		r1uv = (r1uX*cos(r1O_)) + (r1uY*sin(r1O_));
		r2uv = (r2uX*cos(r2O_)) + (r2uY*sin(r2O_));

		r0uw = (r0uY*cos(r0O_) - r0uX*sin(r0O_))/0.1;
		r1uw = (r1uY*cos(r1O_) - r1uX*sin(r1O_))/0.1;
		r2uw = (r2uY*cos(r2O_) - r2uX*sin(r2O_))/0.1;
		
		cmd_vel(0,r0uv,r0uw);
		cmd_vel(1,r1uv,r1uw);
		cmd_vel(2,r2uv,r2uw);

		loop_rate.sleep();
	}
}

