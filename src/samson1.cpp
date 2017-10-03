#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

float d=0.21;
float x, y, theta;
float x_0, y_0;
float p_x, p_y;
bool is_origin_set;


void metaCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	p_x=msg->linear.x;
	p_y=msg->linear.y;
}

	
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	if (!is_origin_set) {
		x_0=msg->pose.pose.position.x;
		y_0=msg->pose.pose.position.y;
		is_origin_set=true;
	}

	x=msg->pose.pose.position.x-x_0;
	y=msg->pose.pose.position.y-y_0;
	theta=tf::getYaw(msg->pose.pose.orientation);
}

int main ( int argc, char **argv) {

	ros::init(argc, argv, "samson1");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);

	ros::Subscriber sub_meta = n.subscribe("/meta", 1000, metaCallback);
	ros::Subscriber sub = n.subscribe("/RosAria/pose", 1000, chatterCallback);
	ros::Rate loop_rate(10);

	p_x=0; p_y=0;
//	float p_x, p_y;
//	ROS_INFO("Coordenada x: ");
//	std::cin >> p_x;
//	ROS_INFO("Coordenada y: ");
//	std::cin >> p_y;

	float k_1=1, k_2=1, a=1;
//	ROS_INFO("Parametro k_1: ");
//	std::cin >> k_1;
//	ROS_INFO("Parametro k_2: ");
//	std::cin >> k_2;
//	ROS_INFO("Parametro a: ");
//	std::cin >> a;

	float e_x, e_y;
	float x_m, y_m;

	while (ros::ok()) {

		ros::spinOnce();

		e_x = p_x - x;
		e_y = p_y - y;

		x_m = cos(theta)*e_x+sin(theta)*e_y;
		y_m = -sin(theta)*e_x+cos(theta)*e_y;
		x_m-= d;

		geometry_msgs::Twist msg;

		msg.linear.x=k_1/(1/a+abs(x_m))*x_m;
		msg.angular.z=k_2/(1/a+abs(y_m))*y_m;

//		ROS_INFO("(x_m,y_m,e_x,e_y): [%f][%f][%f][%f]", x_m, y_m, e_x, e_y);

		chatter_pub.publish(msg);

		loop_rate.sleep();
	}

	return 0;

}
