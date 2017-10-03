#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#define PI 3.141592
using namespace Eigen;
using namespace std;

vector<double> pose_x, pose_y, pose_theta;

void pose0Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	pose_x[0] = msg->pose.pose.position.x;
	pose_y[0] = msg->pose.pose.position.y;
	pose_theta[0] = tf::getYaw(msg->pose.pose.orientation);
}

void pose1Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	pose_x[1] = msg->pose.pose.position.x;
	pose_y[1] = msg->pose.pose.position.y;
	pose_theta[1] = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "P3AT_consenso");
	ros::NodeHandle n;

	int size=atoi(argv[2]);
	int index=atoi(argv[1]);

	ros::Subscriber sub[size];

	//stringstream topic;
	pose_x.push_back(0);
	pose_y.push_back(0);
	pose_theta.push_back(0);
	//topic << "/robot_" << 0 << "/amcl_pose";
	sub[0] = n.subscribe("robot_0/amcl_pose",10, pose0Callback);

	pose_x.push_back(0);
	pose_y.push_back(0);
	pose_theta.push_back(0);
	//topic << "/robot_" << 1 << "/amcl_pose";
	sub[1] = n.subscribe("robot_1/amcl_pose",10, pose1Callback);

	// Inicialmente obtem as informações sobre o conjunto do consenso

	stringstream robot;
	robot << "/robot_" << index << "/RosAria/cmd_vel";

	ros::Publisher cmd_pub = 
		n.advertise<geometry_msgs::Twist>(robot.str().c_str(),100);

	MatrixXd L(size,size);
	L << 0, 0, -1, 1;

	std::cout << size << std::endl;
	std::cout << L << std::endl;

	ros::Rate loop_rate(10);
	geometry_msgs::Twist u;
	while(ros::ok()){

		ros::spinOnce();

		MatrixXd diff(size,size);
		diff << pose_x[0], pose_y[0], pose_x[1], pose_y[1];

		MatrixXd bias(size,size);
		bias << 2, 0, 0, 0;
		diff += bias;

		diff = -L*diff;

		MatrixXd R(size,size);
		R << cos(pose_theta[index]), sin(pose_theta[index]),
			-sin(pose_theta[index]), cos(pose_theta[index]);

		MatrixXd E(size,1);

		E << diff(index,0), diff(index,1);

		MatrixXd X(size,1);
		X = R*E;

		X(0,0)-= 0.3;

		
		u.linear.x=1.8/(1+abs(X(0,0)))*X(0,0);
		u.angular.z=1/(1+abs(X(1,0)))*X(1,0);

		cmd_pub.publish(u);
		loop_rate.sleep();
	}

	return 0;
}
