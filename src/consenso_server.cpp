#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <iostream>
#include <vector>
#include <array>

#define PI 3.141592
using namespace Eigen;
using namespace std;

vector<double> pose_x, pose_y, pose_theta;
int idx;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
	pose_x[idx] = msg->pose.pose.position.x;
	pose_y[idx] = msg->pose.pose.position.y;
	pose_theta[idx] = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "P3AT_consenso_server");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("cons_serv", 1);
	ros::Rate loop_rate(10);

	std_msgs::Int32MultiArray dat;
	int array[H][W] = {
	  {0, 1, 2, 3, 4},
	  {10, 11, 12, 13, 14},
	  {20, 21, 22, 23, 24},
	  {30, 31, 32, 33, 34}};

	// Publica a matriz de Laplace
	MatrixXd L(2,2);
	L << 0, 0, -1, 1;

	std::cout << L << std::endl;

	while(ros::ok()){
	
		ros::spinOnce();

		MatrixXd diff(2,2);
		diff << x, y, x2, y2;

		MatrixXd bias(2,2);
		bias << 1, 0, 0, 0;
		diff += bias;

		diff = -L*diff;

		MatrixXd R(2,2);
		R << cos(theta2), sin(theta2), -sin(theta2), cos(theta2);

		MatrixXd E(2,1);

		E << diff(1,0), diff(1,1);

		MatrixXd X(2,1);
		X  = R*E;

		X(1,0)-= 0.3;

		u.linear.x=1/(1+abs(X(1,0)))*X(1,0);
		u.angular.z=1/(1+abs(X(1,1)))*X(1,1);

		// X:= X - LX dt
		//
		cmd_pub.publish(u);
				
	}

	ros::spin();
	return 0;
}
