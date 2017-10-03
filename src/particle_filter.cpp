#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include <nav_msgs/OccupancyGrid.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>

#define PI 3.1415926

bool PF_init, map_is_set, in_moviment;
bool is_odom_set, is_cloud_updated, is_cloud_resample, is_dt_set, flag;

sensor_msgs::PointCloud a;
nav_msgs::OccupancyGrid map_read;
geometry_msgs::Point dp, orig;
ros::Time cmd_time, cmd_time_old;

void cmdCallBack( const geometry_msgs::Twist::ConstPtr& cmd ) {

	if ( !is_dt_set) {
		cmd_time = ros::Time::now();
		cmd_time_old = ros::Time::now();
		is_dt_set =true;
	}

	cmd_time = ros::Time::now();

	double dt = (cmd_time - cmd_time_old).toSec();

	cmd_time_old = cmd_time;


	in_moviment =  fabs(cmd->linear.x)+fabs(cmd->angular.z)> 0.01;

	if ( in_moviment && is_cloud_updated ) {


//		delta.x += r0/(r0+r1+r2+1);
//		delta.y += r1/(r0+r1+r2+1);
//		delta.z += r2/(r0+r1+r2+1);

		ROS_INFO("Atualizando posição da Nuvem");
		int n=a.points.size();
		float x_max = map_read.info.width*map_read.info.resolution/2-1;
		float y_max = map_read.info.height*map_read.info.resolution/2-1;

		for ( int i=0; i<n; i++) {
			geometry_msgs::Point delta;
			delta.x = cos(a.points[i].z)*cmd->linear.x*dt; 
			delta.y = sin(a.points[i].z)*cmd->linear.x*dt;
			delta.z = cmd->angular.z*dt;
	
			float r0=rand(), r1=rand(), r2=rand();
			a.points[i].x += delta.x;
			//+ (r0+r2-r1)/(2*r0+r1+r2+1);
			a.points[i].y += delta.y;
			//+ (r1+r0-r2)/(r0+2*r1+r2+1);
			a.points[i].z += delta.z;
			//+ (r2+r1-r0)/(r0+r1+2*r2+1);

			while ( a.points[i].x > x_max)
				a.points[i].x -= 2*x_max;
			while ( a.points[i].x < -x_max )
				a.points[i].x += 2*x_max;

			if ( a.points[i].y > y_max)
				a.points[i].y -= 2*y_max;
			else if ( a.points[i].y < -y_max )
				a.points[i].y += 2*y_max;

			if ( a.points[i].z > PI )
				a.points[i].z -= 2*PI;
			else if ( a.points[i].z < -PI ) 
				a.points[i].z += 2*PI;
//			a.points[i].y %= map_read.width;
		}
		// wait stop cmd vel
		is_cloud_updated = false;
	}
}

void scanCallBack( const sensor_msgs::LaserScan::ConstPtr& scan) {

	if ( !is_cloud_updated ) {

		ROS_INFO("Atualizando pesos das amostras");
		unsigned map_size=map_read.info.width;

		float res=map_read.info.resolution;

		float o_x = map_read.info.origin.position.x;
		float o_y = map_read.info.origin.position.y;



		unsigned ns = a.points.size();
		unsigned n = (scan->angle_max - scan->angle_min)/scan->angle_increment;
		for ( unsigned j=0; j<ns; j++) {

			float sum=0;

			float angle= scan->angle_max;
			for ( unsigned idx=0; idx<=n ; idx+=120) {

				angle -= scan->angle_increment;

				unsigned m=(scan->range_max-scan->range_min)/res+1;	

				unsigned idx_x, idx_y;
				float x_tmp, y_tmp, tmp_z;
				double laser = scan->range_min;

				tmp_z= a.points[j].z;
				for (  unsigned idx_laser=0;
						idx_laser<m;
						idx_laser++)
				{
					laser += res/5;

					float tmp_x, tmp_y;
					tmp_x= ( a.points[j].x + cos(angle+tmp_z)*laser-o_x)/res;
					tmp_y= ( a.points[j].y + sin(angle+tmp_z)*laser-o_y)/res;

					x_tmp = tmp_x;
					y_tmp = tmp_y;

					unsigned id_x = (unsigned) tmp_x;
					unsigned id_y = (unsigned) tmp_y;
					idx_x = id_x;
					idx_y = id_y;

					unsigned id = id_x+map_size*id_y;

					int val= map_read.data[id];
					if ( val > 0 || val < 0) 
						break;
				}

				float diff = fabs(1 - laser/scan->ranges[idx]);
				sum += diff*diff*diff;
			}
			a.channels[0].values[j]= 1/sum;
//			ROS_INFO("%0.2f %0.2f %0.2f", sum, a.points[j].x, a.points[j].y);
		}
		is_cloud_updated = true;
		is_cloud_resample = false;
		ROS_INFO("Pesos atualizados");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_scan_to_cloud");
	ros::NodeHandle n;

	std::string map_top="/map";
	map_read=*(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_top, ros::Duration(10)));

	float x_max = (map_read.info.width*map_read.info.resolution);
	float y_max = (map_read.info.height*map_read.info.resolution);

	geometry_msgs::Point32 p;
	p.x = map_read.info.origin.position.x;
	p.y = map_read.info.origin.position.y;
	p.z = 0;
	p.x += 0.337;

	if ( !PF_init ) {

		a.header.seq = 1;
		a.header.stamp =  ros::Time::now();
		a.header.frame_id = "base_link";


		sensor_msgs::ChannelFloat32 cha;
		cha.name = "peso";
		a.channels.push_back(cha);
		int n=30;
		int m=30;
		int k=10;
		geometry_msgs::Point32 tmp;

		for ( float i=1; i<n; i++) {
			tmp.x = p.x+x_max*(i/n);
			for ( float j=1; j<m; j++) {
				tmp.y = p.y+ y_max*(j/m);
				for ( float t=0; t<k; t++) {
					tmp.z=2*PI*(1/2-t/k);
					a.points.push_back(tmp);
					a.channels[0].values.push_back(1);
				}
			}
		}
		PF_init=true;
	}

	ros::Publisher chatter_pub = 
		n.advertise<sensor_msgs::PointCloud>("/my_acml/pointcloud", 1000);

	ros::Subscriber scan_sub = 
		n.subscribe<sensor_msgs::LaserScan>("/scan", 2, scanCallBack);


	ros::Subscriber cmd_sub =
		n.subscribe<geometry_msgs::Twist>("/cmd_vel",
			10, cmdCallBack);

	std::string flag;
	ros::Rate loop_rate(10);
	is_cloud_resample=true;
	while(ros::ok() ) {

//		ROS_INFO("Loop Check");
		a.header.stamp =  ros::Time::now();
		if ( is_cloud_updated && !is_cloud_resample ) {
//		if ( 0 ) {
			ROS_INFO("Resample");
			double sum = 0;
			int n = a.points.size();
			for ( int i=0; i<n; i++) {
				sum += a.channels[0].values[i];
			}

			srand(ros::Time::now().toSec());
			for ( int j=0; j<n; j++) {

				double r = (double) rand()/RAND_MAX;
				r = (double) rand() + 1/r;

				if ( r > sum ) {
					int tmp = (r/sum);
					r -= tmp*sum;
				}
//				ROS_INFO("Loop %0.2f %0.2f %d ", r, sum, j);

				float tmp = 0;
				int idx;
				for ( int i=0; i<n; i++) {
					if ( r <= tmp ) {
						idx = i;
						break;
					}
					tmp += a.channels[0].values[i];
				}

				a.points.push_back(a.points[idx]);
				
			}

			for ( int j=0; j<n; j++) {
				a.points.erase(a.points.begin());
			}
			is_cloud_resample = true;
			ROS_INFO("Fim %f %f %f", cos(PI), cos(-PI), sin(PI), sin(-PI) );
		}

		chatter_pub.publish(a);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
