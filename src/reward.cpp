#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/ChannelFloat32.h>

typedef struct RGB {
	uchar red;
	uchar green;
	uchar blue;
} RGB;

cv::CascadeClassifier door_cascade;
cv_bridge::CvImage cv_image;
sensor_msgs::ChannelFloat32 reward;

void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
{
	cv_image.header = msg->header;
	cv_image.encoding = sensor_msgs::image_encodings::BGR8;

	try {
		cv::Mat image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_COLOR);
		cv::resize ( image, image, cvSize(0, 0), 0.2, 0.2);

		float rw=0;
		cv::Size sz= image.size();
		for ( int i=0; i<sz.width; i++) {
			for ( int j=0; j<sz.height; j++) {
				cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(i,j));
				if ( color[0]<100 && color[1]>150  && color[2] > 150) {
					color[0]=0;
					color[1]=0;
					color[2]=0;
					rw+=0.1;

					image.at<cv::Vec3b>(cv::Point(i,j)) = color;
				}
			}
		}
		if ( reward.values.size() > 0 ) {
			reward.values[0]=rw;
		}
		else {
			reward.name="Reward";
			reward.values.push_back(rw);
		}

		cv_image.image = image;
	} catch ( cv_bridge::Exception& e ) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	/*
	// Detect faces
	std::vector<cv::Rect> door;
	door_cascade.detectMultiScale( cv_image.image, 
			door,
			1.3,
			4,
			0 |CV_HAAR_SCALE_IMAGE, 
			cv::Size(0, 0));
	// Draw circles on the detected faces
	for( int i = 0; i < door.size(); i++ ){
		printf("i:%d, %d\n" , door[i].x, door[i].y);
		cv::rectangle( cv_image.image, door[i],cv::Scalar(0,0,255));
	}
*/

	// Draw an example circle on the video stream
	/*
	if (cv_image.image.rows > 60 && cv_image.image.cols > 60)
		cv::circle(cv_image.image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter2");

//	door_cascade.load("/home/labvcr2/catkin_ws/src/labvcr2/p3at_nav/config/cascade6.xml");

	ros::NodeHandle nh_;
//	ros::Subscriber enable_reward_sub_ = nh_.subscribe("/enable_reward", 10,
//			enable_reward);
	ros::Subscriber image_sub_ = nh_.subscribe("/image_raw/compressed", 10,
			imageCb);

	ros::Publisher reward_pub_ = nh_.advertise<sensor_msgs::ChannelFloat32>("/reward",1);
	ros::Publisher image_pub_ = nh_.advertise<sensor_msgs::Image>("/output_video",1);

	while(ros::ok()) {
		ros::spinOnce();
			reward_pub_.publish(reward);
			image_pub_.publish(cv_image.toImageMsg());
	}
  return 0;
}
