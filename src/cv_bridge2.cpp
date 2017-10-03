#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::CascadeClassifier door_cascade;

cv_bridge::CvImage cv_image;

void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv_image.header = msg->header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    try
    {
	cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);
    	cv::resize(image,image,cvSize(0, 0),0.5,0.5);
	cv_image.image = image;
//	ROS_INFO("passou aqui");
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;

    }

		// Detect faces
		std::vector<cv::Rect> door;
		door_cascade.detectMultiScale( cv_image.image, door, 1.3, 4, 0|CV_HAAR_SCALE_IMAGE, cv::Size(0, 0));

		// Draw circles on the detected faces
		for( int i = 0; i < door.size(); i++ ){
			printf("i:%d, %d\n" , door[i].x, door[i].y);
			cv::rectangle( cv_image.image, door[i],cv::Scalar(0,0,255));
		}

//		cv::imshow("Output Window", frame);

//		char key = (char) waitKey(20);
		// Exit this loop on escape:
//		if(key == 27)
//			break;

	// Draw an example circle on the video stream
    if (cv_image.image.rows > 60 && cv_image.image.cols > 60)
      cv::circle(cv_image.image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


    // Update GUI Window
//    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//    cv::waitKey(3);

    // Output modified video stream

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter2");

  door_cascade.load("/home/labvcr2/catkin_ws/src/labvcr2/p3at_nav/config/cascade6.xml");

	ros::NodeHandle nh_;
  ros::Subscriber image_sub_ = nh_.subscribe("/image_raw/compressed", 10,
      imageCb);

	ros::Publisher image_pub_ = nh_.advertise<sensor_msgs::Image>("/output_video",1);
 


  while(ros::ok()) {
//	ROS_INFO("tambem aqui");
    ros::spinOnce();
    image_pub_.publish(cv_image.toImageMsg());


  }
  return 0;
}
