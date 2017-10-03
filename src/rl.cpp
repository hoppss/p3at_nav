#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <axis_camera/Axis.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class state {
	geometry_msgs::Pose pose;
	public:

	state () {}

	state (float x, float y, float theta) {
		pose.position.x = x;
		pose.position.y = y;
		pose.orientation.x = theta;
	}

	state operator-(state &s2) {
		state tmp;
		for ( int i=0; i<3; i++) {
			tmp.set_coord(i, this->get_coord(i) - s2.get_coord(i));
		}
		return tmp;
	}

	float get_coord(int i) {
		switch ( i) 
		{
			case 0:
				return pose.position.x;
				break;
			case 1:
				return pose.position.y;
				break;
			case 2:
				return pose.position.z;
				break;
			case 3:
				return pose.orientation.x;
				break;
			case 4:
				return pose.orientation.y;
				break;
			case 5:
				return pose.orientation.z;
				break;
			case 6:
				return pose.orientation.w;
				break;
			default:;
		}
	}

	void set_coord(int i, float a) {
		switch (i) 
		{
			case 0:
				pose.position.x=a;
				break;
			case 1:
				pose.position.y=a;
				break;
			case 2:
				pose.position.z=a;
				break;
			case 3:
				pose.orientation.x=a;
				break;
			case 4:
				pose.orientation.y=a;
				break;
			case 5:
				pose.orientation.z=a;
				break;
			case 6:
				pose.orientation.w=a;
				break;
			default:;
		}
	}
	
	state operator*(float M[3][3] ) {
		state tmp;
		for ( int i=0; i<3; i++) {
			float aux=0;
			for ( int j=0; j<3; j++) {
				aux+=M[i][j]*this->get_coord(i);
			}
			tmp.set_coord(i, aux);
		}
		return tmp;
	}

	bool operator==(state &s2) {
		int s=0;
		for ( int i=0; i<7; i++) 
			s += (this->get_coord(i) == s2.get_coord(i));

		if ( s == 7 )
			return true;
		else
			return false;
	}

	float norm2() {
		return
			pose.position.x*pose.position.x+pose.position.y*pose.position.y;
	}

	void print() {
		std::cout << "(";
		for ( int i=0; i<2; i++) {
			std::cout << this->get_coord(i) << ",";
		}
		std::cout << this->get_coord(2) << ")";
		std::cout << "(";
		for ( int i=3; i<6; i++) {
			std::cout << this->get_coord(i) << ",";
		}
		std::cout << this->get_coord(6) << ")";
		std::cout << std::endl;
	}
};


class observador {

	ros::NodeHandle n;
	ros::Subscriber image;

	cv_bridge::CvImagePtr cv_ptr;
	double reward;

	public:
	observador () {}

	observador (const char* image_topic) {
		image = n.subscribe(image_topic, 10, &observador::imagecb, this);
	}

	//void imagecb(const sensor_msgs::CompressedImageConstPtr& msg) {
	void imagecb(const sensor_msgs::ImageConstPtr& msg) {
		/*
		cv_image.header = msg->header;
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;

		try {

			cv::Mat image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_COLOR);
			//cv::resize ( image, image, cvSize(0, 0), 0.2, 0.2);
			cv_image.image=image;

		} catch ( cv_bridge::Exception& e ) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		*/
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::TYPE_8UC4);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
	}

	double run() {
		ros::spinOnce();
		reward=0;
		cv::Size sz= cv_ptr->image.size();
		ROS_INFO("amarelo %d %d", sz.height, sz.width);
		for ( int i=0; i<sz.width; i++) {
			for ( int j=0; j<sz.height; j++) {

				cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(cv::Point(i,j));
				if ( color[0]<100 && color[1]>150  && color[2] > 150) {
					//color[0]=0;
					//color[1]=0;
					//color[2]=0;
					reward+=0.1;
					//cv_image.image.at<cv::Vec3b>(cv::Point(i,j)) = color;
				}
			}
		}
		return reward;
	}
};

class localizador {

	ros::NodeHandle n;
	ros::Subscriber pose_;
	ros::Subscriber ptz_;

	state pose;

	public:

	localizador () {}

	localizador ( const char* pose_topic, const char* ptz_topic) {
		pose_= n.subscribe(pose_topic, 1000, &localizador::chatterCallback, this);
		ptz_= n.subscribe(ptz_topic, 1000, &localizador::ptzCallback, this);
	}

	state get_pose() {
		ros::spinOnce();
		ros::spinOnce();
		return pose;
	}

	void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		pose.set_coord(0,msg->pose.pose.position.x);
		pose.set_coord(1,msg->pose.pose.position.y);
		pose.set_coord(2, tf::getYaw(msg->pose.pose.orientation));
	}
	/*
	// Pode ser usado para pegar a saida do amcl
	void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		pose.set_coord(0,msg->pose.pose.position.x);
		pose.set_coord(1,msg->pose.pose.position.y);
		pose.set_coord(2, tf::getYaw(msg->pose.pose.orientation));
	}

	*/
	void ptzCallback(const axis_camera::Axis::ConstPtr& msg) {
		pose.set_coord(3,msg->pan);
		pose.set_coord(4,msg->tilt);
		pose.set_coord(5,msg->zoom);
		pose.set_coord(6,msg->focus);
	}

};

class controlador {

	ros::NodeHandle n;
	ros::Publisher cmd;
	ros::Publisher cam;
	public:

	controlador (const char* cmd_topic, const char* cam_topic) { 
		cmd = n.advertise<geometry_msgs::Twist>(cmd_topic, 1000);
		cam = n.advertise<axis_camera::Axis>(cam_topic, 1000);
	}

	void set_speed( float x, float theta) {
		geometry_msgs::Twist msg;
		msg.linear.x=x;
//		msg.angular.z=theta;
		cmd.publish(msg);
	}

	void set_cam ( float pan, float tilt, float zoom) {
//		ROS_INFO("Set cam: %f %f %f", pan, tilt, zoom);
		axis_camera::Axis opt;
		opt.pan = pan;
		opt.tilt = tilt;
		opt.zoom = zoom;
		cam.publish(opt);
	}
};


class navegador : controlador, localizador {

	float k_1, k_2, d;
	public:

	navegador ( const char* cmd_topic, const char* cam_topic,
			const char* pose_topic, const char* ptz_topic) :
		controlador(cmd_topic, cam_topic),
		localizador(pose_topic, ptz_topic)
	{
		k_1=1; k_2=1; d=0.21;
	}

	void run ( state p) {
		int epochs=0;
		ros::Rate loop_rate(10);
		while ( ros::ok() ) {

			state odom = this->get_pose();

			state e = p-odom;

			float theta = odom.get_coord(2);

			float M[3][3] = {
				{cos(theta),sin(theta),0},
				{-sin(theta),cos(theta),0},
				{0,0,1}};

			state m = e*M;

			float tmp[3];
			for ( int i=0; i<3; i++)
				tmp[i]=m.get_coord(i);

			tmp[0]-= d;

			this->set_speed( k_1/(1+abs(tmp[0]))*tmp[0], k_2/(1+abs(tmp[1]))*tmp[1]);

			if ( e.norm2() < 0.1 ) {
				epochs++;
				if ( epochs>20 ) {
					this->set_speed(0,0);
					break;
				}
			}
			loop_rate.sleep();
		}

		/*
		ros::Rate loop_rate_cam(1);
		while ( ros::ok() ) {
//			ROS_INFO("Loop camera");
			state odom = this->get_pose();
//this->controlador::set_cam(p.get_coord(3), p.get_coord(4), p.get_coord(5));
			this->controlador::set_cam( (float) p.get_coord(3),
					p.get_coord(4), 1.0);

			float aux=0;
			for ( int i=3; i<5; i++) {
				float tmp = (odom.get_coord(i)-p.get_coord(i));
				aux+=tmp*tmp;
			}
			float tmp = (odom.get_coord(5)-1.0);
//			ROS_INFO("Tmp: %f", tmp);
			aux+=tmp*tmp;

//			ROS_INFO("Cam. Error: %f", aux);
			if ( aux < 1e-2 )
				break;

			loop_rate_cam.sleep();
		}
	*/
	}

	state get_pose() {
		return this->localizador::get_pose();
	}

	void set_speed(float x, float theta) {
		this->controlador::set_speed(x,theta);
	}

};


// Teoricamente a função de reward deveria ser do ambiente
class enviorment : navegador, observador{

	std::vector<state> states;

	public:

	// Ler o arquivo de configuração e setar os estados e ações possíveis.
	enviorment (const char* configFile, 
			const char* cmd, const char* cam,
			const char* pose, const char* ptz, const char* image):
		navegador(cmd, cam, pose, ptz), observador(image)
	{

		std::ifstream stateFile( configFile, std::ifstream::in);
	
		float tmp[3];

		while ( stateFile >> tmp[0] >> tmp[1] >> tmp[2]) {
			state aux(tmp[0], tmp[1], tmp[2]);
			states.push_back(aux);
		}
		stateFile.close();
	}

	// ROS: executa o deslocamento do veículo
	void simulate ( int state_idx) {
		this->navegador::run(states[state_idx]);
	}

	float evaluate ( ) {
		return this->observador::run();
	}

	int size() {
		return states.size();
	}

	void print() {
		for ( int i=0; i<states.size(); i++) {
			states[i].print();
		}
	}

};

class qlearning : enviorment {

	int current_state;
	std::vector<std::vector<float> > Q;
	std::vector<int> I_max;
	std::vector<float> Q_max;

	public:

	qlearning (const char *configFile, const char* qconfigFile,
			const char* cmd, const char* cam,
			const char* pose, const char* ptz, const char* image) :
		enviorment(configFile, cmd, cam, pose, ptz, image)
	{

		int ns=enviorment::size();

		std::ifstream qFile(qconfigFile, std::ifstream::in);

		ROS_INFO("tamanho %d", ns);
		// Pode ser adicionado posteriormente um metodo de salvar o treinamento
		// de preferência no destrutor da classe
		for ( int i=0; i<ns; i++) {
			std::vector<float> tmp_vec(ns);
			for ( int j=0; j<ns; j++) {
				qFile >> tmp_vec[j];
			}
			Q.push_back(tmp_vec);

			float q_max=-1e5;
			int i_max=-1;
			for ( int j=0; j<Q[i].size(); j++) {
				if ( q_max < Q[i][j] ) {
					q_max = Q[i][j];
					i_max = j;
				}
			}
			I_max.push_back(i_max);
			Q_max.push_back(q_max);

		}
		current_state=0;
		this->enviorment::simulate(current_state);
	}

	void exec_mission ( std::vector<int> states_idx ) {
		float gamma=0.8;
		for ( int i=0; i<states_idx.size(); i++) {
			float rw=this->enviorment::evaluate(), q_max;
//			ROS_INFO("evaluation: %f", rw);
			q_max = rw + gamma*Q_max[states_idx[i]];
			if ( q_max > Q_max[current_state] ) {
				I_max[current_state]=states_idx[i];
				Q_max[current_state]=q_max;
			}

			Q[current_state][states_idx[i]]=q_max;

			this->enviorment::simulate(states_idx[i]);
			current_state=states_idx[i];
			this->print();
		}
	}

	void exec () {

		int next_state = I_max[current_state];
		do {
			this->enviorment::simulate(next_state);

		} while ( next_state != I_max[current_state] );

	}

	void print() {
		std::cout << "Q-function" << std::endl;
		for ( int i=0; i<Q.size(); i++) {
			for ( int j=0; j<Q[i].size(); j++) {
				std::cout << std::setw(8) << std::setfill(' ') << std::fixed <<
					std::setprecision(2) << Q[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "Current state: " << current_state << std::endl;
//		std::cout << "Q*" << std::endl;
//		for ( int i=0; i<Q_max.size(); i++) {
//			std::cout << I_max[i] << ":" << Q_max[i] << std::endl;
//		}
	}
};

int main( int argc, char** argv) {

	ros::init(argc, argv, "meta");

	ros::NodeHandle n;

	std::string image="/robot_0/image";
	//std::string cmd="/RosAria/cmd_vel";
	std::string cmd="/robot_0/cmd_vel";
	std::string pose="/robot_0/odom";
	std::string ptz="/state";
	std::string cam="/cmd";

	if ( argc > 2 ) {
		qlearning achar_lixo(argv[1], argv[2], 
				cmd.c_str(), cam.c_str(),
				pose.c_str(), ptz.c_str(), image.c_str());
		achar_lixo.print();
		/*
		std::vector<int> missao;
		for ( int i=0; i<1; i++) {
			missao.push_back(1);
			missao.push_back(0);
			missao.push_back(2);
			missao.push_back(0);
			missao.push_back(3);
			missao.push_back(0);
			missao.push_back(4);
			missao.push_back(0);
			missao.push_back(5);
			missao.push_back(0);
			missao.push_back(6);

			missao.push_back(0);
			missao.push_back(1);
			missao.push_back(1);
			missao.push_back(2);
			missao.push_back(1);
			missao.push_back(3);
			missao.push_back(1);
			missao.push_back(4);
			missao.push_back(1);
			missao.push_back(5);
			missao.push_back(1);
			missao.push_back(6);

			missao.push_back(1);
			missao.push_back(2);
			missao.push_back(2);
			missao.push_back(3);
			missao.push_back(2);
			missao.push_back(4);
			missao.push_back(2);
			missao.push_back(5);
			missao.push_back(2);
			missao.push_back(6);


			missao.push_back(2);
			missao.push_back(3);
			missao.push_back(3);
			missao.push_back(4);
			missao.push_back(3);
			missao.push_back(5);
			missao.push_back(3);
			missao.push_back(6);

			missao.push_back(3);
			missao.push_back(4);
			missao.push_back(4);
			missao.push_back(5);
			missao.push_back(4);
			missao.push_back(6);

			missao.push_back(4);
			missao.push_back(5);
			missao.push_back(5);
			missao.push_back(6);
			missao.push_back(6);

			missao.push_back(5);
		}
		achar_lixo.exec_mission(missao);
			*/
	
		achar_lixo.exec();
	}

	return 0;
}
