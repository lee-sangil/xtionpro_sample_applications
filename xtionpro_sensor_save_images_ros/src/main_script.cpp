#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;

class Communicator{
	public:
		Communicator():sync_(mySyncPolicy(10)){

			// Make folders for saving current image
			std::string pwd = ros::package::getPath("xtionpro_sensor_save_images_ros");
			folder_name_rgb = pwd + std::string("/dataset/rgb/");
			folder_name_depth = pwd + std::string("/dataset/depth/");

			std::string folder_remove_command;
			folder_remove_command = "rm -rf " + folder_name_depth;
			system(folder_remove_command.c_str());
			folder_remove_command = "rm -rf " + folder_name_rgb;
			system(folder_remove_command.c_str());

			std::string folder_create_command;
			folder_create_command = "mkdir -p " + folder_name_depth;
			system(folder_create_command.c_str());
			folder_create_command = "mkdir -p " + folder_name_rgb;
			system(folder_create_command.c_str());

			// Make rgb and depth filename log
			std::ofstream rgb_log, depth_log, asc_log;
			rgb_log.open((pwd + std::string("/dataset/rgb.txt")).c_str());
			depth_log.open((pwd + std::string("/dataset/depth.txt")).c_str());
			asc_log.open((pwd + std::string("/dataset/associations.txt")).c_str());

			rgb_log << "# color images" << std::endl;
			rgb_log << "# file" << std::endl;
			rgb_log << "# timestamp filename" << std::endl;

			depth_log << "# depth images" << std::endl;
			depth_log << "# file" << std::endl;
			depth_log << "# timestamp filename" << std::endl;

			ROS_INFO("file is successfully opened.");

			sub_depth.subscribe( nh_, "/camera/depth/image_rect", 1 );
			sub_rgb.subscribe( nh_, "/camera/rgb/image_rect_color", 1 );

			sync_.connectInput(sub_rgb, sub_depth);
			sync_.registerCallback(boost::bind(&Communicator::callback, this, _1, _2));

			ROS_INFO("initialize ROS");
		}
		~Communicator(){
			if( rgb_log.is_open() ) rgb_log.close();
			if( depth_log.is_open() ) depth_log.close();
			if( asc_log.is_open() ) asc_log.close();

			ROS_INFO("file is successfully closed.");
		}
		void callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);

	private:
		ros::NodeHandle nh_;
		message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
		message_filters::Subscriber<sensor_msgs::Image> sub_depth;
		message_filters::Synchronizer<mySyncPolicy> sync_;
		std::ofstream rgb_log, depth_log, asc_log;
		std::string folder_name_rgb, folder_name_depth;
};

void Communicator::callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth){
	cv_bridge::CvImagePtr img_ptr_rgb, img_ptr_depth;

	try{
		img_ptr_rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
		img_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream time;
	time << std::setprecision(6) << std::fixed << msg_rgb->header.stamp;
	cv::Mat color = img_ptr_rgb->image;
	cv::Mat depth = img_ptr_depth->image;

	std::string image_file_name;
	image_file_name = folder_name_depth + time.str() + ".png";
	cv::imwrite(image_file_name, depth);  
	std::cout << image_file_name << std::endl;

	image_file_name = folder_name_rgb + time.str() + ".png";
	cv::imwrite(image_file_name, color);  
	std::cout << image_file_name << std::endl;

	rgb_log << time.str() << " rgb/" << time.str() << ".png" << std::endl;
	depth_log << time.str() << " depth/" << time.str() << ".png" << std::endl;
	asc_log << time.str() << " rgb/" << time.str() << ".png " << time.str() << " depth/" << time.str() << ".png"; 

}

int main(int argc, char * argv[]){

	ros::init(argc, argv, "xtionpro_sensor_save_images");
	Communicator comm_;

	while( ros::ok() ) {
		ros::spinOnce();
		if( cv::waitKey(1) == 'q') break;
	}

	return 0;
}
