#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>


// user-defined parameters
bool toVisualize = true;
bool toSave = true;

struct timeval ti, tf;
void tic(){
	gettimeofday(&ti, NULL);
}
unsigned long toc(){
	gettimeofday(&tf, NULL);
	return (tf.tv_sec-ti.tv_sec)*1e6 + tf.tv_usec-ti.tv_usec;
}

int main(int argc, char * argv[])
{
	if( argc == 2 )
		toVisualize = (strcmp(argv[1],"1")==0);

	ti.tv_sec = 0;
	ti.tv_usec = 0;

	// Set RGB-D camera device
	system("killall XnSensorServer");
	cv::VideoCapture dev;

	dev.open( CV_CAP_OPENNI_ASUS );
	if( !dev.isOpened() ) {
		printf("\n\nCannot find the RGBD camera. \n");
		printf("Finish This Program. Thank you!! \n\n");
		dev.release();
		return -1;
	}
	dev.set( CV_CAP_PROP_OPENNI_REGISTRATION, 1 );
	dev.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );

	// Get RGB-D camera properties
	double mDepthFrameWidth = dev.get( CV_CAP_PROP_FRAME_WIDTH );
	double mDepthFrameHeight = dev.get( CV_CAP_PROP_FRAME_HEIGHT );
	double mDepthFocalLength = dev.get( CV_CAP_PROP_OPENNI_FOCAL_LENGTH );
	double mDepthBaseline = dev.get( CV_CAP_PROP_OPENNI_BASELINE );
	double mDepthFrameMaxDepth = dev.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH );
	double mDepthFps = dev.get( CV_CAP_PROP_FPS );
	bool mRegistration = dev.get( CV_CAP_PROP_OPENNI_REGISTRATION );

	double mRgbFrameWidth = dev.get( CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_WIDTH );
	double mRgbFrameHeight = dev.get( CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT );
	double mRgbFps = dev.get( CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FPS );

	// show all Depth and RGB image properties
	std::cout << "\nDepth generator output mode:" << std::endl <<
	"FRAME_WIDTH      " << mDepthFrameWidth << std::endl <<
	"FRAME_HEIGHT     " << mDepthFrameHeight << std::endl <<
	"FOCAL_LENGTH     " << mDepthFocalLength << " pixel" << std::endl <<
	"BASELINE         " << mDepthBaseline << " mm" << std::endl <<
	"FRAME_MAX_DEPTH  " << mDepthFrameMaxDepth << " mm" << std::endl <<
	"FPS              " << mDepthFps << std::endl <<
	"REGISTRATION     " << mRegistration << std::endl;

	if( dev.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) ) {
		std::cout << "\n\nRGB Image generator output mode:" << std::endl <<
		"FRAME_WIDTH      " << mRgbFrameWidth << std::endl <<
		"FRAME_HEIGHT     " << mRgbFrameHeight << std::endl <<
		"FPS              " << mRgbFps << std::endl;
	}
	else {
		std::cout << "\nDevice doesn't contain RGB image generator." << std::endl;
		dev.release();
		return -1;
	}


	// Make folders for saving current image
	std::string folder_name_depth = "images/depth/";
	std::string folder_name_rgb = "images/rgb/";

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
	std::ofstream rgb_log, depth_log;
	rgb_log.open("images/rgb.txt");
	depth_log.open("images/depth.txt");

	rgb_log << "# color images" << std::endl;
	rgb_log << "# file" << std::endl;
	rgb_log << "# timestamp filename" << std::endl;

	depth_log << "# depth images" << std::endl;
	depth_log << "# file" << std::endl;
	depth_log << "# timestamp filename" << std::endl;

	//
	unsigned long time;
	cv::Mat depth_image, color_image;
	char image_index[255];
	std::string image_file_name;

	while (true) {
		// Wait for new frame data
		dev.grab();
		time = toc();

		// Retrieve our images
		dev.retrieve(depth_image, CV_CAP_OPENNI_DEPTH_MAP);
		dev.retrieve(color_image, CV_CAP_OPENNI_BGR_IMAGE);

		// Show current images
		if (toVisualize) {
			cv::Mat depth_image_show;
			depth_image.convertTo(depth_image_show, CV_64FC1);

			cv::imshow("xtionpro-Depth", depth_image_show / 10000);
			cv::imshow("xtionpro-RGB", color_image);
			cv::waitKey(1);
		}

		// Save current images
		if (toSave) {
			sprintf(image_index, "%016ld", time);

			image_file_name = folder_name_depth + image_index + ".png";
			cv::imwrite(image_file_name, depth_image);  
			std::cout << image_file_name << std::endl;

			image_file_name = folder_name_rgb + image_index + ".png";
			cv::imwrite(image_file_name, color_image);  
			std::cout << image_file_name << std::endl;

			rgb_log << image_index << " rgb/" << image_index << ".png" << std::endl;
			depth_log << image_index << " depth/" << image_index << ".png" << std::endl;
		}

    // Close this prgoram
		if(cv::waitKey(5) == 'q') {
			break;
		}
	}


	return 0;
}
