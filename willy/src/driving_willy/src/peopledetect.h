#ifndef _PEOPLEDETECT_H_
#define _PEOPLEDETECT_H_

#include <iostream>
#include <stdlib.h>
#include <cmath>

//ROS
#include "include.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include "include.h"

using namespace cv;
using namespace std;

class PeopleDetector
{
	public:

		WillyController* _controller;
		cv::HOGDescriptor hog;
		ros::NodeHandle nh_;
		ros::Subscriber image_sub_;
		ros::Subscriber depth_sub_;
		vector<Rect> found, found_filtered;
		cv::Mat copiedFrame;
		double middleDistance;
		double turnDistance;
		double forwardDistance;
		double backwardDistance;

		 typedef union U_FloatParse {
		     float float_data;
		     unsigned char byte_data[4];
		 } U_FloatConvert;

		sensor_msgs::ImageConstPtr lastDepthImage;

		//Constructor
		PeopleDetector(WillyController* Controller);
		//Deconstructor
		~PeopleDetector();
		//Depth map callback
		void depthCallback(const sensor_msgs::ImageConstPtr& msg);
		//Rgb image callback
		void rgbCallback(const sensor_msgs::ImageConstPtr& msg );
		//Movement functions triggered with detected person
		void TurnPeopleDetect(double turnDistance);
		void ForwardPeopleDetect(double forwardDistance);
		void BackwardPeopleDetect(double backwardDistance);
		// Reading depth value from given x, y and depth image
		int ReadDepthData(unsigned int width_pos, unsigned int height_pos, sensor_msgs::ImageConstPtr depth_image);
			
	private: 
};

#endif
