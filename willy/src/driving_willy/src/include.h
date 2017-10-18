#ifndef _MAIN_H_
#define _MAIN_H_
	
	//CPP Plugins
	#include <iostream>
	#include <queue>
	#include <string>
	#include <unistd.h>
	#include <stdlib.h>
	#include <cmath>
	#include <list>

	#include <geometry_msgs/Vector3.h>
	#include <geometry_msgs/Twist.h>
	
	
	// ROS includes
	#include <ros/ros.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <std_msgs/Float64.h>
	#include <std_msgs/UInt8.h>
	#include <std_msgs/UInt8.h>
	#include <sensor_msgs/LaserEcho.h>
	#include <sensor_msgs/image_encodings.h>
	#include <sensor_msgs/Image.h>
	#include <std_msgs/Int32MultiArray.h>

	// const sensor_msgs::LaserEcho& sonar
	// PCL includes
	#include <pcl/conversions.h>
	#include <pcl_ros/point_cloud.h>
	#include <pcl/point_types.h>

	// People Detection Includes
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>
	
	//OpenCV
	#include <opencv2/opencv.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>

	
	//Projects scripts
	#include "Interfaces/ICommand.h"
	
	#include "Controllers/WillyController.h"
	#include "Controllers/AutonomousDrivingController.h"
	//#include "Controllers/PeopleDetectorController.h"
	
	#include "Commands/Turn.h"
	#include "Commands/Forward.h"
	#include "Commands/Backward.h"
	#include "Commands/Stop.h"
	#include "Commands/Movement.h"
	#include "Commands/ForwardUntilHit.h"
	#include "peopledetect.h"


	#include <fstream>


#endif