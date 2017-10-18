#ifndef _PEOPLE_DETECTOR_CONTROLLER_H_
#define _PEOPLE_DETECTOR_CONTROLLER_H_

using namespace std; 
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

class PeopleDetectorController
{
	public:
		//Constructor
		PeopleDetectorController();

		//Deconstructor
		~PeopleDetectorController();
		//Method which executes the given command.
		void Execute(ICommand&);

		//Here's where the image comes in.
		void PeopleDetectionImageCallback(const sensor_msgs::ImageConstPtr& msg);

		void readAndDisplayImage(sensor_msgs::ImageConstPtr msg);

		//Here is the calcuation of the People detection. Like should i drive forward or turn.
		void CalculatePeopleDetection();

		//Method where the ROS Node can be given. 
		void SetNode(ros::NodeHandle *n);

private:

};

#endif
