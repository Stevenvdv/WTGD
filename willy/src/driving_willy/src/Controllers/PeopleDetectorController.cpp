#include "../include.h"

using namespace std; 
using namespace cv;

//Constructor

PeopleDetectorController::PeopleDetectorController(){

    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    cv::namedWindow(OPENCV_WINDOW);
    cvStartWindowThread();

	
}

PeopleDetectorController::~PeopleDetectorController()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

// Method which executes the given command.

void PeopleDetectorController::Execute(ICommand& command){
	command.Execute();
}


//Here's where the image comes in.
void PeopleDetectorController::PeopleDetectionImageCallback(const sensor_msgs::ImageConstPtr& msg) { 

	readAndDisplayImage(msg);

}
void PeopleDetectorController::readAndDisplayImage(sensor_msgs::ImageConstPtr msg)

	{ 
      for (unsigned i = 0; i < found.size(); ++i)
      {
        found.clear();
        found_filtered.clear();
      }

      Point MiddlePoint(180 , 120);
      cv_bridge::CvImagePtr cv_ptr;
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::resize(cv_ptr->image, copiedFrame , Size(360,240));        
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }

      // cv::cvtColor(copiedFrame, copiedFrame, cv::COLOR_RGB2GRAY);
      hog.detectMultiScale(copiedFrame, found, 0, Size(8,8), Size(32,32), 1.05, 2);

        for(size_t i = 0; i < found.size(); i++)
        { 
            Rect r = found[i];
            PeopleDetectionX = (r.x + r.width*0.5);
            PeopleDetectionY = (r.y + r.height*0.5);
            
            size_t j;
            // Do not add small detections inside a bigger detection.
            for ( j = 0; j < found.size(); j++ )
                if ( j != i && (r & found[j]) == r )
                    break;

            if ( j == found.size() )
              if( abs(180 - PeopleDetectionX) < middleDistance){
                  middleDistance = abs( 180 - PeopleDetectionX );
                  // found_filtered.pop_back();
                  found_filtered.assign(1,r);
                  cout  << PeopleDetectionX << " , " << PeopleDetectionY << endl; 
                }
        }

        for (size_t i = 0; i < found_filtered.size(); i++)
        {
            Rect r = found_filtered[i];
            Point middle( r.x+r.width*0.5 , r.y+r.height*0.5);
            //cv::circle(copiedFrame, middle, 0, cv::Scalar(0,0,255), 8 );
        }
        //cv::circle( copiedFrame , MiddlePoint , 0 , cv::Scalar(0,255,0) , 4);
        middleDistance = 180;
        cv::resize(copiedFrame, copiedFrame , Size(720,480));
      cv::imshow(OPENCV_WINDOW, copiedFrame);
      cv::waitKey(30);
}
//Here is the calcuation of the People detection. Like should i drive forward or turn.
void PeopleDetectorController::CalculatePeopleDetection(){

}


void PeopleDetectorController::SetNode(ros::NodeHandle *n){
	_commandPublisher = n->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	}