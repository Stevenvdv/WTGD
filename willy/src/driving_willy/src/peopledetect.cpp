#include "include.h"

/* Global Variables */


PeopleDetector::PeopleDetector(WillyController* Controller)
    {

        _controller = Controller;
        
        // Subscrive to input video feed video feed
        image_sub_ = nh_.subscribe("/camera/rgb/image_color", 1, &PeopleDetector::rgbCallback, this);
        depth_sub_ = nh_.subscribe("/camera/depth/image_raw", 1, &PeopleDetector::depthCallback, this);

        hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        cv::namedWindow("Image window");
        cvStartWindowThread();
    }
//Deconsturcor
PeopleDetector::~PeopleDetector()
    {
        cv::destroyWindow("Image window");
    }
//Depth image callback
void PeopleDetector::depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //Putting depth map into property
        lastDepthImage = msg;
    }
//Rgb image callback
void PeopleDetector::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    { 
        // Cleaning vector from previous values
        for (unsigned i = 0; i < found.size(); ++i)
        {
            found.clear();
            found_filtered.clear();
        }

        
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::resize(cv_ptr->image, copiedFrame , Size(320,240));
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        // Greyscale image works better for people detection. Dunno why. ¯\_(ツ)_/¯.
        cv::cvtColor(copiedFrame, copiedFrame, cv::COLOR_RGB2GRAY);

        //Histogram of gradients, detecting people.
        hog.detectMultiScale(copiedFrame, found, 0, Size(8,8), Size(32,32), 1.05, 2);

        for(size_t i = 0; i < found.size(); i++)
        { 
            Rect r = found[i];
            // X & Y values of detected person
            double xMidPersonCoord = (r.x + r.width*0.5);
            double yMidPersonCoord = (r.y + r.height*0.5);

            size_t j;
            // Do not add small detections inside a bigger detection.
            for ( j = 0; j < found.size(); j++ ){
                if ( j != i && (r & found[j]) == r ){
                    break;
                }
            }
            if ( j == found.size() )
            {
                //If code detected more than 1 person, check which one is closer to middle
                if( abs(160 - xMidPersonCoord) < middleDistance)
                {
                    //Absolute value of closest person to middle point
                    middleDistance = abs( 160 - xMidPersonCoord );

                    // Put closest person in the first place of vector
                    found_filtered.assign(1,r);
                    
                    //Calcualte degrees to turn
                    turnDistance = ((xMidPersonCoord-160)/10);

                    //Depth of detected person
                    int Depth = ReadDepthData(xMidPersonCoord*2, yMidPersonCoord*2, lastDepthImage);

                    if(Depth != 0){
                        // X,Y,Z values of detected person
                        cout<< "[ " << xMidPersonCoord <<" , " << yMidPersonCoord << " , " << Depth << " ]" << endl;

                        //Calculate forward distance
                        forwardDistance = Depth - 2000;
                       
                        // If person is detected in the area of turning, it triggers turning functions

                        if((turnDistance > -13 && turnDistance < -5) || (turnDistance < 13 && turnDistance > 5) ) 
                        {
                            PeopleDetector::TurnPeopleDetect(turnDistance);
                        }

                        //If detected person is in the 'safe area' robot goes forward
                        else if(-5 < turnDistance && turnDistance < 5 && forwardDistance > 500)
                        {
                            PeopleDetector::ForwardPeopleDetect(forwardDistance);
                        }
                        // If person is in the 'safe area ' robots goes backwards
                        else if(-5 < turnDistance && turnDistance < 5 && forwardDistance < 500)
                        {
                            //Backwards function gets positive vlues to operate
                             backwardDistance = forwardDistance * (-1);
                            PeopleDetector::BackwardPeopleDetect(backwardDistance);
                        }
                        else{
                            ros::Duration(0.3).sleep();
                        }
                    }
                }   
            }
        }

        // Drawing detected person
        for (size_t i = 0; i < found_filtered.size(); i++)
        {
            Rect r = found_filtered[i];
            Point middle( r.x+r.width*0.5 , r.y+r.height*0.5);
            cv::circle(copiedFrame, middle, 0, cv::Scalar(0,0,255), 8 );
        }

        //Middle crosshair 
        Point MiddlePoint(160 , 120);
        cv::circle( copiedFrame , MiddlePoint , 0 , cv::Scalar(0,255,0) , 4);

        //Resetting middle distance for next closest to middle person
        middleDistance = 160;

        //Image resizing for better view
        cv::resize(copiedFrame, copiedFrame , Size(640,480));

        //Image Display
        cv::imshow("Image window", copiedFrame);
        cv::waitKey(30); // ??? 
    }


// Trigger for Turning Function
void PeopleDetector::TurnPeopleDetect(double distance)
    {
        Turn cmdTurn = Turn(distance, _controller);
        ROS_INFO("%f",distance);
        _controller->Execute(cmdTurn);
            
    }


// Trigger for Forward Function
void PeopleDetector::ForwardPeopleDetect(double forwardDistance)
    {
        Forward cmdForward = Forward(forwardDistance/2000, _controller);
        ROS_INFO("FORWAR:::%f",forwardDistance);
        ROS_INFO("___________________");
        _controller->Execute(cmdForward);
            
    }

// Trigger for Backward Function
void PeopleDetector::BackwardPeopleDetect(double backwardDistance)
    {
        Backward cmdBackward = Backward(backwardDistance/2000, _controller);
        ROS_INFO("BACKWARD:::%f",backwardDistance);
        ROS_INFO("___________________");
        _controller->Execute(cmdBackward);
            
    }

//This method Calculates the depth from a depth image. Brace yourself
int PeopleDetector::ReadDepthData(unsigned int width_pos, unsigned int height_pos, sensor_msgs::ImageConstPtr depth_image)
    {
        if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width)) {
            return -1;
        }

        int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));

        if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;

        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) || ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) {
            for (i = 0; i < 4; i++) {
                depth_data.byte_data[i] = depth_image->data[index + i];
            }
            if (depth_data.float_data == depth_data.float_data){
                return int(depth_data.float_data*1000);
            }
            return -1;
        }

        for (i = 0; i < 4; i++) {
          depth_data.byte_data[i] = depth_image->data[3 + index - i];
        }

        if (depth_data.float_data == depth_data.float_data){
          return int(depth_data.float_data*1000);
        }
        return -1;
        }

        int temp_val;

        if (depth_image->is_bigendian){
        temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
        }
        else {
        temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
        }
        // Make sure data is valid (check if NaN)
        if (temp_val == temp_val){
        return temp_val;
        }

        return -1;  // If depth data invalid
    }

//Controller declaration
WillyController controller;

//People Detection main

int main(int argc, char **argv) {
 
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //Set up the subsriber of the wheel encoders to the WheelCallback of the WillyController.
    ros::Subscriber subWheelEncoder = nh.subscribe("wheel_encoder", 100, &WillyController::WheelCallback, &controller);

    //Set up the subscriber for the sonar
    ros::Subscriber subSonar = nh.subscribe("/sonar", 100, &WillyController::SonarCallback, &controller);

    //Gives the node to the controller.
    controller.SetNode(&nh);

    //Set the asynchronised spinner for ros.
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //Start People Detection
    PeopleDetector pd = PeopleDetector(&controller);
    
    ros::spin(); //???
    ros::waitForShutdown(); // ???
 }
