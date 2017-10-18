#include "../include.h"

using namespace std;

//Author: Henk-Jan Leusink
//Method which Executes the ForwardUntilHit method. 
//This method let the robot drive until he hits something. 
void ForwardUntilHit::Execute()
{
	usleep(100);

	while(_controller->SonarData[1].Value == 0) {
		ROS_INFO("	-> Wait for sonar input <- ");
		ros::Duration(1).sleep();
	}

	_controller->SendCommandToArduino(Movement::GetForwardCommand());

	usleep(100);

	//Check continuously if you can drive forward.
	while(_controller->CanDriveForward)
	{	
		ros::Duration(0.1).sleep();
	}
	
	_controller->SendCommandToArduino(Movement::GetStopCommand());
}

//Constructor
ForwardUntilHit::ForwardUntilHit(WillyController* Controller) {
	_controller = Controller;
}

std::string ForwardUntilHit::toString() {
	return " ForwardUntilHit";
}

