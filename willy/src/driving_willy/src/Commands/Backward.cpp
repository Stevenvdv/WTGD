#include "../include.h"

using namespace std;

//Author: Michal Nowak
//Execute Function for Backward command
void Backward::Execute()
{
	ROS_INFO("Execute Backward");
	//Get first ticks
	geometry_msgs::Vector3 Ticks = _controller->GetLatestTicks();

	//Set the current X as location
	int CurrentY = Ticks.y;

	//Wait till the encoder gives some feedback
	while(_controller->ReceivedFirstTick == false) {
		Ticks = _controller->GetLatestTicks();
		CurrentY = Ticks.y;
		usleep(50);
	}

	//Calculates the location where he should go.
	int StopAtY = CurrentY - (_meters * 600);
	
	//Wait for one second before we send the command
	ros::Duration(1).sleep();

	//Send the backward command to the arduino
	_controller->SendCommandToArduino(Movement::GetBackwardCommand());

	//Keep driving till he enters the Location where he must end
	while(CurrentY > StopAtY) {
		//Update current Ticks.
		Ticks = _controller->GetLatestTicks();
		CurrentY = Ticks.y;

		if(_controller->CanDriveBackward == false) {
			CurrentY = StopAtY;
			ROS_INFO("STOPPED BY SONAR! Cannot drive backward");
		}

		//Sleep 50 milliseconds and check again.
		usleep(50);	
	}
	
	//Send the stop command to the controller
	_controller->SendCommandToArduino(Movement::GetStopCommand());
}

//Constructor
Backward::Backward(double Meters, WillyController* Controller) {
 	_meters = Meters;
	_controller = Controller;
}

std::string Backward::toString() {
	return " Backward ";
}

