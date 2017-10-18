#include "../include.h"

//Author: Henk-Jan Leusink
//Method which executes the Turn Command.
void Turn::Execute()
{
	ROS_INFO("Execute Turn");
	//Get the first ticks. 
	geometry_msgs::Vector3 Ticks;

	//Wait till the encoder gives some feedback.
	while(!_controller->ReceivedFirstTick) {
		usleep(50);
	}

	//Get ticks
	Ticks = _controller->GetLatestTicks();
	int CurrentDistanceLeftMinusRight = 0;
	int StopAtDistance = 0;

	//Right
	if(_degrees > 0) {
		ROS_INFO("Turn right");
		CurrentDistanceLeftMinusRight = (Ticks.x - Ticks.y);
		StopAtDistance = CurrentDistanceLeftMinusRight + (4.6 * _degrees);
	} else {
		ROS_INFO("Turn left");
		//Left	
		CurrentDistanceLeftMinusRight = (Ticks.y - Ticks.x);
		StopAtDistance = CurrentDistanceLeftMinusRight - (4.6 * _degrees);	
	}

	//Send the stop command to the controller
	if(_degrees > 0) {
		_controller->SendCommandToArduino(Movement::GetRightCommand());
	} else {
		_controller->SendCommandToArduino(Movement::GetLeftCommand());
	}


	while(CurrentDistanceLeftMinusRight < StopAtDistance) {
		//Update current Ticks.
		Ticks = _controller->GetLatestTicks();

		//Right
		if(_degrees > 0) 
		{
			CurrentDistanceLeftMinusRight = (Ticks.x - Ticks.y);
			if(!_controller->CanTurnRight) {
				CurrentDistanceLeftMinusRight = StopAtDistance;
				ROS_INFO("STOPPED BY SONAR! Cannot move to the right");
			}
		} 
		//Left
		else 
		{
			CurrentDistanceLeftMinusRight = (Ticks.y - Ticks.x);
			if(!_controller->CanTurnLeft) {
				CurrentDistanceLeftMinusRight = StopAtDistance;
				ROS_INFO("STOPPED BY SONAR! Cannot move to the left");
			}
		}

		//Sleep 50 milliseconds and check again.
		usleep(50);		
	}

	//Send the stop command to the controller
	_controller->SendCommandToArduino(Movement::GetStopCommand());
}

//Constructor. Degrees can be from -180 till 180
Turn::Turn(double Degrees, WillyController* Controller) 
{
	if(Degrees < -180) {
		Degrees = -180;
	}

	if(Degrees > 180) {
		Degrees = 180;
	}

	_degrees = Degrees;
	_controller = Controller;
}

//Default ToString method. 
std::string Turn::toString() {
	return "Turn";
}