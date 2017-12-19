#include "../include.h"
#include <iostream>
#include <sstream>

//Author: Henk-Jan Leusink
//Description: 	This class is the controller of Willy.
// 				It handles feedback which is send from Willy and send controls to Willy for driving.
//Default Constructor



WillyController::WillyController() {
	ReceivedFirstTick = false;
	CanDriveForward = true;
	CanDriveBackward = true;
	CanTurnLeft = true;
	CanTurnRight = true;


	////
	//// |8-------7--------6|
	//// |                  |
	//// |                  |
	//// |9                5|
	//// |                  |
	//// |                  |
	//// |                  |
	//// |                  |
	//// |                  |
	//// |0                4|
	//// |                  |
	//// |                  |
	//// |1-------2--------3|
	////

	//Sonar array data
	SonarData[0].Degrees = 270;
	SonarData[0].Value = 0;

	SonarData[1].Degrees = 220;
	SonarData[1].Value = 0;

	SonarData[2].Degrees = 180;
	SonarData[2].Value = 0;

	SonarData[3].Degrees = 140;
	SonarData[3].Value = 0;

	SonarData[4].Degrees = 90;
	SonarData[4].Value = 0;

	SonarData[5].Degrees = 90;
	SonarData[5].Value = 0;

	SonarData[6].Degrees = 45;
	SonarData[6].Value = 0;

	SonarData[7].Degrees = 0;
	SonarData[7].Value = 0;

	SonarData[8].Degrees = 315;
	SonarData[8].Value = 0;

	SonarData[9].Degrees = 270;
	SonarData[9].Value = 0;
}

//This method executes the command which extend the ICommand.
void WillyController::Execute(ICommand& command) {
	command.Execute();
}

//This method gets fired wen there's a new message from the sonar system.
void WillyController::SonarCallback(const sensor_msgs::LaserEcho& sonar) {
	for(int i = 0; i < 10; i++){
		SonarData[i].Value = sonar.echoes[i];
	}
	CalculateMovingPossibilities();
}

int WillyController::GetSonarValueByDegrees(int Degrees) {
	for(int i = 0; i < sizeof(SonarData) / sizeof(SonarData[0]); i++){
		if(SonarData[i].Degrees == Degrees) {
			return SonarData[i].Value;
		}
	}
	return 0;
}

//Gets fired when a new wheel_encoder topic is send.
void WillyController::WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks) {
	if(!ReceivedFirstTick) {
		ReceivedFirstTick = true;
	}

	_ticks.x = ticks->x;
	_ticks.y = ticks->y;
}

void WillyController::GpsCallback(const std_msgs::String::ConstPtr& msg)
{
	printf("gps msg = %s\n", msg->data.c_str());
	std::string input = msg->data.c_str();
	std::istringstream ss(input);
	std::string token;
	int counter = 0;
	while(std::getline(ss, token, ','))
	{
		printf("%s\n", (char*)token.c_str());
		++counter;
	}
	// msg->data.c_str().substr (0,5);
}

double WillyController::getLng()
{

}

double WillyController::getLat()
{

}

int WillyController::getSat()
{

}

void WillyController::setLng(double lng)
{

}

void WillyController::setLat(double lat)
{

}
void WillyController::setSat(int sat)
{

}

//This method receives the ROS NodeHandle and creates a new publisher
void WillyController::SetNode(ros::NodeHandle *n) {
	_commandPublisher = n->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	_movingPossibilitiesPublisher = n->advertise<std_msgs::Int32MultiArray>("/possible_directions", 100);
}

void WillyController::CalculateMovingPossibilities() {
	CanDriveForward = true;
	CanDriveBackward = true;
	CanTurnLeft = true;
	CanTurnRight = true;

	for(int i = 0; i < sizeof(ChecksTurnLeft) / sizeof(ChecksTurnLeft[0]); i++){
		if(ChecksTurnLeft[i].Value != 0 && ChecksTurnLeft[i].Value > SonarData[ChecksTurnLeft[i].SonarID].Value) {
			CanTurnLeft = false;
			printf("Left:%d > %d\n",ChecksTurnLeft[i].Value, SonarData[ChecksTurnLeft[i].SonarID].Value);
			break;
		}
	}

	for(int i = 0; i < sizeof(ChecksTurnRight) / sizeof(ChecksTurnRight[0]); i++){
		if(ChecksTurnRight[i].Value != 0 && ChecksTurnRight[i].Value > SonarData[ChecksTurnRight[i].SonarID].Value) {
			CanTurnRight = false;
			printf("Right:%d > %d\n",ChecksTurnRight[i].Value, SonarData[ChecksTurnRight[i].SonarID].Value);
			break;
		}
	}

	for(int i = 0; i < sizeof(ChecksDriveForward) / sizeof(ChecksDriveForward[0]); i++){
		if(ChecksDriveForward[i].Value != 0 && ChecksDriveForward[i].Value > SonarData[ChecksDriveForward[i].SonarID].Value) {
			CanDriveForward = false;
			printf("Forward:%d > %d\n",ChecksDriveForward[i].Value, SonarData[ChecksDriveForward[i].SonarID].Value);
			break;
		}
	}

	for(int i = 0; i < sizeof(ChecksDriveBackward) / sizeof(ChecksDriveBackward[0]); i++){
		if(ChecksDriveBackward[i].Value != 0 && ChecksDriveBackward[i].Value > SonarData[ChecksDriveBackward[i].SonarID].Value) {
			CanDriveBackward = false;
			printf("Backward:%d > %d\n",ChecksDriveBackward[i].Value, SonarData[ChecksDriveBackward[i].SonarID].Value);
			break;
		}
	}

	std_msgs::Int32MultiArray array;

	array.data.clear();
	array.data.push_back(CanDriveForward);
	array.data.push_back(CanTurnLeft);
	array.data.push_back(CanDriveBackward);
	array.data.push_back(CanTurnRight);

	//Publish array
	_movingPossibilitiesPublisher.publish(array);
	ROS_INFO("-----------------------------------------------");
}

//This method sends the msg to the arduino. It can be controlled from the commands.
void WillyController::SendCommandToArduino(geometry_msgs::Twist msg) {
	_commandPublisher.publish(msg);
}

//This method returns the last feedback which is received by the arduino.
geometry_msgs::Vector3 WillyController::GetLatestTicks() {
	return _ticks;
}
