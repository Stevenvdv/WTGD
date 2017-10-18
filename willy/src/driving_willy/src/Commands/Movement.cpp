#include "../include.h"

using namespace std;

//Author: Henk-Jan Leusink
//Description: This class contains only static methods which returns default commands. 

//Method for returning the forward command.
geometry_msgs::Twist Movement::GetForwardCommand() {
	geometry_msgs::Twist goForwardDriving;
	
	goForwardDriving.linear.x = 0.50;
	goForwardDriving.linear.y = 0;
	goForwardDriving.linear.z = 0;
	goForwardDriving.angular.x = 0;
	goForwardDriving.angular.y = 0;
	goForwardDriving.angular.z = 0;

	return goForwardDriving;
}

//Method for returning the backward command.
geometry_msgs::Twist Movement::GetBackwardCommand() {
	geometry_msgs::Twist goBackwardDriving;
	
	goBackwardDriving.linear.x = -0.32;
	goBackwardDriving.linear.y = 0;
	goBackwardDriving.linear.z = 0;
	goBackwardDriving.angular.x = 0;
	goBackwardDriving.angular.y = 0;
	goBackwardDriving.angular.z = 0;

	return goBackwardDriving;
}

//Method for returning the stop command.
geometry_msgs::Twist Movement::GetStopCommand() {
	geometry_msgs::Twist stopDriving;
	
	stopDriving.linear.x = 0;
	stopDriving.linear.y = 0;
	stopDriving.linear.z = 0;
	stopDriving.angular.x = 0;
	stopDriving.angular.y = 0;
	stopDriving.angular.z = 0;

	return stopDriving;
}


//Method for returning the left command.
geometry_msgs::Twist Movement::GetLeftCommand() {
	geometry_msgs::Twist leftDriving;
	
	leftDriving.linear.x = 0;
	leftDriving.linear.y = 0;
	leftDriving.linear.z = 0;
	leftDriving.angular.x = 0;
	leftDriving.angular.y = 0;
	leftDriving.angular.z = 0.25;

	return leftDriving;
}

//Method for returning the right command.
geometry_msgs::Twist Movement::GetRightCommand() {
	geometry_msgs::Twist rightDriving;
	
	rightDriving.linear.x = 0;
	rightDriving.linear.y = 0;
	rightDriving.linear.z = 0;
	rightDriving.angular.x = 0;
	rightDriving.angular.y = 0;
	rightDriving.angular.z = -0.25;

	return rightDriving;
}
