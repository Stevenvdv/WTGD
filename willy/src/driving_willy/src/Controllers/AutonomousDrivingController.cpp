#include "../include.h"
#include <math.h>
#include <cmath>
#define earthRadiusMeters 6371000 //meters

AutonomousDrivingController::AutonomousDrivingController(WillyController* Controller, ros::NodeHandle* n) {
	nh = n;
	_controller = Controller;

	//placeholder gps values.
	GPSLatA = 52.512323;
	GPSLongA = 6.092517;
	printf("%d\n",GPSLatA );
	GPSLatB = 52.512165;
	GPSLongB = 6.092933;

	GPSLatC = 52.512001;
	GPSLongC = 6.092820;

	GPSLatD = 52.512113;
	GPSLongD = 6.092353;

}

bool turningLeft = false;
bool turningRight = false;
bool backward = false;

void AutonomousDrivingController::Start() {
	getRouteFromParam();
	while(true) {
		_controller->CalculateMovingPossibilities();
		if(_controller->CanDriveForward == true)
		{
			_controller->SendCommandToArduino(Movement::GetForwardCommand());
			printf("forward");
		}
		else if(_controller->CanTurnRight == true && turningRight == false)
		{
			_controller->SendCommandToArduino(Movement::GetRightCommand());
			turningRight = true;
		}
		else if(_controller->CanTurnLeft == true && turningLeft == false)
		{
			_controller->SendCommandToArduino(Movement::GetLeftCommand());
			turningLeft = true;
		}
		else if(_controller->CanDriveBackward == true)
		{
			_controller->SendCommandToArduino(Movement::GetBackwardCommand());
			backward = true;
		}
		else {_controller->SendCommandToArduino(Movement::GetStopCommand());}

		if(turningLeft == true)
		{
			if(_controller->CanDriveForward == true)
			{
				ros::Duration(2).sleep();
				_controller->SendCommandToArduino(Movement::GetForwardCommand());
			}
			if(_controller->CanTurnLeft == false)
			{
				_controller->SendCommandToArduino(Movement::GetStopCommand());
			}
			turningLeft = false;
		}

		if(turningRight == true)
		{
			if(_controller->CanDriveForward == true)
			{
				ros::Duration(2).sleep();
				_controller->SendCommandToArduino(Movement::GetForwardCommand());
			}
			if(_controller->CanTurnRight == false)
			{
				_controller->SendCommandToArduino(Movement::GetStopCommand());
			}
			turningRight = false;
		}

		if(backward == true)
		{
			if(_controller->CanTurnLeft == true)
			{
				ros::Duration(3).sleep();
				_controller->SendCommandToArduino(Movement::GetLeftCommand());
				turningLeft = true;
				backward = false;
			}
			if(_controller->CanTurnRight == true)
			{
				ros::Duration(3).sleep();
				_controller->SendCommandToArduino(Movement::GetRightCommand());
				turningRight = true;
				backward = false;
			}
		}

		ros::Duration(1).sleep();
	}
}

//This function converts decimal degrees to radians
double AutonomousDrivingController::deg2rad(double deg) {
  return (deg * M_PI / 180);
}

//This function converts radians to decimal degrees
double AutonomousDrivingController::rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

//This function calculates the distance between two GPS coords in meters.
double AutonomousDrivingController::distanceBetweenGPS(double lat1d, double lon1d, double lat2d, double lon2d) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);
  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);
  return 2.0 * earthRadiusMeters * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double AutonomousDrivingController::bearingBetweenGPS(double lat1d, double lon1d, double lat2d, double lon2d) {
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = cos(lat1r)*sin(lat2r) - sin(lat1r)*cos(lat2r)*cos(lon2r-lon1r);
	v = sin(lon2r-lon1r) * cos(lat2r);
	return rad2deg(atan2(v,u));

}

//this function gets the gps route data stored on the ros parameter server.
void AutonomousDrivingController::getRouteFromParam() {
	if(!nh->getParam("gpsLat",routeLat)) {
		ROS_INFO("Getting gpsLat from parameter server failed!");
	}
	if(!nh->getParam("gpsLong",routeLong)) {
		ROS_INFO("Getting gpsLong from parameter server failed");
	}
}
