#ifndef _AUTONOMOUS_DRIVING_CONTROLLER_
#define _AUTONOMOUS_DRIVING_CONTROLLER_

using namespace std;

class AutonomousDrivingController
{
	public:
		//Constructor
		AutonomousDrivingController(WillyController* Controller, ros::NodeHandle* n);

		void Start();
		double GPSLatA,GPSLongA,GPSLatB,GPSLongB,GPSLatC,GPSLongC,GPSLatD,GPSLongD,GPSCurrentLat,GPSCurrentLong;
		std::vector<double> routeLat;
		std::vector<double> routeLong;
		void getRouteFromParam();

	private:
		WillyController* _controller;
		ros::NodeHandle* nh;
		double distanceBetweenGPS(double lat1d, double lon1d, double lat2d, double lon2d);
		double bearingBetweenGPS(double lat1d, double lon1d, double lat2d, double lon2d);
		double rad2deg(double rad);
		double deg2rad(double deg);
		void driveTowardsPoint(double lat1, double lon1, double lat2, double lon2);
		void evadeObject();
};

#endif
