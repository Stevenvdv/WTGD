#ifndef _AUTONOMOUS_DRIVING_CONTROLLER_
#define _AUTONOMOUS_DRIVING_CONTROLLER_

using namespace std;

class AutonomousDrivingController
{
	public:
		//Constructor
		AutonomousDrivingController(WillyController* Controller);
		
		void Start();
		
	private: 
		WillyController* _controller;
};

#endif
