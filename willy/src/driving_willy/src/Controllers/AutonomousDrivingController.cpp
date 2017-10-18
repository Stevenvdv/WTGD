#include "../include.h"

//Author: Henk-Jan Leusink
//Description: 	This class is the controller of Willy. 
// 				It handles feedback which is send from Willy and send controls to Willy for driving.
//Default Constructor
AutonomousDrivingController::AutonomousDrivingController(WillyController* Controller) {
	_controller = Controller;
}

void AutonomousDrivingController::Start() {
	//Declare forward object
	ForwardUntilHit commandForward = ForwardUntilHit(_controller);

	//Declare default backward command for later use
	Backward commandBackward = Backward(0.25, _controller);

	//Start loop
	while(true) {
		//Go forward when it is possible.
		if(_controller->CanDriveForward) {
			_controller->Execute(commandForward);
		}

		//Stand still for 2 seconds because maybe there's a person walking before.
		ros::Duration(2).sleep();
		
		//Check again if you can go forward.
		if(_controller->CanDriveForward) {
			_controller->Execute(commandForward);
		}

		//Go backward when you can't turn left and right OR random
		if(!_controller->CanTurnLeft && !_controller->CanTurnRight || ((1 + ( std::rand() % ( 10 - 1 + 1 ) )) > 5)) {
			_controller->Execute(commandBackward);
		}

		//Here happens the logic of the turning degrees. Should you go left or right and how many degrees.
		int RandomTurningDegrees = 10;

		if(_controller->CanTurnLeft && _controller->CanTurnRight) {
			RandomTurningDegrees = ((float)(rand()%(180-0 + 1) + 0) - 90);
		} else if(!_controller->CanTurnLeft && _controller->CanTurnRight) {
			RandomTurningDegrees = (10 + ( std::rand() % ( 100 - 10 + 1 ) ));
		} else if(_controller->CanTurnLeft && !_controller->CanTurnRight) {
			RandomTurningDegrees = (10 + ( std::rand() % ( 100 - 10 + 1 ) )) - 110;
		}

		//Create and execute the turning command
		Turn commandTurn = Turn(RandomTurningDegrees, _controller);
		_controller->Execute(commandTurn);

		//Wait for a second and then repeat the steps again.
		ros::Duration(1).sleep();
	}
}
