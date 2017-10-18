#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

class Movement
{
	public:
		
		//Method which returns the forward command.
		static geometry_msgs::Twist GetForwardCommand();

		//Method which returns the backward command.
		static geometry_msgs::Twist GetBackwardCommand();

		//Method which returns the stop command.
		static geometry_msgs::Twist GetStopCommand();

		//Method which returns the left command
		static geometry_msgs::Twist GetLeftCommand();

		//Method which returns the right command
		static geometry_msgs::Twist GetRightCommand();
};

#endif