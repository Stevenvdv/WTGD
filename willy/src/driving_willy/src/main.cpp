#include "include.h"

#include "Includes/PUGIXML/pugixml.hpp"
#include "Includes/PUGIXML/pugixml.cpp"
#include "Includes/PUGIXML/pugiconfig.hpp"

//Set the controller as a global variable.
WillyController controller;

// tag::impl[]
struct ConfigIterator: pugi::xml_tree_walker
{
    virtual bool for_each(pugi::xml_node& node)
    {
    	string type = node.attribute("type").value();

		if(type == "TurnLeft") {
			SonarCheck check;

			check.SonarID = node.attribute("SonarID").as_int();
			check.Value = node.attribute("Value").as_int();

			controller.ChecksTurnLeft[controller.ChecksTurnLeftElements] = check;
			controller.ChecksTurnLeftElements++;
		}

		if(type == "TurnRight") {
			SonarCheck check;

			check.SonarID = node.attribute("SonarID").as_int();
			check.Value = node.attribute("Value").as_int();

			controller.ChecksTurnRight[controller.ChecksTurnRightElements] = check;
			controller.ChecksTurnRightElements++;
		}

		if(type == "DriveForward") {
			SonarCheck check;

			check.SonarID = node.attribute("SonarID").as_int();
			check.Value = node.attribute("Value").as_int();

			controller.ChecksDriveForward[controller.ChecksDriveForwardElements] = check;
			controller.ChecksDriveForwardElements++;
		}


		if(type == "DriveBackward") {
			SonarCheck check;

			check.SonarID = node.attribute("SonarID").as_int();
			check.Value = node.attribute("Value").as_int();

			controller.ChecksDriveBackward[controller.ChecksDriveBackwardElements] = check;
			controller.ChecksDriveBackwardElements++;
		}



        return true;
    }
};


int main(int argc, char** argv)
{

  pugi::xml_document ConfigFile;
  ConfigFile.load_file("/home/willy/Documents/driving-willy/WTGD/willy/src/driving_willy/src/tree.xml");

  ConfigIterator walker;
  ConfigFile.traverse(walker);

	//Ros initation.
  ros::init(argc, argv, "DrivingWilly");
	ros::NodeHandle n;

  Transform transformOdometryData  = Transform(&n);

	//Set up the subsriber of the wheel encoders to the WheelCallback of the WillyController.
  ros::Subscriber subWheelEncoder = n.subscribe("/wheel_encoder", 100, &Transform::WheelCallback, &transformOdometryData);
  ros::Subscriber subWheelEncoderWillyController = n.subscribe("/wheel_encoder", 100, &WillyController::WheelCallback, &controller);

  //Set up the subscriber for the sonar
  ros::Subscriber subSonar = n.subscribe("/sonar", 100, &WillyController::SonarCallback, &controller);
  transformOdometryData.TransformData();
  //Gives the node to the controller.
  controller.SetNode(&n);

	//Set the asynchronised spinner for ros.
	ros::AsyncSpinner spinner(4);
	spinner.start();

	AutonomousDrivingController autonomouseDriving = AutonomousDrivingController(&controller);
	autonomouseDriving.Start();

	// Wait
	ros::waitForShutdown();
	return 0;
}
