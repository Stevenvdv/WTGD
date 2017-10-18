#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
double delta_x = 0.0;
double delta_y = 0.0;
double wheel_diameter = 0.34;
double counts_per_revolution = 128;
double encoder_scale_factor = 3.14159265 * wheel_diameter / counts_per_revolution;
double pos_x = 0.0;
double pos_y = 0.0;
double heading = 0;

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
  //get delta encoder ticks
  delta_x = ticks->x - _PreviousLeftEncoderCounts;
  delta_y = ticks->y - _PreviousRightEncoderCounts;

  //calculate movement of robot
  double displacement = (delta_x + delta_y) * encoder_scale_factor / 2;

  //calculate rotation of robot
  double rotation = (delta_x + delta_y) * encoder_scale_factor / wheel_diameter;

  //calculate x and y
  pos_x += displacement * cos(heading + rotation / 2);
  pos_y += displacement * sin(heading + rotation / 2);

  //get direction robot moves
  heading += rotation;

  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
  // last_time_encoder = current_time_encoder;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("wheel_encoder", 100, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Time current_time;

  ros::Rate r(10);
  while(n.ok()){

    current_time = ros::Time::now();

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_x;
    odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = 0;

    // //set the velocity
    // odom.child_frame_id = "base_link";
    // odom.twist.twist.linear.x = 0;
    // odom.twist.twist.linear.y = 0;
    // odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    // last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }
}
