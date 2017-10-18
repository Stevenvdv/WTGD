#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

class Odometry {
  public: 
    float WHEEL_BASE;
    float LEFT_CLICKS_PER_INCH;
    float RIGHT_CLICKS_PER_INCH;

    float TWOPI;

    /* ----------------------------------------------------------------------- */
    /* odometer maintains these global variables: */

    float theta;                    /* bot heading */
    float X_pos;                    /* bot X position in inches */
    float Y_pos;                    /* bot Y position in inches */

    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    int lsamp, rsamp, L_ticks, R_ticks, last_left, last_right;
    float left_inches, right_inches, inches;
    Odometry(ros::NodeHandle *n) 
    {
      odom_pub = n->advertise<nav_msgs::Odometry>("odom", 200);

      WHEEL_BASE = 0.55;
      LEFT_CLICKS_PER_INCH = 1;
      RIGHT_CLICKS_PER_INCH = 1;

      TWOPI = 6.28;
    }

    void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
    {

      int left_distance = ticks->x; 
      int right_distance = ticks->y;
      /* sample the left and right encoder counts as close together */
      /* in time as possible */
      lsamp = left_distance;
      rsamp = right_distance; 

      /* determine how many ticks since our last sampling? */
      L_ticks = lsamp - last_left;  
      R_ticks = rsamp - last_right;

      /* and update last sampling for next time */
      last_left = lsamp; 
      last_right = rsamp; 

      /* convert longs to floats and ticks to inches */
      left_inches = (float)L_ticks/LEFT_CLICKS_PER_INCH;
      right_inches = (float)R_ticks/RIGHT_CLICKS_PER_INCH;

      /* calculate distance we have traveled since last sampling */
      inches = (left_inches + right_inches) / 2.0;

      /* accumulate total rotation around our center */
      theta += (left_inches - right_inches) / WHEEL_BASE;

      /* and clip the rotation to plus or minus 360 degrees */
      theta -= (float)((int)(theta/TWOPI))*TWOPI;

      /* now calculate and accumulate our position in inches */
      Y_pos += inches * cos(theta); 
      X_pos += inches * sin(theta); 

      nav_msgs::Odometry odom;

      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = X_pos;
      odom.pose.pose.position.y = Y_pos;
      odom.pose.pose.position.z = 0.0;
      // odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = 0;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = 0;

      //publish the message
      odom_pub.publish(odom);
    }


  private:
    ros::Publisher _odomPublisher;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  Odometry odom = Odometry(&n);

  ros::Subscriber subWheelEncoder = n.subscribe("wheel_encoder", 100, &Odometry::WheelCallback, &odom);

  //Set the asynchronised spinner for ros.
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();
}
