#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

class Odometry {
  public: 
    long _PreviousLeftEncoderCounts;
    long _PreviousRightEncoderCounts;

    float x; 
    float y;

    float WheelRadius; 
    int TicksPerRevolution; 

    int LastTicksLeft;
    int LastTicksRight;

    float pi;
    
    int prev_x;
    int prev_y;
    int prev_theta;

    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    Odometry(ros::NodeHandle *n) 
    {
      
      odom_pub = n->advertise<nav_msgs::Odometry>("odom", 200);   
      
      WheelRadius = 0.34;
      TicksPerRevolution = 128;

      LastTicksLeft = 0; 
      LastTicksRight = 0;

      pi = 3.14; 

    }

    void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
    {
      // R = self.robot_wheel_radius

      // N = float( self.wheel_encoder_ticks_per_revolution )
      
      // # read the wheel encoder values
      // ticks_left, ticks_right = self.robot.read_wheel_encoders()

      int ticks_left = ticks->x; 
      int ticks_right = ticks->y;

      int d_ticks_left = ticks_left - LastTicksLeft;
      int d_ticks_right = ticks_right - LastTicksRight;

      LastTicksLeft = ticks_left;
      LastTicksRight = ticks_right;

      // # get the difference in ticks since the last iteration
      // d_ticks_left = ticks_left - self.prev_ticks_left
      // d_ticks_right = ticks_right - self.prev_ticks_right
      
      // # estimate the wheel movements
      int d_left_wheel = 2 * pi * WheelRadius * ( d_ticks_left / TicksPerRevolution );
      int d_right_wheel = 2 * pi * WheelRadius * ( d_ticks_right / TicksPerRevolution );
      int d_center = 0.5 * ( d_left_wheel + d_right_wheel );
      
      // # calculate the new pose
      // prev_x, prev_y, prev_theta = self.estimated_pose.scalar_unpack()


      int new_x = prev_x + ( d_center * cos( prev_theta ) );
      int new_y = prev_y + ( d_center * sin( prev_theta ) );
      int new_theta = prev_theta + ( ( d_right_wheel - d_left_wheel ) / 0.55 );
      
      // # update the pose estimate with the new values
      // self.estimated_pose.scalar_update( new_x, new_y, new_theta )

      prev_x = new_x;
      prev_y = new_y;
      prev_theta = new_theta;

      // # save the current tick count for the next iteration
      // self.prev_ticks_left = ticks_left
      // self.prev_ticks_right = ticks_right

      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = new_x;
      odom.pose.pose.position.y = new_y;
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
