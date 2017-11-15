#include "../include.h"


void Transform::WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{

  current_time_encoder = ros::Time::now();

  deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
  deltaRight = ticks->y - _PreviousRightEncoderCounts;

  vx = deltaLeft * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();
  vy = deltaRight * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();


  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
  last_time_encoder = current_time_encoder;
}

Transform::Transform(ros::NodeHandle *n)
{
  odom_pub = n->advertise<nav_msgs::Odometry>("/odom", 50);

  last_time = ros::Time::now();

  DistancePerCount = (3.14159265 * 0.34) / 2763;
  _PreviousRightEncoderCounts = 0;
  _PreviousLeftEncoderCounts = 0;
}

void Transform::TransformData()
{
  while(true){

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ros::Duration(1).sleep();
  }
}
