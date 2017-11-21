#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_

#include "../include.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

class Transform
{
  public:
    //constructor
    Transform(ros::NodeHandle *n);

    void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks);

    void TransformData();

  private:

    long _PreviousLeftEncoderCounts;
    long _PreviousRightEncoderCounts;
    ros::Time current_time_encoder, last_time_encoder;
    ros::Time current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;
    double DistancePerCount;

    double x;
    double y;
    double th;

    double vx;
    double vy;
    double vth;
    double deltaLeft;
    double deltaRight;

    ros::Publisher odom_pub;
    ros::Subscriber sub;
};

#endif
