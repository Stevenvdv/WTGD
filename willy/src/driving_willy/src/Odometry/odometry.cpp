#include "../include.h"

//constructor


void odometry::WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
  lastMeasureX = (int)ticks->x;
  lastMeasureY = (int)ticks->y;

  totalDistanceX += lastMeasureX;
  totalDistanceY += lastMeasureY;
}

int odometry::getTotalDistanceX()
{
  return totalDistanceX;
}

int odometry::getTotalDistanceY()
{
  return totalDistanceY;
}
