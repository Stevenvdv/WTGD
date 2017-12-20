#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "../include.h"

using namespace std;

class odometry
{
  public:
    void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks);
    int getTotalDistanceY();
    int getTotalDistanceX();

  private:
    int lastMeasureX;
    int lastMeasureY;
    int totalDistanceX;
    int totalDistanceY;
    int turningDegrees;
};



#endif
