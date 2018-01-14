//code for arduino 0 / ROS
//purpuse : reading gps data and sending it with I2C to the master (connected to pc)
//made by : Rick Overhorst, credits to owners of the libraries
//disclaimer if things break and blow up, ain't my problem. respect laws blablabla
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Float32.h"
#include <Arduino.h>
#include <HMC5883L.h>

HMC5883L compass;
ros::NodeHandle  nh;

std_msgs::String str_msg;
std_msgs::Float32 degrees;
ros::Publisher gps_pub("gps", &str_msg);
ros::Publisher compass_pub("compass", &degrees);

char msg[100];
void receiveEvent(int bytes);
int update = 0;
uint16_t sat,lat,lng;

int main(int argc, char const *argv[])
{
  //init
  init();
  Wire.begin(0x01);
  Wire.onReceive(receiveEvent);
  nh.initNode();
  nh.advertise(gps_pub);
  nh.advertise(compass_pub);

  // Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    // Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0);
  // checkSettings();

  while (1)
  {
    if(update)
    {
      // Serial.println("stff");
      // Serial.println(msg);
      Vector norm = compass.readNormalize();
      float heading = atan2(norm.YAxis, norm.XAxis);
      float declinationAngle = (1.0 + (39.0 / 60.0)) / (180 / M_PI);
      heading += declinationAngle;

      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0)
      {
        heading += 2 * PI;
      }

      if (heading > 2 * PI)
      {
        heading -= 2 * PI;
      }

      // Convert to degrees
      float headingDegrees = heading * 180/M_PI;
      str_msg.data = msg;
      degrees.data = headingDegrees;
      gps_pub.publish( &str_msg );
      compass_pub.publish(&degrees);

      nh.spinOnce();
      update = 0;
    }
      delay(500);
  }
  return 0;
}

void receiveEvent(int bytes)
{
    Wire.readBytes(msg, bytes);
    update = 1;
}
