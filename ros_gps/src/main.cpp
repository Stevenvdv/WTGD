//code for arduino uno r3
//purpuse : reading gps data and sending it with I2C to the master (connected to pc)
//made by : Rick Overhorst, credits to owners of the libraries
//disclaimer if things break and blow up, ain't my problem. respect laws blablabla
#include <Wire.h>
#include <ros.h> 
#include <std_msgs/String.h>
#include <Arduino.h>
ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher gps_pub("gps", &str_msg);

char msg[100];

void receiveEvent(int bytes);
int update = 0;
uint16_t sat,lat,lng;
int main(int argc, char const *argv[])
{
  init();
  Wire.begin(0x01);
  Wire.onReceive(receiveEvent);
  nh.initNode();
  nh.advertise(gps_pub);

  while (1)
  {
    if(update)
    {
      // Serial.println("stff");
      // Serial.println(msg);
      str_msg.data = msg;
      gps_pub.publish( &str_msg );
      nh.spinOnce();
      update = 0;
    }
      delay(100);
  }
  return 0;
}

void receiveEvent(int bytes)
{
    Wire.readBytes(msg, bytes);
    update = 1;
}
