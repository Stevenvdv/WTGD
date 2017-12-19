#include <ros.h>                                                      // include Ros Librarie
#include <geometry_msgs/Twist.h>                                      // include Ros Message type
#include <geometry_msgs/Vector3.h>                                    //
#include <std_msgs/String.h>
#include <Encoder.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>                                              // include Encoder Librarie

ros::NodeHandle nh;

#define LEncoderA 21                                                  // Defining Encoder Pins
#define LEncoderB 20
#define REncoderA 2
#define REncoderB 3

int irun;
int iturn;
unsigned char data[255];
double wheelCircum = 110;
double fullRotation = 500;
int lastTicksX=0;
int lastTicksY=0;
double distanceSingleTick = wheelCircum / fullRotation;
int odomCounter = 0;

void messageCb( const geometry_msgs::Twist& twistMsg)                 // subscriber setup
{                                                                     //
  irun = twistMsg.linear.x * 100;                                     //
  iturn = twistMsg.angular.z * 100;                                   //
}                                                                     //
                                                                      //
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);    // subscriber setup

geometry_msgs::Vector3 ticks;
ros::Publisher encoder_ticks_pub("wheel_encoder", &ticks);            // Publish the encoder ticks

Encoder LEncoder(LEncoderA, LEncoderB);
Encoder REncoder(REncoderA, REncoderB);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_ticks_pub);
  //Serial.begin(9600);
  Serial1.begin(19200, SERIAL_8E1);
}

void loop()
{
  int drive = irun;                                                       //Set value from ros to motorcontroller
  int turn = iturn;                                                       //Set value from ros to motorcontroller

  if(drive > 100) {
    drive = 100;
  }

  if(drive < -100) {
    drive = -100;
  }

  if(turn > 100) {
    turn = 100;
  }
  if(turn < -100) {
    turn = -100;
  }

  data[0]=0x6A;                                              //-Datagram always start with 0x6A
  data[1]=drive;                                             //-Drive +-100
  data[2]=turn;                                              //-Turn +-100
  data[3]=0;                                                 //-Driv mode
  data[4]=0x0c;                                              //-Drive mode: 0x0c=fastest
  data[5]=0xff-(data[0]+data[1]+data[2]+data[3]+data[4]);    //-Checksum

  for(unsigned char i=0;i<6;i++)
  {
    Serial1.write(data[i]);                                  // Writing the data to the motorcontroller
  }


  //--Odometry-- Do not edit unless you want to change how Odometry works.
  int currentTicksX = (int)LEncoder.read();
  int currentTicksY = (int)REncoder.read();

  int ticksTraveledX = 0;
  bool negativeX = false;
  int ticksTraveledY = 0;
  bool negativeY = false;

  odomCounter++;
  if(odomCounter == 20)
  {
    if(currentTicksX != lastTicksX && currentTicksY != lastTicksY)  //Only execute odometry if data has changed.
    {
      if(currentTicksX > lastTicksX)                            //Check how many wheel encoder ticks have been made between now and the last time
      {                                                         //we checked for the left wheel encoder.
        ticksTraveledX = currentTicksX - lastTicksX;
      }else if(currentTicksX < lastTicksX)
      {
        ticksTraveledX = lastTicksX - currentTicksX;
        negativeX = true;                                       //check wether we have gone backwards or forwards.
      }

      if(currentTicksY > lastTicksY)                            //Check how many wheel encoder ticks have been made between now and the last time
      {                                                         //we checked for the right wheel encoder.
        ticksTraveledY = currentTicksY - lastTicksY;
      }else if(currentTicksY < lastTicksY)
      {
        ticksTraveledY = lastTicksY - currentTicksY;
        negativeY = true;                                       //check wether we have gone backwards or forwards.
      }

      if(negativeX == true)                                     //Calculate the distance travelled in CM for the left wheel encoder.
      {
        ticks.x = -1 * (ticksTraveledX * distanceSingleTick);
      }else
      {
        ticks.x = ticksTraveledX * distanceSingleTick;
      }

      if(negativeY == true)                                     //Calculate the distance travelled in CM for the right wheel encoder.
      {
        ticks.y = -1 * (ticksTraveledY * distanceSingleTick);
      }else
      {
        ticks.y = ticksTraveledY * distanceSingleTick;
      }
        lastTicksX = currentTicksX;
        lastTicksY = currentTicksY;

        encoder_ticks_pub.publish(&ticks);                      //Publish the ticks in CM's on the wheel_encoder topic.
      }
      odomCounter = 0;
  }

    nh.spinOnce();

    delay(20);

}
