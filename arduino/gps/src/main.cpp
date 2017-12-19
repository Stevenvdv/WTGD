//code for arduino uno r3
//purpuse : reading gps data and sending it with I2C
//made by : Rick Overhorst, credits to owners of the libraries
//disclaimer if things break and blow up, ain't my problem. respect laws blablabla
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//function prototype
void updateInfo();

//variables
int main(int argc, char const *argv[])
{
  //init
  init();
  Serial.begin(9600);
  Serial.println("starting....");
  Wire.begin();
  ss.begin(GPSBaud);
  while(1)
  {
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        updateInfo();

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  }
  return 0;
}

void updateInfo()
{
  char msg[100];
  // Serial.println(gps.location.lat(),7);
  // Serial.println(sizeof(gps.location.lat()));
  char lat[15];
  char lng[15];
  dtostrf(gps.location.lat(),9,7,lat);
  dtostrf(gps.location.lng(),8,7,lng);
  // Serial.println(lat);
  sprintf(msg, "%d,",gps.satellites.value());
  strcat(msg, lat);
  strcat(msg,",");
  strcat(msg, lng);
  Serial.println(msg);
  Wire.beginTransmission(0x01);
  Wire.write(msg);
  Wire.endTransmission();
  Serial.println(msg);
  free(msg);
  // _delay_ms(100);
}
