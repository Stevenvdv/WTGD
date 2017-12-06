#include <Arduino.h>
#include <Antonius.h>
Antonius a;
int main(int argc, char const *argv[]) {
  init();
  Serial.begin(9600);
  int counter = 0;
  while(counter < 10)
  {
    // a.sendByte(0x0D,0x00,0x70);
    // Serial.println("************************************************************");
    // a.sendByte(0x0D,0x01,0xA0);
    // Serial.println("************************************************************");
    // a.sendByte(0x0D,0x02,0x00);
    // Serial.println("************************************************************");
    // a.sendByte(0x0D,0x02,0x00);
    // Serial.println("************************************************************");
    // Serial.print("data = ");
    Serial.println(a.requestByte(0x0D,0x00),BIN);
    Serial.println(a.requestByte(0x0D,0x01),BIN);
    counter++;
    Serial.println("------------------------------------------------------------");
    // delay(10);
  }
  return 0;
}
