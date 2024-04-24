#include <NewPing.h>
#define USL1_Trig 24
#define USL1_Echo 14

#define USF1_Trig 26
#define USF1_Echo 19

#define UST_Trig 22
#define UST_Echo 18

#define US_down_Trig 11
#define US_down_Echo 10

#define max_distance 100

NewPing SonarL1(USL1_Trig, USL1_Echo, max_distance);
NewPing SonarF1(USF1_Trig, USF1_Echo, max_distance);
NewPing SonarT(UST_Trig, UST_Echo, max_distance);
NewPing SonarD(US_down_Trig, US_down_Echo, max_distance);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
 read_US();

}

void read_US() {
  // Serial.print("1 ");
  // Serial.println(SonarL1.ping_cm());
  // Serial.print("2 ");
  // Serial.println(SonarF1.ping_cm());
  // Serial.print("3 ");
  // Serial.println(SonarT.ping_cm());
  // Serial.print("4");
  Serial.println(SonarD.ping_cm());
  
  delay(50);

}
