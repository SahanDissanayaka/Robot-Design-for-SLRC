#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;// center
VL53L0X sensor2;// wall below
VL53L0X sensor3;// wall middle
VL53L0X sensor4;// wall up
VL53L0X sensor5;


void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Serial.println("HHHHHH");

  TCA9548A(1);

  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  sensor1.startContinuous();

  TCA9548A(2);

  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor2.startContinuous();

  TCA9548A(4);

  sensor3.setTimeout(500);
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor3.startContinuous();

  TCA9548A(3);

  sensor4.setTimeout(500);
  if (!sensor4.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor4.startContinuous();

  TCA9548A(5);

  sensor5.setTimeout(500);
  if (!sensor5.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor5.startContinuous();

}

void loop() {
  TCA9548A(1);
  Serial.print("Sen1 ");
  Serial.println(sensor1.readRangeContinuousMillimeters()/10);

  TCA9548A(2);
  Serial.print("Sen2 ");
  int dis = sensor2.readRangeContinuousMillimeters()/10;
  Serial.println(dis);

  TCA9548A(4);
  Serial.print("Sen3 ");
  Serial.println(sensor3.readRangeContinuousMillimeters()/10);

  TCA9548A(3);
  Serial.print("Sen4 ");
  Serial.println(sensor4.readRangeContinuousMillimeters()/10);

  TCA9548A(5);
  Serial.print("Sen5 ");
  Serial.println(sensor5.readRangeContinuousMillimeters()/10);

  delay(100);

}
