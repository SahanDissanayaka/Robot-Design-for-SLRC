
/*********************************************************************
Original source: http://playground.arduino.cc/Main/I2cScanner

this program will find the I2C address on the I2C device. Just upload the code into your Arduino
and open serial monitor and wait. It will display the I2C address as 0x3C or similar.

 * Please view other Robojax codes and videos at http://robojax.com/learn/arduino
 * if you are sharing this code, you must keep this copyright note.
 * 
*********************************************************************/


#include <Wire.h>


void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial)
    ;  // Leonardo: wait for serial monitor
  Serial.println("I2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  delay(5000);  // wait 5 seconds for next scan
}
