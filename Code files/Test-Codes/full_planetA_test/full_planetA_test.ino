#include <Wire.h>
#include <VL53L0X.h>
#include <LiquidCrystal_I2C.h>

#define IR1 11
#define IR2 10
#define IR3 9
#define IR4 8
#define IR5 7
#define IR6 6
#define IR7 5
#define IR8 4

#define LMotorA 52
#define LMotorB 50
#define LMotorPWM 2

#define RMotorA 48
#define RMotorB 46
#define RMotorPWM 3

#define MAX_SPEED 240



int IR_val[8] = {0,0,0,0,0,0,0,0};
int pre_IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_weights[8] = {-20,-15,-10,-5,5,10,15,20};

int MotorBasespeed = 90;


int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;



float P,I,D;
float error;
float previousError =0;
float Kp = 2.5;
float Kd = 0.5;
float Ki = 0.01;

void read_IR();
void PID_control();
void mdrive();
void stop();
void turn_left();
void turn_right();


VL53L0X sensor;
#define HIGH_ACCURACY

LiquidCrystal_I2C Lcd(0x27,16,2);


// Planet A 
bool get_colour = false;



void setup() {
  Serial.begin(9600);
  Lcd.begin();
  Lcd.backlight();

  Lcd.setCursor(0,0);
  

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

}

void loop() {
  int distance = 0;//get the distance
  if(distance <= 6){// 6cm near to the stage have to get the colour
    // get the colour
    get_colour = true;
    

  }
  else{
    read_IR();
  if ( IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0){
    if (previousError<0){
      turn_right();
      Serial.println("aa");
    }else{
      turn_left();
      Serial.println("bb");
    }
  }
  else{
    PID_control();
    int ml = 120;
    int mr = 120;

    mdrive(ml+speedAdjust, mr-speedAdjust);
  }

  }
  

  

  int distance = sensor.readRangeSingleMillimeters();
  Serial.print(distance);
  Serial.println();

  Lcd.clear();
  Lcd.print(distance);

  
}

void read_IR(){
  IR_val[0] = !digitalRead(IR1);
  IR_val[1] = !digitalRead(IR2);
  IR_val[2] = !digitalRead(IR3);
  IR_val[3] = !digitalRead(IR4);
  IR_val[4] = !digitalRead(IR5);
  IR_val[5] = !digitalRead(IR6);
  IR_val[6] = !digitalRead(IR7);
  IR_val[7] = !digitalRead(IR8);

   Serial.print(IR_val[0]);
   Serial.print(" ");
   Serial.print(IR_val[1]);
   Serial.print(" ");
   Serial.print(IR_val[2]);
   Serial.print(" ");
   Serial.print(IR_val[3]);
   Serial.print(" ");
   Serial.print(IR_val[4]);
   Serial.print(" ");
   Serial.print(IR_val[5]);
   Serial.print(" ");
   Serial.print(IR_val[6]);
   Serial.print(" ");
   Serial.println(IR_val[7]);
}

void PID_control(){

  error = 0;
  
  for (int i=0; i<8; i++){
    error += IR_weights[i] * IR_val[i];
    }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;

  
}

void mdrive(int ml, int mr){
  if (ml > 0) {
    if (ml > 255) {
      ml = 255;
    }
    digitalWrite(LMotorA, HIGH);
    digitalWrite(LMotorB, LOW);
    analogWrite(LMotorPWM, ml);
  }
  else {
    if (ml < -255) {
      ml = -255;
    }
    digitalWrite(LMotorA, LOW);
    digitalWrite(LMotorB, HIGH);
    analogWrite(LMotorPWM, -1*ml);

  }
  if (mr > 0) {
    if (mr > 255) {
      mr = 255;
    }
    digitalWrite(RMotorA, HIGH);
    digitalWrite(RMotorB, LOW);
    analogWrite(RMotorPWM, mr);
  }
  else {
    if (mr < -255) {
      mr = -255;
    }
    digitalWrite(RMotorA, LOW);
    digitalWrite(RMotorB, HIGH);
    analogWrite(RMotorPWM, -1*mr);
  }
}

void turn_right(){
  mdrive(100,100);
  delay(500);
  read_IR();
  while ( IR_val[3] == 0 || IR_val[4] == 0){
    read_IR();
    mdrive(-90, 90);
    delay(10);
  }
}

void turn_left(){
  mdrive(100,100);
  delay(500);
  read_IR();
  while ( IR_val[3] == 0 || IR_val[4] == 0){
    read_IR();
    mdrive(90, -90);
    delay(10);
  }
}