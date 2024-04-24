#include <NewPing.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

#define IR1 45
#define IR2 43
#define IR3 41
#define IR4 39
#define IR5 37
#define IR6 35
#define IR7 33
#define IR8 31

#define LMotorA 51
#define LMotorB 53
#define LMotorPWM 2

#define RMotorA 49
#define RMotorB 47
#define RMotorPWM 3

#define USL1_Trig 24
#define USL1_Echo 14

#define USTD_Trig 22
#define USTD_Echo 18

#define USF1_Trig 26
#define USF1_Echo 19

#define UST_Trig 11
#define UST_Echo 10

int max_distance =100;

NewPing SonarL1(USL1_Trig, USL1_Echo, max_distance);
NewPing SonarTD(USTD_Trig, USTD_Echo, max_distance);
NewPing SonarF1(USF1_Trig, USF1_Echo, max_distance);
NewPing SonarT(UST_Trig, UST_Echo, max_distance);

int IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int pre_IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int IR_weights[8] = { -20, -15, -10, -5, 5, 10, 15, 20 };

int MotorBasespeed = 90;


int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;



float P, I, D;
float error;
float previousError = 0;
float Kp = 2.5;
float Kd = 0.5;
float Ki = 0.01;

void read_IR();
void PID_control();
void mdrive();
void stop();
void turn_left();
void turn_right();
bool check_for_walls();
bool wall_follow_finished = false;
void read_US();

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void turn_angle(float target_angle);

int Us_val[4] = { 0, 0, 0, 0 };
float Pw, Iw, Dw;
int errorw = 0;
float previousErrorw = 0;
float Kpw = 5;
float Kdw = 0;
float Kiw = 0;
int speedAdjustw = 0;

float P_angle, I_angle, D_angle;
float error_angle;
float previousError_angle = 0;
float Kp_angle = 6.5;
float Kd_angle = 0;
float Ki_angle = 0;
int speedAdjust_angle = 0;

bool check_wall();

bool wall_at_left = false;
bool wall_at_right = false;
bool wall_following = false;

Servo servoMotor;


void setup() {
  Serial.begin(9600);


  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  servoMotor.attach(6);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
  delay(100);
  servoMotor.write(20);
  delay(15);
}

void loop() {
  read_IR();
  if (!wall_following) {
    if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
      if (check_for_walls()) {
        if (wall_at_left) {
          turn_angle(-90);
          wall_following = true;
          wall_follow();
        }else{
          wall_following = true;
          wall_follow();
        }
      }
    } else {
      PID_control();
      int ml = 120;
      int mr = 120;

      mdrive(ml + speedAdjust, mr - speedAdjust);
    }

    read_IR();
    PID_control();
    int ml = 200;
    int mr = 200;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }else if(wall_following && !wall_follow_finished){
    wall_follow();
    read_IR();
    if(IR_val[0] == 1 || IR_val[1] == 1 || IR_val[2] == 1 || IR_val[3] == 1 || IR_val[4] == 1 || IR_val[5] == 1 || IR_val[6] == 1 || IR_val[7] == 1){
      wall_follow_finished = true;
      line_follow();
    }
  }else if(wall_following && wall_follow_finished){
    line_follow();
  }
}

void read_IR() {
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

void PID_control() {

  error = 0;

  for (int i = 0; i < 8; i++) {
    error += IR_weights[i] * IR_val[i];
  }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;
}

void mdrive(int ml, int mr) {
  if (ml > 0) {
    if (ml > 255) {
      ml = 255;
    }
    digitalWrite(LMotorA, HIGH);
    digitalWrite(LMotorB, LOW);
    analogWrite(LMotorPWM, ml);
  } else {
    if (ml < -255) {
      ml = -255;
    }
    digitalWrite(LMotorA, LOW);
    digitalWrite(LMotorB, HIGH);
    analogWrite(LMotorPWM, -1 * ml);
  }
  if (mr > 0) {
    if (mr > 255) {
      mr = 255;
    }
    digitalWrite(RMotorA, HIGH);
    digitalWrite(RMotorB, LOW);
    analogWrite(RMotorPWM, mr);
  } else {
    if (mr < -255) {
      mr = -255;
    }
    digitalWrite(RMotorA, LOW);
    digitalWrite(RMotorB, HIGH);
    analogWrite(RMotorPWM, -1 * mr);
  }
}

void turn_right() {
  mdrive(100, 100);
  delay(500);
  read_IR();
  while (IR_val[3] == 0 || IR_val[4] == 0) {
    read_IR();
    mdrive(-90, 90);
    delay(10);
  }
}

void turn_left() {
  mdrive(100, 100);
  delay(500);
  read_IR();
  while (IR_val[3] == 0 || IR_val[4] == 0) {
    read_IR();
    mdrive(90, -90);
    delay(10);
  }
}

bool check_for_walls() {
  int dis1 = SonarL1.ping_cm();
  Serial.print("1: ");
  Serial.println(dis1);
  int dis2 = SonarTD.ping_cm();
  Serial.print("2: ");
  Serial.println(dis2);
  if (dis1 < 50 || dis2 < 50) {
    if (dis1 < 50) {
      wall_at_left = true;
    } else {
      wall_at_right = true;
    }
    return true;
  }
  return false;
}

void zreadings() {
  mpu.update();  // print data every 10ms
  // Serial.print("Y : ");
  //Serial.println(mpu.getAngleZ());
}

void PID_control_angle(float target_angle, float current_initial_angle, float current_angle) {
  error_angle = 0;
  target_angle += current_initial_angle;
  error_angle = target_angle - current_angle;
  P_angle = error_angle;
  I_angle = I_angle + error_angle;
  D_angle = error_angle - previousError_angle;

  previousError_angle = error_angle;

  speedAdjust_angle = Kp_angle * P_angle + Ki_angle * I_angle + Kd_angle * D_angle;
  // Serial.print("speedAdjust  ");
  // Serial.println(speedAdjust);
}

void turn_angle(float target_angle) {
  mpu.update();  // print data every 10ms
  Serial.print("Y : ");
  Serial.println(mpu.getAngleZ());
  float initial_angle = mpu.getAngleZ();
  unsigned long start_time = millis();
  while (millis() - start_time < 1500) {
    unsigned long currentMillis = millis();  // Get the current time

    // Check if 10 milliseconds have passed since the last execution
    if (currentMillis - timer >= 10) {
      timer = currentMillis;  // Update the timer

      zreadings();                            // Read sensor data
      float current_angle = mpu.getAngleZ();  // Get current angle

      // Perform PID control
      PID_control_angle(target_angle, initial_angle, current_angle);

      // Adjust motor speeds based on PID output
      mdrive(-speedAdjust_angle, speedAdjust_angle);
    }
  }
}

bool check_wall() {
  if (Us_val[1] < 5 && Us_val[1] > 0) {
    return true;
  }
  return false;
}

void wall_follow(){
  read_US();
  Serial.println("BBB");
  if (check_wall()) {
    mdrive(0,0);
    delay(100);
    turn_angle(90);
  } else {
    Serial.println("A");
    PID_control();
    int ml = 100;
    int mr = 100;

    mdrive(ml+speedAdjustw, mr-speedAdjustw);
  }
}

void line_follow(){
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

  read_IR();
  PID_control();
  int ml = 200;
  int mr = 200;

  mdrive(ml+speedAdjust, mr-speedAdjust);
}

void read_US() {
  Us_val[0] = SonarL1.ping_cm();
  Serial.println(Us_val[0]);
  Us_val[1] = SonarF1.ping_cm();
  //Serial.println(Us_val[1]);
  Us_val[2] = SonarT.ping_cm();
  //Serial.println(Us_val[2]);
  // TCA9548A(2);
  // Us_val[3] = sensor2.readRangeContinuousMillimeters() / 10;
  //Serial.println(Us_val[3]);
  //Us_val[4] = SonarTD.ping_cm();
  delay(50);
}