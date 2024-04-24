#include <Wire.h>
#include <MPU6050_light.h>

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


#define S0 29
#define S1 27
#define S2 25
#define S3 23
#define sensorOut 34

int redMin = 54;
int redMax = 379;
int greenMin = 57;
int greenMax = 407;
int blueMin = 50;
int blueMax = 373;

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

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

//detecting colour
String detected_color;

// Angle turning
float P_angle, I_angle, D_angle;
float error_angle;
float previousError_angle = 0;
float Kp_angle = 9;
float Kd_angle = 1;
float Ki_angle = 0;
int speedAdjust_angle = 0;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void turn_angle(float target_angle);

bool path_founded = false;

void setup() {
  Serial.begin(9600);
  // Set pins 4 to 7 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Sensor output as input
  pinMode(sensorOut, INPUT);

  // Set Pulse Width scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);


  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  Wire.begin();
  mpu.begin();
  //Serial.println("Calculating gyro offset, do not move MPU6050");
  mpu.calcGyroOffsets();
  delay(100);
}

void loop() {
  if (!path_founded) {
    detected_color = color();
    if (detected_color == "Red") {
      String check_color = color();
      if (check_color == "Red") {
        Serial.println("Red found!!!");
        mdrive(0, 0);
        delay(1000);
        turn_angle(90);
        String second_colour = color();
        if (second_colour == "Green") {
          path_founded = true;
          Serial.println("Green found!!!");
        } else {
          turn_angle(180);
          path_founded = true;
          Serial.println("Green found!!!");
        }
      }

    }else {
      read_IR();
      if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
        if (previousError < 0) {
          turn_right();
          Serial.println("aa");
        } else {
          turn_left();
          Serial.println("bb");
        }
      } else {
        PID_control();
        int ml = 120;
        int mr = 120;

        mdrive(ml + speedAdjust, mr - speedAdjust);
      }
    }
  } else {
    read_IR();
    if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
      if (previousError < 0) {
        turn_right();
        Serial.println("aa");
      } else {
        turn_left();
        Serial.println("bb");
      }
    } else {
      PID_control();
      int ml = 120;
      int mr = 120;

      mdrive(ml + speedAdjust, mr - speedAdjust);
    }
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

  // Serial.print(IR_val[0]);
  // Serial.print(" ");
  // Serial.print(IR_val[1]);
  // Serial.print(" ");
  // Serial.print(IR_val[2]);
  // Serial.print(" ");
  // Serial.print(IR_val[3]);
  // Serial.print(" ");
  // Serial.print(IR_val[4]);
  // Serial.print(" ");
  // Serial.print(IR_val[5]);
  // Serial.print(" ");
  // Serial.print(IR_val[6]);
  // Serial.print(" ");
  // Serial.println(IR_val[7]);
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
    mdrive(-150, 150);
    delay(10);
  }
}

void turn_left() {
  mdrive(100, 100);
  delay(500);
  read_IR();
  while (IR_val[3] == 0 || IR_val[4] == 0) {
    read_IR();
    mdrive(150, -150);
    delay(10);
  }
}

int getRedPW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getBluePW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

String color() {
  redPW = getRedPW();
  redValue = map(redPW, redMin, redMax, 255, 0);
  //delay(200);

  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  //delay(200);

  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  //delay(200);

  // Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" ");
  //Serial.print(" --- Green = ");
  Serial.print(greenValue);
  Serial.print(" ");
  //Serial.print(" --- Blue = ");
  Serial.println(blueValue);
  // Serial.print(" ");

  if (redValue > 230 && blueValue > 230 && greenValue > 230) {
    Serial.println("Color is White");
    return "White";
  } else if (redValue < 100 && blueValue < 100 && greenValue < 100) {
    Serial.println("Color is Black");
    return "Black";
  } else {
    if (redValue > greenValue && redValue > blueValue) {
      Serial.println("Color is Red");

      return "Red";
    } else if (greenValue > redValue && greenValue > blueValue) {
      Serial.println("Color is Green");

      return "Green";
    } else if (blueValue > greenValue && blueValue > redValue) {
      Serial.println("Color is Blue");

      return "Blue";
    }
  }
}

void zreadings() {
  mpu.update();  // print data every 10ms
  // Serial.print("Y : ");
  Serial.println(mpu.getAngleZ());
}

void PID_control_angle(float target_angle, float current_initial_angle, float current_angle) {
  target_angle += current_initial_angle;
  error_angle = target_angle - current_angle;
  P_angle = error_angle;
  I_angle = I + error_angle;
  D_angle = error_angle - previousError_angle;

  previousError_angle = error_angle;

  speedAdjust = Kp_angle * P_angle + Ki_angle * I_angle + Kd_angle * D_angle;
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
      mdrive(-speedAdjust, speedAdjust);
    }
  }
}
