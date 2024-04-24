#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>

#include <VL53L0X.h>

#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C

#define S0 42
#define S1 40
#define S2 38
#define S3 36
#define sensorOut 32

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

#define MAX_SPEED 240

Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

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
float Kd = 0;
float Ki = 0;

void read_IR();
void PID_control();
void mdrive();
void stop();
void turn_left();
void turn_right();
void display_line();
String get_color();
void line_follow_normal();
void line_follow();

// Angle turning
float P_angle, I_angle, D_angle;
float error_angle;
float previousError_angle = 0;
float Kp_angle = 6.5;
float Kd_angle = 0;
float Ki_angle = 0;
int speedAdjust_angle = 0;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void turn_angle(float target_angle);


VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;// wall middle
VL53L0X sensor4;// wall up
VL53L0X sensor5;
int distance;

String wall_colour;
bool got_colour = false;

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, screen_address)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true) {}
  }

  display.display();
  delay(2000);

  display.clearDisplay();
  display_line("We are Groot", 0, 0, 2);



  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  Wire.begin();
  TCA9548A(1);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor1.startContinuous();

  TCA9548A(2);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor2.startContinuous();

  TCA9548A(3);

  sensor3.setTimeout(500);
  if (!sensor3.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor3.startContinuous();

  TCA9548A(4);

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

  mpu.begin();
  //Serial.println("Calculating gyro offset, do not move MPU6050");
  mpu.calcGyroOffsets();
}

void loop() {
  if (!got_colour) {
    TCA9548A(2);
    distance = sensor2.readRangeContinuousMillimeters();
    display.clearDisplay();
    display_line(String(distance), 0, 0, 2);

    if (distance <= 70) {
      mdrive(0, 0);
      delay(500);
      turn_angle(180);
      TCA9548A(2);
      int d = sensor2.readRangeContinuousMillimeters();
      while(d < 5){
        mdrive(-80,-80);
        delay(15);
        TCA9548A(5);
        d = sensor5.readRangeContinuousMillimeters();
      }
      wall_colour = get_color();
      display.clearDisplay();
      display_line(wall_colour, 0, 0, 2);
      got_colour = true;

      
    } else {

      line_follow_normal();
    }
  } else {

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
    mdrive(150, -150);
    delay(10);
  }
}

void turn_left() {
  mdrive(100, 100);
  delay(500);
  read_IR();
  while (IR_val[3] == 0 || IR_val[4] == 0) {
    read_IR();
    mdrive(-150, 150);
    delay(10);
  }
}

void display_line(String text, int column, int row, int text_size) {

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);

  display.display();
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
  delay(200);

  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  delay(200);

  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  delay(200);

  // Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" ");
  //Serial.print(" --- Green = ");
  Serial.print(greenValue);
  Serial.print(" ");
  //Serial.print(" --- Blue = ");
  Serial.println(blueValue);
  // Serial.print(" ");

  if (redValue > 300 && blueValue > 300 && greenValue > 300) {
    Serial.println("Color is White");
    return "White";
  } else if (redValue < 200 && blueValue < 200 && greenValue < 200) {
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

String get_color() {
  int blue_count = 0;
  int green_count = 0;
  for (int i = 0; i < 10; i++) {
    String colour = color();
    if (colour == "Green") {
      green_count++;
    } else {
      blue_count++;
    }
  }
  if (blue_count < green_count) {
    return "Green";
  }
  return "Blue";
}

void line_follow_normal() {
  read_IR();
  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
    if (previousError < 0) {
      mdrive(150, 150);
      delay(500);
      turn_left();

    } else {
      mdrive(150, 150);
      delay(500);
      turn_right();
    }
  } else {
    PID_control();
    int ml = 90;
    int mr = 90;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }

  read_IR();
  PID_control();
  int ml = 90;
  int mr = 90;

  mdrive(ml + speedAdjust, mr - speedAdjust);
}

void line_follow() {
  read_IR();

  if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
    mdrive(150, 150);
    delay(500);
    while (!noLineDetected()) {
      read_IR();
      mdrive(-150, 150);
    }
    turn_left();
  }

  else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1) {
    mdrive(150, 150);
    delay(1000);
    while (!noLineDetected()) {
      read_IR();
      mdrive(-150, 150);
    }
    while (IR_val[5] == 0 || IR_val[4] == 0) {
      read_IR();  // Corrected function call
      mdrive(-150, 150);
      delay(10);
    }
  }

  // no line detection
  else if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
    if (previousError < 0) {
      mdrive(150, 150);
      delay(500);
      turn_left();

    } else {
      mdrive(150, 150);
      delay(500);
      turn_right();
    }
  } else {
    PID_control();
    int ml = 90;
    int mr = 90;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }
}

bool noLineDetected() {
  return (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0);
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