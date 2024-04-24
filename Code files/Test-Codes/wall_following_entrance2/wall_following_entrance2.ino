#include <NewPing.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <VL53L0X.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C

Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

#define Green_LED1 46
#define Blue_LED1 52
#define Green_LED2 44
#define Blue_LED2 50
#define Red_LED2 48

#define LMotorA 51
#define LMotorB 53
#define LMotorPWM 2

#define RMotorA 49
#define RMotorB 47
#define RMotorPWM 3

#define USL1_Trig 24
#define USL1_Echo 14

#define USF1_Trig 26
#define USF1_Echo 19

#define UST_Trig 11
#define UST_Echo 10

#define USTD_Trig 22
#define USTD_Echo 18

#define max_distance 100
#define safety_distance 8

NewPing SonarL1(USL1_Trig, USL1_Echo, max_distance);
NewPing SonarF1(USF1_Trig, USF1_Echo, max_distance);
NewPing SonarT(UST_Trig, UST_Echo, max_distance);
NewPing SonarTD(USTD_Trig, USTD_Echo, max_distance);

int speedAdjust = 0;



float P,I,D;
float error;
float previousError =0;
float Kp = 2.5;
float Kd = 0.5;
float Ki = 0.01;

Servo servoMotor;

int IR_val[8] = {0,0,0,0,0,0,0,0};
int pre_IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_weights[8] = {-20,-15,-10,-5,5,10,15,20};


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

float angle_before_turn;
float angle_after_turn;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void turn_angle(float target_angle);
void tower();
void obstacle_detection();

bool check_wall();
int angle = 20;
int pos = 10;
bool tower_in_left = false;
bool tower_in_middle = false;
int middle_dist;
int up_dist;
int height;
int max_height = 0;
bool max_height_found = false;
bool check_near_tower();
bool is_wall_present();
bool check_away_tower();
void check_height_away();
bool check_for_walls();
bool wall_follow_finished = false;

bool wall_at_left = false;
bool wall_at_right = false;
bool wall_following = false;

void wall_follow(){

}

VL53L0X sensor1;  // center
VL53L0X sensor2;  // wall below
VL53L0X sensor3;  // wall middle
VL53L0X sensor4;  // wall up


void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void setup() {
  Serial.begin(9600);
  pinMode(Green_LED1,OUTPUT);
  pinMode(Blue_LED1,OUTPUT);
  pinMode(Green_LED2,OUTPUT);
  pinMode(Blue_LED2,OUTPUT);
  pinMode(Red_LED2,OUTPUT);

  display.display();
  delay(2000);

  display.clearDisplay();
  display_line("We are Groot", 0, 0, 2);

  servoMotor.attach(6);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
  delay(100);

  TCA9548A(1);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  sensor1.startContinuous();

  TCA9548A(2);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor2.startContinuous();

  TCA9548A(4);

  sensor3.setTimeout(500);
  if (!sensor3.init()) {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor3.startContinuous();

  TCA9548A(3);

  sensor4.setTimeout(500);
  if (!sensor4.init()) {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }
  sensor4.startContinuous();
  servoMotor.write(120);
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
      PID_controlw();
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
    if(max_height = 5){
      digitalWrite(Red_LED2,HIGH);
    }else if(max_height == 10){
      digitalWrite(Green_LED2,HIGH);
    }else{
      digitalWrite(Blue_LED2,HIGH);
    }
  }

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

void PID_controlw() {
  //-------------------------Wall following----------------------------//
  errorw = 0;

  errorw = Us_val[0] - safety_distance;
  Pw = errorw;
  Iw = Iw + errorw;
  Dw = errorw - previousErrorw;

  previousErrorw = errorw;

  speedAdjustw = Kpw * Pw + Kiw * Iw + Kdw * Dw;
  //-------------------------Wall following----------------------------//
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

void read_US() {
  Us_val[0] = SonarL1.ping_cm();
  Serial.println(Us_val[0]);
  Us_val[1] = SonarF1.ping_cm();
  //Serial.println(Us_val[1]);
  Us_val[2] = SonarT.ping_cm();
  //Serial.println(Us_val[2]);
  TCA9548A(2);
  Us_val[3] = sensor2.readRangeContinuousMillimeters() / 10;
  //Serial.println(Us_val[3]);
  Us_val[4] = SonarTD.ping_cm();
  delay(50);
}

void zreadings() {
  mpu.update();  // print data every 10ms
  // Serial.print("Y : ");
  //Serial.println(mpu.getAngleZ());
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
  if ((Us_val[1] < 10 && Us_val[1] > 0) && ((Us_val[2] < 10 && Us_val[2] > 0) || Us_val[3] < 10)) {
    return true;
  }
  return false;
}

bool check_near_tower() {
  if ((Us_val[1] < 10 && Us_val[1] > 0) || ((Us_val[2] < 7 && Us_val[2] > 0) || Us_val[3] < 10)) {
    if (Us_val[2] < 10 && Us_val[3] < 10) {
      tower_in_middle = true;
    } else if (Us_val[3] < 10) {
      tower_in_middle = true;
    } else if (Us_val[2] < 7) {
      tower_in_left = true;
    } else if (Us_val[1] < 10) {
      tower_in_middle = true;
    }
    return true;
  } else {
    tower_in_left = false;
    tower_in_middle = false;
  }
  return false;
}

bool check_away_tower() {
  if (Us_val[4] < 50 && Us_val[4] > 0) {
    return true;
  }
  return false;
}

void check_height() {
  TCA9548A(4);
  middle_dist = sensor3.readRangeContinuousMillimeters() / 10;
  if (middle_dist < 19) {
    TCA9548A(3);
    up_dist = sensor4.readRangeContinuousMillimeters() / 10;
    if (up_dist < 15) {
      height = 20;
      display.clearDisplay();
      display_line("20", 0, 0, 2);
      max_height_found = true;
    } else {
      height = 15;
      display.clearDisplay();
      display_line("15", 0, 0, 2);
    }
  } else {
    height = 10;
    display.clearDisplay();
    display_line("10", 0, 0, 2);
  }
  if (height > max_height) {
    max_height = height;
  }
}

void check_height_away() {
  TCA9548A(4);
  middle_dist = sensor3.readRangeContinuousMillimeters() / 10;
  if (middle_dist < 54) {
    TCA9548A(3);
    up_dist = sensor4.readRangeContinuousMillimeters() / 10;
    if (up_dist < 50) {
      height = 20;
      display.clearDisplay();
      display_line("20", 0, 0, 2);
      max_height_found = true;
    } else {
      height = 15;
      display.clearDisplay();
      display_line("15", 0, 0, 2);
    }
  } else {
    height = 10;
    display.clearDisplay();
    display_line("10", 0, 0, 2);
  }
  if (height > max_height) {
    max_height = height;
  }
}

bool not_a_wall() {
  mdrive(-90, -85);
  delay(500);
  if (SonarTD.ping_cm() < 60) {
    mdrive(90,85);
    delay(1000);
    if(SonarTD.ping_cm() < 60){
      return false;
    }
    mdrive(-90,-85);
    delay(500);
    return true;
  }
  mdrive(90,85);
  delay(500);
  return true;
}

void display_line(String text, int column, int row, int text_size) {

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);

  display.display();
}

bool is_wall_present() {
  TCA9548A(2);
  int distance = sensor2.readRangeContinuousMillimeters() / 10;
  if (distance < 50) {
    return true;
  }
  return false;
}

void pass_tower() {
  if (is_wall_present()) {
    Us_val[1] = SonarF1.ping_cm();
    while (Us_val[1] > 10 || Us_val[1] == 0) {
      mdrive(90, 85);
      delay(15);
      Us_val[1] = SonarF1.ping_cm();
    }
    turn_angle(90);
  } else {
    mdrive(90, 85);
    delay(4000);
    turn_angle(-90);
    TCA9548A(2);
    int dis = sensor2.readRangeContinuousMillimeters() / 10;
    while (dis > 10) {
      mdrive(90, 85);
      TCA9548A(2);
      dis = sensor2.readRangeContinuousMillimeters() / 10;
      display.clearDisplay();
      display_line(String(dis), 0, 0, 2);
    }
    turn_angle(80);
  }
}

void wall_follow(){
  read_US();

  if (check_wall()) {
    display.clearDisplay();
    display_line("wall detected", 0, 0, 2);
    mdrive(0, 0);
    delay(100);
    turn_angle(90);
  } else if (check_near_tower()) {
    display.clearDisplay();
    display_line("tower detected", 0, 0, 2);
    mdrive(0, 0);
    delay(100);
    if (tower_in_left) {
      display.clearDisplay();
      display_line("tower in left", 0, 0, 2);
      delay(1000);
      servoMotor.write(20);
      check_height();
      servoMotor.write(120);
      turn_angle(90);
      mdrive(90, 90);
      delay(1600);
      turn_angle(-90);
      pass_tower();
    } else if (tower_in_middle) {
      display.clearDisplay();
      display_line("tower in middle", 0, 0, 2);
      delay(1000);
      mpu.update();
      angle_before_turn = mpu.getAngleZ();
      while (Us_val[2] > 15 || Us_val[2] == 0) {
        mdrive(90, -90);
        Us_val[2] = SonarT.ping_cm();
        delay(15);
      }
      servoMotor.write(20);
      check_height();
      servoMotor.write(120);
      delay(100);
      mpu.update();
      angle_after_turn = mpu.getAngleZ();
      turn_angle(90 - angle_after_turn + angle_before_turn);
      //turn_angle(90);
      mdrive(90, 90);
      delay(800);
      turn_angle(-90);
      pass_tower();
    }

  } else if (check_away_tower()) {
    mdrive(0, 0);
    delay(100);
    if (not_a_wall()) {
      check_height_away();
    }



  } else {
    PID_control();
    int ml = 100;
    int mr = 100;

    mdrive(ml + speedAdjustw, mr - speedAdjustw);
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