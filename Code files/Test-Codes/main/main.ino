#include <MPU6050_light.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>
#include <Servo.h>

// display
#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C

// servo
#define SERVO_PIN 4


// IR array
#define IR1 45
#define IR2 43
#define IR3 41
#define IR4 39
#define IR5 37
#define IR6 35
#define IR7 33
#define IR8 31

// LEDs
#define Green_LED 46
#define Blue_LED 52
bool object_detected = false;
int task_count = 1;

// Arm
#define ArmMotorA 30
#define ArmMotorB 28
#define ArmMotorPWM 8


// Motors
#define LMotorA 51
#define LMotorB 53
#define LMotorPWM 2

#define RMotorA 49
#define RMotorB 47
#define RMotorPWM 3

// Metal
#define metal A0

#define MAX_SPEED 240

// Color Front
#define FS0 42
#define FS1 40
#define FS2 38
#define FS3 36
#define FsensorOut 32

int FredMin = 130;
int FredMax = 475;
int FgreenMin = 140;
int FgreenMax = 535;
int FblueMin = 125;
int FblueMax = 445;

// Variables for Color Pulse Width Measurements
int FredPW = 0;
int FgreenPW = 0;
int FbluePW = 0;

int FredValue;
int FgreenValue;
int FblueValue;

int distance;
String wall_colour;
bool got_colour = false;

// color back
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

//detecting colour
String detected_color;

bool path_founded = false;

// Objects
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);
Servo servoMotor;
VL53L0X sensor1;
VL53L0X sensor2;

int IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int IR_weights[8] = { -20, -15, -10, -5, 5, 10, 15, 20 };
float P = 0, I = 0, D = 0;
float error;
float previousError = 0;
float Kp = 2.5, Ki = 0.01, Kd = 0;
int speedAdjust = 0;
int box_grapped = 0;
int box_palce = 0;

void read_IR();
void PID_control();
void normal_line_follow();
void line_follow();
void mdrive(int ml, int mr);
void F_color_ditection();
int tof_distance_2();
void box();
void find_box();
void check_metal();
void driveArmMotor(int speed, int Del);
void servo_dis(int x1, int x2);
void servo_in(int x1, int x2);
void displayDistance();
void turn_left();
void turn_right();
bool noLineDetected();
void adjustDirection();
void adjustSpeed();
void display_line(String text, int column, int row, int text_size);
bool t_junction_1();
bool t_junction_2();
bool t_junction_3();

// Angle turning
float P_angle, I_angle, D_angle;
float error_angle;
float previousError_angle = 0;
float Kp_angle = 6.5;
float Kd_angle = 0;
float Ki_angle = 0;
int speedAdjust_angle = 0;

String colour_direction;
String want_color;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void turn_angle(float target_angle);

int count = 0;
int distance_array[400];
int array_size = 400;
float varience = 0;
float mean;
int total;


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

  servoMotor.attach(SERVO_PIN);
  pinMode(ArmMotorA, OUTPUT);
  pinMode(ArmMotorB, OUTPUT);
  pinMode(ArmMotorPWM, OUTPUT);
  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);
  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);
  pinMode(Green_LED, OUTPUT);
  pinMode(Blue_LED, OUTPUT);

  Wire.begin();
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
  servo_in(0, 60);

  mpu.begin();
  //Serial.println("Calculating gyro offset, do not move MPU6050");
  mpu.calcGyroOffsets();

  // color back
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Sensor output as input
  pinMode(sensorOut, INPUT);

  // Set Pulse Width scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // color front

  pinMode(FS0, OUTPUT);
  pinMode(FS1, OUTPUT);
  pinMode(FS2, OUTPUT);
  pinMode(FS3, OUTPUT);

  // Set Sensor output as input
  pinMode(FsensorOut, INPUT);

  // Set Pulse Width scaling to 20%
  digitalWrite(FS0, HIGH);
  digitalWrite(FS1, LOW);
}

void loop() {
  if(task_count == 1){
    F_color_ditection();
    display.clearDisplay();
    display_line("task1",0,20,2);
  }else if(task_count == 2){
    tast2_line_follow();
    display.clearDisplay();
    display_line("task2",0,20,2);
  }else if(task_count == 3){
    path_detection();
    display.clearDisplay();
    display_line("task3",0,20,2);
  }else if(task_count == 4){
    object_detection();
    display.clearDisplay();
    display_line("task4",0,20,2);
  }else if(task_count == 5){
    task5();
    display.clearDisplay();
    display_line("task5",0,20,2);
  }else if(task_count == 6){
    box();
    display.clearDisplay();
    display_line("task6",0,20,2);
  }
  else if(task_count == 7){
    place_box();
    display.clearDisplay();
    display_line("task7",0,20,2);
  } 
  else if(task_count == 8){
    tast2_line_follow();
    display.clearDisplay();
    display_line("task8",0,20,2);
  }  
  else if(task_count == 9){
    final();
    display.clearDisplay();
    display_line("task9",0,20,2);
  }
  else if(task_count == 10){
    mdrive(0,0);
    delay(100);
    display.clearDisplay();
    display_line("task9",0,20,2);
  }     
  // if (box_grapped == 0 ){
  //   box();
  // }
  //F_color_ditection();
}

// Color
void F_color_ditection() {
  if (!got_colour) {
    TCA9548A(2);
    distance = sensor2.readRangeContinuousMillimeters();
    display.clearDisplay();
    display_line(String(distance), 0, 0, 2);

    if (distance <= 80) {
      mdrive(0, 0);
      delay(500);
      wall_colour = F_get_color();
      display.clearDisplay();
      display_line(wall_colour, 0, 0, 2);
      got_colour = true;

      turn_angle(180);
    } else {

      normal_line_follow();
    }
  } 
  else {
    read_IR();
    if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1) {
      mdrive(150, 150);
      delay(1000);
      while (!noLineDetected()) {
        read_IR();
        mdrive(-150, 150);
      }
      while (IR_val[3] == 0 || IR_val[4] == 0) {
        read_IR();  // Corrected function call
        mdrive(-150, 150);
        delay(10);
      }
      task_count ++;
    }
    else {
      PID_control();
      int ml = 180;
      int mr = 180;

      mdrive(ml + speedAdjust, mr - speedAdjust);
    }
  }
}

void tast2_line_follow() {
  read_IR();

  if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
    mdrive(150, 150);
    delay(500);
    while (!noLineDetected()) {
      read_IR();
      mdrive(-150, 150);
    }
    turn_left();
    task_count ++;
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
    task_count ++;
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
    int ml = 180;
    int mr = 180;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }
}

//Path detection
void path_detection() {
  if (!path_founded) {
    detected_color = color();
    if (detected_color == "Red") {
      String check_color = color();
      if (check_color == "Red") {
        Serial.println("Red found!!!");
        mdrive(0, 0);
        delay(1000);
        turn_angle(90);
        // mdrive(-150, 150);
        // delay(1500);

        String second_colour = color();
        if (second_colour == wall_colour) {
          colour_direction = "left";
          path_founded = true;
        } else {
          turn_angle(180);
          colour_direction = "right";
          path_founded = true;
          
        }
        task_count++;
      } else {
        normal_line_follow();
        Serial.println("AAA");
      }
    } else {
      Serial.println("BBB");
      normal_line_follow();
    }
  } else {
    Serial.println("CCC");
    line_follow();
  }
}

// Box
void box() {
  read_IR();
  while (!(t_junction_3())) {
    normal_line_follow();
  }
  mdrive(200, 200);
  delay(500);
  read_IR();
  while (!noLineDetected()) {
    read_IR();
    mdrive(-200, 200);
  }
  turn_left();
  find_box();
  if (box_grapped == 1) {
    if (box_palce == 1) {
      while (t_junction_3()) {
        normal_line_follow();
      }
      mdrive(200, 200);
      delay(500);
      read_IR();
      while (!noLineDetected()) {
        read_IR();
        mdrive(200, -200);
      }
      turn_right();
    } else if (box_palce == 2) {
      while (t_junction_3()) {
        normal_line_follow();
      }
      normal_line_follow();
    } else if (box_palce == 3) {
      while (t_junction_3()) {
        normal_line_follow();
      }
      mdrive(200, 200);
      delay(500);
      read_IR();
      while (!noLineDetected()) {
        read_IR();
        mdrive(-200, 200);
      }
      turn_left();
    }
    while(!t_junction_3()){
        normal_line_follow();
    }
    mdrive(150,150);
    delay(1000); 
    mdrive(0,0);
    task_count ++;
  }
}

// object detection
void object_detection() {
  if (!object_detected) {
    line_follow();

    if (count < array_size) {
      TCA9548A(1);
      int distance1 = sensor1.readRangeContinuousMillimeters();
      if (distance1 < 230) {
        distance_array[count] = distance1;
        display.clearDisplay();
        display_line(String(count), 0, 10, 2);
        display_line(String(distance1), 0, 30, 2);
        count++;
      }

    } else {
      total = 0;
      for (int i = 0; i < array_size; i++) {
        total += distance_array[i];
      }
      mean = total / 400.0;
      display.clearDisplay();
      display_line(String(mean), 0, 20, 2);

      varience = 0;
      for (int i = 0; i < array_size; i++) {
        varience += pow(distance_array[i] - mean, 2);
      }
      varience = varience / 400;
      display_line(String(varience), 0, 40, 2);

      if (varience <= 1000) {
        // cylinder
        digitalWrite(Green_LED, HIGH);
        object_detected = true;
      } else {
        // cuboid
        digitalWrite(Blue_LED, HIGH);
        object_detected = true;
      }
      
    }
  }
  else{
    tast2_line_follow();
  }
  
}

void task5(){
    detected_color = color();
    if(wall_colour == "Green"){
      want_color = "Blue";
    }else{
      want_color = "Green";
    }
    if(detected_color == want_color){
      String check_color = color();
      if(check_color == want_color){
        mdrive(0,0);
        delay(1000);
        if(colour_direction = "left"){
          turn_angle(90);
        }else{
          turn_angle(-90);
        }
        
        task_count++;

      }else{
        normal_line_follow();
      }
    }else{
        normal_line_follow();
      } 
}

void place_box(){
  while(tof_distance_2()>=14){
    line_follow();
  }
  mdrive(0,0);
  delay(100);
  driveArmMotor(-255, 900);
  servo_in(0,180);
  mdrive(-150,-150);
  delay(1000);
  mdrive(0,0);
  delay(1000);
  servo_dis(0,180);
  while(!(t_junction_3())){
    mdrive(150,150);
  }
  mdrive(0,0);
  delay(1000);
  mdrive(-150,-150);
  delay(1000);
  mdrive(0,0);
  delay(1000);
  turn_angle(180);
  tast2_line_follow();
}
void final(){
  while(!t_junction_3()){
    normal_line_follow();
  }
  mdrive(150,150);
  delay(500);
  mdrive(0,0);
  task_count++;
}

int F_getFRedPW() {
  digitalWrite(FS2, LOW);
  digitalWrite(FS3, LOW);
  int PW = pulseIn(FsensorOut, LOW);
  return PW;
}

int F_getFGreenPW() {
  digitalWrite(FS2, HIGH);
  digitalWrite(FS3, HIGH);
  int PW = pulseIn(FsensorOut, LOW);
  return PW;
}

int F_getFBluePW() {
  digitalWrite(FS2, LOW);
  digitalWrite(FS3, HIGH);
  int PW = pulseIn(FsensorOut, LOW);
  return PW;
}

String F_color() {
  FredPW = F_getFRedPW();
  FredValue = map(FredPW, FredMin, FredMax, 255, 0);
  delay(200);

  FgreenPW = F_getFGreenPW();
  FgreenValue = map(FgreenPW, FgreenMin, FgreenMax, 255, 0);
  delay(200);

  FbluePW = F_getFBluePW();
  FblueValue = map(FbluePW, FblueMin, FblueMax, 255, 0);
  delay(200);

  // Serial.print("Red = ");
  Serial.print(FredValue);
  Serial.print(" ");
  //Serial.print(" --- Green = ");
  Serial.print(FgreenValue);
  Serial.print(" ");
  //Serial.print(" --- Blue = ");
  Serial.println(FblueValue);
  // Serial.print(" ");

  if (FredValue > 300 && FblueValue > 300 && FgreenValue > 300) {
    Serial.println("Color is White");
    return "White";
  } else if (FredValue < 200 && FblueValue < 200 && FgreenValue < 200) {
    Serial.println("Color is Black");
    return "Black";
  } else {
    if (FredValue > FgreenValue && FredValue > FblueValue) {
      Serial.println("Color is Red");

      return "Red";
    } else if (FgreenValue > FredValue && FgreenValue > FblueValue) {
      Serial.println("Color is Green");

      return "Green";
    } else if (FblueValue > FgreenValue && FblueValue > FredValue) {
      Serial.println("Color is Blue");

      return "Blue";
    }
  }
}

String F_get_color() {
  int blue_count = 0;
  int green_count = 0;
  for (int i = 0; i < 10; i++) {
    String colour = F_color();
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

void normal_line_follow() {
  
  // read_IR();
  // PID_control();
  // int ml = 200;
  // int mr = 200;

  // mdrive(ml + speedAdjust, mr - speedAdjust);

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
    int ml = 180;
    int mr = 180;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }
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
    int ml = 180;
    int mr = 180;

    mdrive(ml + speedAdjust, mr - speedAdjust);
  }

  // read_IR();
  // PID_control();
  // int ml = 200;
  // int mr = 200;

  // mdrive(ml+speedAdjust, mr-speedAdjust);

  // if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
  //   mdrive(100, 100);
  //   delay(100);
  //   read_IR();
  //   if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
  //     delay(100);
  //     read_IR();
  //     if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
  //       delay(100);
  //       read_IR();
  //       if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
  //         while(1){
  //           delay(200);
  //           mdrive(0,0);
  //         }
  //       }
  //       else{
  //         mdrive(-120, 120);
  //         delay(350);
  //       }
  //     }
  //     else{
  //       mdrive(-120,120);
  //       delay(350);
  //     }
  //   }
  //   else{
  //       mdrive(-120,120);
  //       delay(350);
  //     }

  // }
}

int tof_distance_2() {
  TCA9548A(2);
  int distance2 = sensor2.readRangeContinuousMillimeters();
  display.clearDisplay();
  display_line(String(distance2), 0, 0, 2);
  return distance2;
}

void find_box() {
  box_palce += 1;
  while (tof_distance_2() >= 60 && !(t_junction_3())) {
    normal_line_follow();
  }
  if (tof_distance_2() <= 60) {
    while (tof_distance_2() >= 30) {
      mdrive(150, 150);
    }
    mdrive(0, 0);
    delay(100);
    servo_dis(0, 150);
    check_metal();
  } else {
    read_IR();
    while (!(noLineDetected())) {
      read_IR();
      mdrive(-150, 150);
    }
    turn_left();
    mdrive(0, 0);
  }
}

void check_metal() {
  if (digitalRead(metal) == 1) {
    driveArmMotor(255, 1000);
    box_grapped = 1;
    driveArmMotor(0, 1000);
    delay(1000);
  } else {
    servo_in(0, 180);
    mdrive(-100, -100);
    delay(1000);
  }
  read_IR();
  while (!(noLineDetected())) {
    read_IR();
    mdrive(-150, 150);
  }
  turn_left();
  mdrive(0, 0);
}

void servo_dis(int x1, int x2) {
  for (int angle = x1; angle <= x2; angle += 1) {
    servoMotor.write(angle);
    delay(15);
  }
}

void servo_in(int x1, int x2) {
  for (int angle = x2; angle >= x1; angle -= 1) {
    servoMotor.write(angle);
    delay(15);
  }
}

void driveArmMotor(int speed, int Del) {
  analogWrite(ArmMotorPWM, abs(speed));

  digitalWrite(ArmMotorA, speed > 0 ? HIGH : LOW);
  digitalWrite(ArmMotorB, speed > 0 ? LOW : HIGH);
  delay(Del);
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

bool noLineDetected() {
  return (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0);
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
  while (IR_val[3] == 0 || IR_val[2] == 0) {
    read_IR();
    mdrive(150, -150);
    delay(10);
  }
}

void turn_left() {
  mdrive(100, 100);
  delay(500);
  read_IR();
  while (IR_val[5] == 0 || IR_val[4] == 0) {
    read_IR();
    mdrive(-150, 150);
    delay(10);
  }
}


// Wam paththata
bool t_junction_1() {
  return (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1);
}

// Dakunu paththata
bool t_junction_2() {
  return (IR_val[7] == 1 && IR_val[6] == 1 && IR_val[5] == 1 && IR_val[4] == 1 && IR_val[3] == 1);
}

bool t_junction_3() {
  return (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1);
}

void display_line(String text, int column, int row, int text_size) {

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);

  display.display();
}

// MPU6050

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