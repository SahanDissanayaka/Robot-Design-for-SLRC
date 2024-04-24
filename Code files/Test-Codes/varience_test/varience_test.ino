#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>

#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C



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
void display_line();


VL53L0X sensor1;
VL53L0X sensor2;

int count = 0;
int distance_array[100];
int array_size=100;
double varience = 0;
float mean;
int total;
int minimum = 400;
int maximum = 0;


void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}




void setup() {
  Serial.begin(9600);
  //Wire.begin();

  // TCA9548A(1);
  // sensor.init();
  // sensor.setTimeout(500);

  if (!display.begin(SSD1306_SWITCHCAPVCC, screen_address)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true) {}
  }

  display.display();
  delay(2000);

  display.clearDisplay();
  display_line("We are Groot", 0, 0,2);
  
  

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  Wire.begin();
  TCA9548A(1);

  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous();

  TCA9548A(2);

  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor2.startContinuous();

}

void loop() {
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
    int ml = 150;
    int mr = 150;
    mdrive(ml+speedAdjust, mr-speedAdjust);

  }

if(count < array_size){
  TCA9548A(1);
  int distance1 = sensor1.readRangeContinuousMillimeters();
  if(distance1 < 230){
    distance_array[count] = distance1;
    display.clearDisplay();
    display_line(String(count), 0, 10,2);
    display_line(String(distance1),0,30,2);
    if(distance1 <minimum){
      minimum = distance1;
    }
    if(distance1 > maximum){
      maximum = distance1;
    }
    count++;
  }

}else{
  total = 0; 
  for(int i = 0;i<array_size;i++){
    total += distance_array[i];
  }
  mean = total/100.0;
  // display.clearDisplay();
  // display_line(String(mean), 0, 20,2);

  varience = 0;
  for(int i = 0;i<array_size;i++){
    varience += pow(distance_array[i] - mean,2);
  }
  varience = varience/100;
  // display_line(String(varience), 0, 40,2);

  display.clearDisplay();
  display_line(String(minimum), 0, 20,2);
  display_line(String(maximum), 0, 40,2);

  
}

  
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

  //  Serial.print(IR_val[0]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[1]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[2]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[3]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[4]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[5]);
  //  Serial.print(" ");
  //  Serial.print(IR_val[6]);
  //  Serial.print(" ");
  //  Serial.println(IR_val[7]);
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
    mdrive(-150, 150);
    delay(10);
  }
}

void turn_left(){
  mdrive(100,100);
  delay(500);
  read_IR();
  while ( IR_val[3] == 0 || IR_val[4] == 0){
    read_IR();
    mdrive(150, -150);
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