#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define screen_width 128
#define screen_height 64
#define oled_reset -1
#define screen_address 0x3C

// #define S0 42
// #define S1 40
// #define S2 38
// #define S3 36
// #define sensorOut 32

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

// int redMin = 229;
// int redMax = 660;
// int greenMin = 271;
// int greenMax = 724;
// int blueMin = 270;
// int blueMax = 555;

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

Adafruit_SSD1306 display(screen_width, screen_height, &Wire, oled_reset);

void display_line(String text, int column, int row, int text_size) {

  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);

  display.display();
}

void setup() {
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

  // Setup Serial Monitor
  Serial.begin(9600);

  // if (!display.begin(SSD1306_SWITCHCAPVCC, screen_address)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  //   while (true) {}
  // }

  // display.display();
  // delay(2000);

  // display.clearDisplay();
  // display_line("We are Groot", 0, 0,2);

  // Wire.begin();
}

void loop() {
  // Read Red Pulse Width
  redPW = getRedPW();
  redValue = map(redPW, redMin, redMax, 255, 0);
  // Delay to stabilize sensor
  //delay(200);

  // Read Green Pulse Width
  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  // Delay to stabilize sensor
  //delay(200);

  // Read Blue Pulse Width
  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  // Delay to stabilize sensor
  //delay(200);

  // Print output to Serial Monitor
  Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" - Green = ");
  Serial.print(greenValue);
  Serial.print(" - Blue = ");
  Serial.println(blueValue);

  // Determine the dominant color
  if (redValue > 230 && blueValue > 230 && greenValue > 230) {
    Serial.println("Color is White");
    // display.clearDisplay();
    // display_line("White", 0, 0,2);
  }
  else if(redValue < 100 && blueValue < 100 && greenValue < 100){
    Serial.println("Color is Black");
    // display.clearDisplay();
    // display_line("Black", 0, 0,2);
  }
  else {
    if (redValue > greenValue && redValue > blueValue) {
      Serial.println("Color is Red");
      // display.clearDisplay();
      // display_line("White", 0, 0,2);
    } else if (greenValue > redValue && greenValue > blueValue) {
      Serial.println("Color is Green");
      // display.clearDisplay();
      // display_line("Green", 0, 0,2);
    } else if (blueValue > greenValue && blueValue > redValue) {
      Serial.println("Color is Blue");
      // display.clearDisplay();
      // display_line("Blue", 0, 0,2);
    }
  }
}

// Function to read Red Pulse Widths
int getRedPW() {
  // Set sensor to read Red only
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Read the output Pulse Width
  int PW = pulseIn(sensorOut, LOW);

  // Return the value
  return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
  // Set sensor to read Green only
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  // Read the output Pulse Width
  int PW = pulseIn(sensorOut, LOW);

  // Return the value
  return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
  // Set sensor to read Blue only
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  // Read the output Pulse Width
  int PW = pulseIn(sensorOut, LOW);

  // Return the value
  return PW;
}