#define S0 42
#define S1 40
#define S2 38
#define S3 36
#define sensorOut 32


// int redMin = 229;
// int redMax = 660;
// int greenMin = 271;
// int greenMax = 724;
// int blueMin = 270;
// int blueMax = 555;

int redMin = 130;
int redMax = 475;
int greenMin = 140;
int greenMax = 535;
int blueMin = 125;
int blueMax = 445;

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

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
}

void loop() {
  // Read Red Pulse Width
  redPW = getRedPW();
  redValue = map(redPW, redMin, redMax, 255, 0);
  // Delay to stabilize sensor
  delay(200);

  // Read Green Pulse Width
  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  // Delay to stabilize sensor
  delay(200);

  // Read Blue Pulse Width
  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  // Delay to stabilize sensor
  delay(200);

  // Print output to Serial Monitor
  Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" - Green = ");
  Serial.print(greenValue);
  Serial.print(" - Blue = ");
  Serial.println(blueValue);

  // Determine the dominant color
  if (redValue > greenValue && redValue > blueValue) {
    Serial.println("Color is Red");
  } else if (greenValue > redValue && greenValue > blueValue) {
    Serial.println("Color is Green");
  } else if (blueValue > greenValue && blueValue > redValue) {
    Serial.println("Color is Blue");
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