#include <Wire.h>
#include <MPU6050_light.h>

#define LMotorA 51
#define LMotorB 53
#define LMotorPWM 2

#define RMotorA 49
#define RMotorB 47
#define RMotorPWM 3

#define buzzer 30

float P, I, D;
float error = 10;
float previousError;
float Kp = 9;
float Kd = 1;
float Ki = 0;
float speedAdjust;
float current_angle;
float initial_angle;

MPU6050 mpu(Wire);
unsigned long timer = 0;

void zreadings();
void PID_control_angle(float target_angle, float current_initial_angle);
void mdrive(int ml, int mr);

void setup() {
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  Serial.begin(115200);                           // Ensure serial monitor set to this value also    
  Wire.begin();
  mpu.begin();
  Serial.println("Calculating gyro offset, do not move MPU6050");
  mpu.calcGyroOffsets(); 
  delay(100);
  mpu.update();                         // print data every 10ms
  Serial.print("Y : ");
  Serial.println(mpu.getAngleZ());
  initial_angle =  mpu.getAngleZ();
  // mdrive(100,100);
  // delay(2000);
  turn_angle(-90);
  // mdrive(100,100);
  // delay(2000);
}

void loop() {
  
  // unsigned long currentMillis = millis();  // Get the current time
  
  // // Check if 10 milliseconds have passed since the last execution
  // if (currentMillis - timer >= 10) {
  //   timer = currentMillis;  // Update the timer
    
  //   zreadings();  // Read sensor data
  //   current_angle = mpu.getAngleZ();  // Get current angle

  //   // Perform PID control
  //   PID_control_angle(90, initial_angle);
    
  //   // Adjust motor speeds based on PID output
  //   mdrive(-speedAdjust, speedAdjust);
  // }
}

void zreadings(){
  mpu.update();                         // print data every 10ms
  // Serial.print("Y : ");
  Serial.println(mpu.getAngleZ());
}

void PID_control_angle(float target_angle, float current_initial_angle,float current_angle) {
  target_angle += current_initial_angle;
  error = target_angle - current_angle;
  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;
  Serial.print("speedAdjust  ");
  Serial.println(speedAdjust);
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

void turn_angle(float target_angle){
  mpu.update();                         // print data every 10ms
  Serial.print("Y : ");
  Serial.println(mpu.getAngleZ());
  initial_angle =  mpu.getAngleZ();
  unsigned long start_time = millis();
  while(millis() - start_time < 1500){
    unsigned long currentMillis = millis();  // Get the current time
  
    // Check if 10 milliseconds have passed since the last execution
    if (currentMillis - timer >= 10) {
      timer = currentMillis;  // Update the timer
      
      zreadings();  // Read sensor data
      current_angle = mpu.getAngleZ();  // Get current angle

      // Perform PID control
      PID_control_angle(target_angle, initial_angle,current_angle);
      
      // Adjust motor speeds based on PID output
      mdrive(-speedAdjust, speedAdjust);
    }
  }
}