#include <NewPing.h>
#include <Wire.h>
#include <MPU6050_light.h>

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

#define max_distance 100
#define safety_distance 10

NewPing SonarL1(USL1_Trig, USL1_Echo, max_distance);
NewPing SonarF1(USF1_Trig, USF1_Echo, max_distance);


int Us_val[2] = { 0, 0 };
float Pw, Iw, Dw;
int errorw = 0;
float previousErrorw = 0;
float Kpw = 3;
float Kdw = 0;
float Kiw = 0;

int speedAdjustw = 0;
// bool wall_detected = false;

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

bool check_wall();

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
  delay(100);
}

void loop() {
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

void PID_control() {
  Serial.println("PID");
  //-------------------------Wall following----------------------------//
  errorw = 0;

  
  errorw = Us_val[0] - safety_distance;
  Serial.println(String(errorw));
  
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
  //Serial.println(Us_val[0]);
  Us_val[1] = SonarF1.ping_cm();
  //Serial.println(Us_val[1]);
  delay(50);
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