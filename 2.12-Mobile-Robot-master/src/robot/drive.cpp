#include <Arduino.h>
#include "drive.h"

#define DIR1_L 32
#define PWM1_L 14
#define DIR2_L A0
#define PWM2_L A1
#define DIR1_R 33
#define PWM1_R 15
#define DIR2_R 12
#define PWM2_R 27



#define PWM1_L_CHANNEL 0
#define PWM1_R_CHANNEL 1
#define PWM2_L_CHANNEL 2
#define PWM2_R_CHANNEL 3

//frequency driving the motor controller in hz
#define PWM_FREQ 10000 
//max number of bits for pwm
#define PWM_RESOLUTION 10

void driveSetup() {
  //make all motor driver pins outputs
  pinMode(DIR1_L, OUTPUT);
  pinMode(PWM1_L, OUTPUT);
  pinMode(DIR2_L, OUTPUT);
  pinMode(PWM2_L, OUTPUT);
  pinMode(DIR1_R, OUTPUT);
  pinMode(PWM1_R, OUTPUT);
  pinMode(DIR2_R, OUTPUT);
  pinMode(PWM2_R, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(PWM1_L, PWM1_L_CHANNEL);
  ledcAttachPin(PWM2_L, PWM2_L_CHANNEL);
  ledcAttachPin(PWM1_R, PWM1_R_CHANNEL);
  ledcAttachPin(PWM2_R, PWM2_R_CHANNEL);

}

//drives the drivetrain based on voltage to send to the motors
void driveVoltsLR(float leftVolts, float rightVolts){
  driveLR(leftVolts/DRIVE_VOLTAGE, rightVolts/DRIVE_VOLTAGE);
}
//Function that uses PWM to drive the motors
//Values should be within +- the drivetrain voltage 
//Positive values always drive the robot forward
void driveVolts(float frontLeftVolts, float backLeftVolts, float frontRightVolts, float backRightVolts){
  drive(frontLeftVolts/DRIVE_VOLTAGE, backLeftVolts/DRIVE_VOLTAGE, frontRightVolts/DRIVE_VOLTAGE, backRightVolts/DRIVE_VOLTAGE);
}
//Drives left side motors at the voltage and right side motors at the same voltage
void driveLR(float leftPower, float rightPower){
  drive(leftPower, leftPower, rightPower, rightPower);
}

//Function that uses PWM to drive the motors
//Values should be from -1 to 1
//Positive values always drive the robot forward
void drive(float frontLeft, float backLeft , float frontRight, float backRight){

  //constrain drive PWM to hardware limit
  frontLeft = constrain(frontLeft, -1, 1);
  backLeft = constrain(backLeft, -1, 1);
  frontRight = constrain(frontRight, -1, 1);
  backRight = constrain(backRight, -1, 1);

  //store wheel directions based on if input is negative
  bool dirFL = frontLeft < 0;
  bool dirBL = backLeft > 0;
  bool dirFR = frontRight > 0;
  bool dirBR = backRight < 0;

  //drive motor
  digitalWrite(DIR1_L,dirFL);
  digitalWrite(DIR2_L,dirBL);
  digitalWrite(DIR1_R,dirFR);
  digitalWrite(DIR2_R,dirBR);
  //get max pwm based on bit resolution
  //if we increase resolution then the max pwm value increases
  uint maxPWM = pow(2,PWM_RESOLUTION) -1;

  ledcWrite(PWM1_L_CHANNEL, abs(frontLeft)*maxPWM);
  ledcWrite(PWM2_L_CHANNEL, abs(backLeft)*maxPWM);
  ledcWrite(PWM1_R_CHANNEL, abs(frontRight)*maxPWM);
  ledcWrite(PWM2_R_CHANNEL, abs(backRight)*maxPWM);

  //Serial.printf("FL: %d, BL: %d, FR: %d, BR: %d", abs(frontLeft)*maxPWM, abs(backLeft)*maxPWM, abs(frontRight)*maxPWM, abs(backRight)*maxPWM);
}