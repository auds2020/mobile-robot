#include <Arduino.h>

#define DIR1_L 32
#define PWM1_L 14
#define DIR2_L A0
#define PWM2_L A1
#define DIR1_R 33
#define PWM1_R 15
#define DIR2_R 12
#define PWM2_R 27
#define MAX_DRIVE 255



void drive(int16_t frontLeft, int16_t backLeft , int16_t frontRight, int16_t backRight);
void driveLR(int16_t leftPower, int16_t rightPower);

void setup() {
  //make all motor driver pins outputs
  pinMode(DIR1_L, OUTPUT);
  pinMode(PWM1_L, OUTPUT);
  pinMode(DIR2_L, OUTPUT);
  pinMode(PWM2_L, OUTPUT);
  pinMode(DIR1_R, OUTPUT);
  pinMode(PWM1_R, OUTPUT);
  pinMode(DIR2_R, OUTPUT);
  pinMode(PWM2_R, OUTPUT);
}
//max motor power is 255
int8_t motorPower = 50;
void loop() {
  //drive forwards
  driveLR(motorPower, motorPower);
  delay(2000);
  //drive backwards
  driveLR(-motorPower, -motorPower);
  delay(2000);
  //turn left
  driveLR(-motorPower, motorPower);
  delay(2000);
  //turn right
  driveLR(motorPower,-motorPower);
  delay(2000);
  //stop
  driveLR(0,0);
  delay(8000);
}


//Drives left side motors at the voltage and right side motors at the same voltage
void driveLR(int16_t leftPower, int16_t rightPower){
  drive(leftPower, leftPower, rightPower, rightPower);
}

//Function that uses PWM to drive the motors
//Values should be from -255 to 255
//Positive values always drive the robot forward
//defaults to the 4 drive variables as input
void drive(int16_t frontLeft, int16_t backLeft , int16_t frontRight, int16_t backRight){

  //constrain drive PWM to hardware limit
  frontLeft = constrain(frontLeft, -MAX_DRIVE, MAX_DRIVE);
  backLeft = constrain(backLeft, -MAX_DRIVE, MAX_DRIVE);
  frontRight = constrain(frontRight, -MAX_DRIVE, MAX_DRIVE);
  backRight = constrain(backRight, -MAX_DRIVE, MAX_DRIVE);

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
  
  analogWrite(PWM1_L,abs(frontLeft));
  analogWrite(PWM2_L,abs(backLeft));
  analogWrite(PWM1_R,abs(frontRight));
  analogWrite(PWM2_R,abs(backRight));  
}