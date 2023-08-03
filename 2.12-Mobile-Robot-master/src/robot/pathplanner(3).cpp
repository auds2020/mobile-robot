#include <Arduino.h>
#include <Servo.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"

//IMU Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

float xrotIMU=0;

enum state{startForward, startLeftTurn, approachAED, alignWithAED, AEDMoveForward, AEDStop, AEDPickup, UR5Backward, stall, joyStickControl};
unsigned int robotstate;
unsigned int prestate = startForward;

float prestatedistancetraveled;


//wheel radius in meters
#define r 0.06
//distance from back wheel to center in meters
#define b 0.2

//holds the odometry data to be sent to the microcontroller
odometry_message odom_data;

float pathDistance = 0;
//x and y position of the robot in meters
float x = 0;
float y = 0;
float theta = 0;

float dPhiFL = 0;
float dPhiBL = 0;
float dPhiFR = 0;
float dPhiBR = 0;

//allows the intergral control to max contribution at the max drive voltage
//prevents integral windum
float maxSumError = (DRIVE_VOLTAGE/ki)/2;

unsigned long prevLoopTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

Servo servo1;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 140;
int servoPin = 13;

//Mannys work on trying to slowly ramp desired vel instead of step it with PID
//By adding an intermediate desiredfiltvelocity before reaching robot. Josephs advice
float filtDesiredVelFL = 0;
float filtDesiredVelBL = 0;
float filtDesiredVelFR = 0;
float filtDesiredVelBR = 0;

float alphafilt = .01;

void pickupAED();

void setDesiredVel(float vel, float k);
void updateRobotPose(float dPhiL, float dPhiR);
void getSetPointTrajectory();
void updateOdometry();
void printOdometry();

void setup(){
    Serial.begin(115200);
    encoderSetup();
    driveSetup();

    wirelessSetup();

    //IMU SETUP
    Serial.begin(115200);
    Serial.println("Orientation Sensor Test"); Serial.println("");
    
    /* Initialise the sensor */
    
    if (bno.begin()){
    }

    else if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(1000); 
    bno.setExtCrystalUse(true);
    //IMU SETUP

    servo1.attach(servoPin);

    robotstate = startForward;
}

void loop(){
    //IMU LOOP
    sensors_event_t event; 
    bno.getEvent(&event);
    xrotIMU=event.orientation.x;
    //IMU Loop
    servo1.write(pos);
    if (micros() - prevLoopTimeMicros > loopDelayMicros){
        prevLoopTimeMicros = micros();
        //get new encoder readings and update the velocity
        //also updates dPhi values for the change in angle of each motor
        updateVelocity(loopDelayMicros*1e-6);

        //dRad is the change in radians since the last reading of the encoders
        //just use the back left and back right encoders to calculate trajectory
        updateRobotPose(dPhiBL, dPhiBR);

        //sends odometry to the remote
        updateOdometry();
        sendOdometry();

        if(robotstate != joyStickControl){
            prestate = robotstate;
        }
        
        if(joyData.leftPressed){
            robotstate = joyStickControl;
        }

        if(joyData.rightPressed){
            robotstate = prestate;
        }


        getSetPointTrajectory();
        //getSetPointDriveTest();
        //getSetPointJoystick();

        //use first order alpha based filter to get filtered velocities
        filtDesiredVelFL = alphafilt*desiredVelFL + (1-alphafilt)*filtDesiredVelFL;
        filtDesiredVelBL = alphafilt*desiredVelBL + (1-alphafilt)*filtDesiredVelBL;
        filtDesiredVelFR = alphafilt*desiredVelFR + (1-alphafilt)*filtDesiredVelFR;
        filtDesiredVelBR = alphafilt*desiredVelBR + (1-alphafilt)*filtDesiredVelBR;

        //calculate error for each motor
        float newErrorFL = filtDesiredVelFL - filtVelFL;
        float newErrorBL = filtDesiredVelBL - filtVelBL;
        float newErrorFR = filtDesiredVelFR - filtVelFR;
        float newErrorBR = filtDesiredVelBR - filtVelBR;

        //get control signal by running PID on all for motors
        voltageFL = 0;      
        voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros*1e-6);
        voltageFR = 0;            
        voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros*1e-6);
        
        //only drive the back motors
        driveVolts(0, voltageBL, 0, voltageBR);
    }

    //put print statements here
    if (millis() - prevPrintTimeMillis > printDelayMillis){
        prevPrintTimeMillis = millis();
        printOdometry();
        //Serial.printf("Left Vel: %.2f Right Vel %.2f\n", filtVelBL, filtVelBR);
        //Serial.printf("dPhiBL: %.4f dPhiBR %.4f\n", dPhiBL, dPhiBR);
    }

}

//sets the desired velocity based on desired velocity vel in m/s
//and k curvature in 1/m representing 1/(radius of curvature)
void setDesiredVel(float vel, float k){
    //TODO convert the velocity and k curvature to new values for desiredVelBL and desiredVelBR
    desiredVelBL = vel*(1.0-b*k)/r;
    desiredVelFL = vel*(1.0-b*k)/r;
    desiredVelBR = vel*(1.0+b*k)/r;
    desiredVelFR = vel*(1.0+b*k)/r;
}

//makes robot follow a trajectory
void getSetPointTrajectory(){
    //default to not moving
    //velocity in m/s
    //k is 1/radius from center of rotation circle
    float vel = 0 , k = 0;
    int lth = 300;
    int hth = 800;
    //TODO Add trajectory planning by changing the value of vel and k
    //based on odemetry conditions
    switch(robotstate){
        case startForward:
            //Serial.println("State: startforward");
            desiredVelBL = 5;
            desiredVelFL = 0;
            desiredVelBR = 5;
            desiredVelFR = 0;
            if(x > .7){
                robotstate = startLeftTurn;
            }
            break;

        case startLeftTurn:
            Serial.println("State: startleft");
            setDesiredVel(.15, 3);
            if(abs(xrotIMU - 275) < .1){
                robotstate = approachAED;
                theta = xrotIMU;
            }
            break;

        case approachAED:
            Serial.println("State: apporachAED");
            desiredVelBL = 5;
            desiredVelFL = 0;
            desiredVelBR = 5;
            desiredVelFR = 0;
            if(y > .9){
                robotstate = alignWithAED;
            }
            break;

        case alignWithAED:
            Serial.println("State: alignWithAED");
            desiredVelBL = -1;
            desiredVelFL = 0;
            desiredVelBR = 1;
            desiredVelFR = 0;
            if(abs(xrotIMU - 187) < .1){
                robotstate = AEDMoveForward;
                prestatedistancetraveled = pathDistance;
                theta = xrotIMU;
            }
            break;

        case AEDMoveForward:
            Serial.println("State: AEDMoveForward");
            desiredVelBL = 2;
            desiredVelFL = 0;
            desiredVelBR = 2;
            desiredVelFR = 0;
            if(pathDistance - prestatedistancetraveled > .1){
                robotstate = AEDStop;
            }
            break;

        case AEDStop:
            Serial.println("State: AEDstop");
            desiredVelBL = 0;
            desiredVelFL = 0;
            desiredVelBR = 0;
            desiredVelFR = 0;
            robotstate = AEDPickup;
            break;

        case AEDPickup:
            Serial.println("State: AEDPickup");
            pickupAED();
            desiredVelBL = 0;
            desiredVelFL = 0;
            desiredVelBR = 0;
            desiredVelFR = 0;
            robotstate = UR5Backward;
            break;

        case UR5Backward:
            Serial.println("State: UR5Back");
            desiredVelBL = -5;
            desiredVelFL = 0;
            desiredVelBR = -5;
            desiredVelFR = 0;
            if(x > 2.0){
                robotstate = stall;
            }
            break;

        case stall:
            Serial.println("State: Stall");
            desiredVelBL = 0;
            desiredVelFL = 0;
            desiredVelBR = 0;
            desiredVelFR = 0;
            break;

        case joyStickControl:
            // up on x is low down on x is high
            // left on y is low righy on y is high 100 high 0 low for both

            Serial.println("State: joystickcontrol");
            
            if(joyData.joyX < lth && (joyData.joyY < hth && joyData.joyY > lth)){
                desiredVelBL = 4;
                desiredVelFL = 0;
                desiredVelBR = 4;
                desiredVelFR = 0;
            } else if (joyData.joyX > hth && (joyData.joyY < hth && joyData.joyY > lth)){
                desiredVelBL = -4;
                desiredVelFL = 0;
                desiredVelBR = -4;
                desiredVelFR = 0;
            } else if (joyData.joyY < lth && (joyData.joyX < hth && joyData.joyX > lth)){
                desiredVelBL = -1.5;
                desiredVelFL = 0;
                desiredVelBR = 1.5;
                desiredVelFR = 0;
            } else if (joyData.joyY > hth && (joyData.joyX < hth && joyData.joyX > lth)){
                desiredVelBL = 1.5;
                desiredVelFL = 0;
                desiredVelBR = -1.5;
                desiredVelFR = 0;
            } else if (joyData.downPressed){
                desiredVelBL = 0;
                desiredVelFL = 0;
                desiredVelBR = 0;
                desiredVelFR = 0;
            } else if (joyData.upPressed){
                pos = pos - 5;
                desiredVelBL = 0;
                desiredVelFL = 0;
                desiredVelBR = 0;
                desiredVelFR = 0;
            }
            break;

        default:
            Serial.println("State: default");
            desiredVelBL = 0;
            desiredVelFL = 0;
            desiredVelBR = 0;
            desiredVelFR = 0;
            break;
    }
}

//updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR){
    //TODO change in angle
    float dtheta = r/(2.0*b)*(dPhiR-dPhiL);
    //TODO update theta value
    theta += dtheta;
    //TODO use the equations from the handout to calculate the change in x and y
    float dx = r/2*(dPhiR*cos(theta)+dPhiL*cos(theta));
    float dy = r/2*(dPhiR*sin(theta)+dPhiL*sin(theta));
    //TODO update x and y positions
    x += dx;
    y += dy;
    //TODO update the pathDistance
    pathDistance += sqrt(dx*dx+dy*dy);
    //Serial.printf("x: %.2f y: %.2f\n", x, y);
}

//stores all the the latest odometry data into the odometry struct
void updateOdometry(){
    odom_data.millis = millis();
    odom_data.pathDistance = pathDistance;
    odom_data.x = x;
    odom_data.y = y;
    odom_data.theta = theta;
    odom_data.velL = filtVelBL;
    odom_data.velR = filtVelBR;
}
//prints current odometry to be read into MATLAB
void printOdometry(){
    //convert the time to seconds
    //Serial.printf("%.2f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", odom_data.millis/1000.0, odom_data.x, odom_data.y, odom_data.theta, odom_data.pathDistance, xrotIMU);
}

void pickupAED(void){
    for (pos = 140; pos >= 60; pos -= 1) { // goes from 180 degrees to 0 degrees
        servo1.write(pos);    // tell servo to go to position in variable 'pos'
        delay(15);             // waits 15ms for the servo to reach the position
    }
}