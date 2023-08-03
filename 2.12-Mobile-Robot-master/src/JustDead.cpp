#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"
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

void setDesiredVel(float vel, float k);
void updateRobotPose(float dPhiL, float dPhiR);
void getSetPointTrajectory();
void updateOdometry();
void printOdometry();

//States
enum{start, alignWithAED, moveToGrabAED, grabAED, moveToFaceUR5, startMovingTowardsUR5};
unsigned char robotstate = start;


void setup(){
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
}

void loop(){
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

        //Set trajectory based on state
        getSetPointTrajectory();

        //calculate error for each motor
        float newErrorFL = desiredVelFL - filtVelFL;
        float newErrorBL = desiredVelBL - filtVelBL;
        float newErrorFR = desiredVelFR - filtVelFR;
        float newErrorBR = desiredVelBR - filtVelBR;

        //get control signal by running PID on all for motors
        voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros*1e-6);      
        voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros*1e-6);
        voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros*1e-6);            
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

    desiredVelBL = vel*(1-b*k)/r;
    desiredVelFL = desiredVelFL;
    desiredVelBR = vel*(1+b*k)/r;
    desiredVelFR = desiredVelFR;
}

//makes robot follow a trajectory
void getSetPointTrajectory(){
    //default to not moving
    //velocity in m/s
    //k is 1/radius from center of rotation circle
    float vel = 0 , k = 0;
    //TODO Add trajectory planning by changing the value of vel and k
    //based on odemetry conditions
   
    switch(robotstate){
        case start:         // Want to move forward before turning to alighn with AED
            if(odom_data.x <= 1.5){
                vel = .3;   
                k = 0;
                setDesiredVel(vel, k);  //Just want it to go straight
            }else{
                robotstate = alignWithAED;
            }

        case alignWithAED:
            //Need it first to face 90 deg right
            if(odom_data.theta < 90){ // More or less doing a right turn with radius of .25m since we know that works
                desiredVelBL = vel*(1-b*k)/r;
                desiredVelFL = desiredVelFL;
                desiredVelBR = vel*(1+b*k)/r;
                desiredVelFR = desiredVelFR;
            } else if (/* condition */){
                /* code */
            }
            
        
    }

}

//updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR){
    //TODO change in angle
    float dtheta = r/(2*b)*(dPhiR - dPhiL);
    //TODO update theta value
    theta += dtheta;
    //TODO use the equations from the handout to calculate the change in x and y
    float dx = r/2*(cos(theta)*dPhiR + cos(theta)*dPhiL);
    float dy = r/2*(sin(theta)*dPhiR + sin(theta)*dPhiL);
    //TODO update x and y positions
    x += dx;
    y += dy;
    //TODO update the pathDistance
    pathDistance += sqrt(sq(dx) + sq(dy));
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
    Serial.printf("%.2f\t%.4f\t%.4f\t%.4f\t%.4f\n", odom_data.millis/1000.0, odom_data.x, odom_data.y, odom_data.theta, odom_data.pathDistance);
}
