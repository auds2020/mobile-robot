#include <Arduino.h>
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

bool bnoOn;

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

enum state{align, initIMU, start, alignWithAED, AEDapproach, AEDGrab, alignWithUR5, approachUR5};
unsigned int robotstate;

void setup(){
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
    // //IMU SETUP
    Serial.begin(115200);
    Serial.println("Sensor Data"); Serial.println("");
    bnoOn = false;
    
    // /* Initialise the sensor */
    
    // if (bno.begin()){
    // }

    // else if(!bno.begin())
    // {
    //     /* There was a problem detecting the BNO055 ... check your connections */
    //     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //     while(1);
    // }
    
    // delay(1000); 
    // bno.setExtCrystalUse(true);
    // //IMU SETUP

    robotstate = align;
}

void loop(){
    //IMU LOOP
    sensors_event_t event; 
    bno.getEvent(&event);
    xrotIMU=event.orientation.x;
    //IMU Loop
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

        //uncomment the desired method for updating the PI setpoint 
        getSetPointTrajectory();
        //getSetPointDriveTest();
        //getSetPointJoystick();

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
    //TODO Add trajectory planning by changing the value of vel and k
    //based on odemetry conditions
    switch (robotstate){
        // Start by backing up against wall to align for consistant odometry data
        case align:
            Serial.println("In align state:");
            setDesiredVel(-0.02, 0);
            if(odom_data.x < -.05){
                setDesiredVel(0,0);
                robotstate = initIMU;
            }
            break;

        case initIMU:
            Serial.println("In initIMU state:");
            //Reset x and y odometry data
            x = 0;
            y = 0;
            theta = 0;
            //IMU SETUP
            /* Initialise the sensor */

            if (bno.begin()){
                bnoOn = true;
            }

            else if(!bno.begin())
            {
                /* There was a problem detecting the BNO055 ... check your connections */
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
                while(1);
            }

            delay(1000); 
            bno.setExtCrystalUse(true);
            //Move to next stage
            robotstate = start;
            break;

        //Move forward
        case start:
            Serial.println("In start state:");
            setDesiredVel(.02, 0);
            if(odom_data.x > .5){
                robotstate = alignWithAED;
            }
            break;
        // Need to now align with AED through 2 turns
        case alignWithAED:
            desiredVelBL = 0;
            desiredVelFL = 0;
            desiredVelBR = 0;
            desiredVelFR = 0;
            break;
    }

    // Old Code
    // if (pathDistance <= 20){
    //     //Back Up against wall to align
    //     if
    //     //STRAIGHT LINE FORWARD
    //     if (pathDistance <= 1){
    //         vel = 0.2;
    //         k = 0;
    //     }
    //     //left turn #1
    //     else if (pathDistance <= 1.5) { 
    //         vel=0.2;
    //         k=4;
    //     }
    //     else if (pathDistance <= 20 && pathDistance > 1.5){
    //         vel = 0.8;
    //         Serial.println("Velocity is 0.8");
    //         if (xrotIMU > 275){
    //             k = 2;
    //             Serial.println("Turning left");
    //         }
    //         else if (xrotIMU < 265){
    //             k = -4;
    //             Serial.println("Turning right");
    //         }
    //         else {
    //             k = 0;
    //         }
    //     }
    // } else {
    //     //STOP
    //     vel = 0;
    //     k = 0;
    // }
    // setDesiredVel(vel, k);
}

//updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR){
    //TODO change in angle
    float dtheta = r/(2.0*b)*(dPhiR-dPhiL);
    //TODO update theta value
    theta += dtheta;
    //TODO use the equations from the handout to calculate the change in x and y
    float dx = r/2*(dPhiR*cos(dtheta)+dPhiL*cos(dtheta));
    float dy = r/2*(dPhiR*sin(dtheta)+dPhiL*sin(dtheta));
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
    Serial.printf("%.2f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", odom_data.millis/1000.0, odom_data.x, odom_data.y, odom_data.theta, odom_data.pathDistance, xrotIMU);
}
