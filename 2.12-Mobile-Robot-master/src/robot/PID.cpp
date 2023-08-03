#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "PID.h"

//instantaneous velocity of each wheel in radians per second
float velFL = 0;
float velBL = 0;
float velFR = 0;
float velBR = 0;

//filtered velocity of each wheel in radians per second
float filtVelFL = 0;
float filtVelBL = 0;
float filtVelFR = 0;
float filtVelBR = 0;

//scaling factor for each new reading
//if alpha = 0, each new reading is not even considered
//if alpha = 1, each new reading is the only thing considered
//lower values of alpha smooth the filtered velocity more, but delay the signal
float alpha = 0.05;

//sum errors for integral term
float sumErrorFL = 0;
float sumErrorBL = 0;
float sumErrorFR = 0;
float sumErrorBR = 0;

//desired velocity setpoints in rad/s
float desiredVelFL = 0;
float desiredVelBL = 0;
float desiredVelFR = 0;
float desiredVelBR = 0;

//voltage to send to the motors
float voltageFL = 0;
float voltageBL = 0;
float voltageFR = 0;
float voltageBR = 0;

//error reading
float errorFL = 0;
float errorBL = 0;
float errorFR = 0;
float errorBR = 0;

//PID Constants
float kp = 5;
float ki = 20;
float kd = 0;

float lastRadFL = 0;
float lastRadBL = 0;
float lastRadFR = 0;
float lastRadBR = 0;


void getSetPointDriveTest(float angVel){
    //make a 20 second loop
    unsigned long time = (millis()/1000)%20;
    if (0 <= time && time < 4){
        //forward
        //Serial.println("Forward");
        desiredVelFL = angVel;
        desiredVelBL = angVel;
        desiredVelFR = angVel;
        desiredVelBR = angVel;
    } else if (4 <= time && time < 8) {
        //backwards
        //Serial.println("Backwards");
        desiredVelFL = -angVel;
        desiredVelBL = -angVel;
        desiredVelFR = -angVel;
        desiredVelBR = -angVel;       
    } else if (8 <= time && time < 12) {
        //left
        //Serial.println("Left");
        desiredVelFL = -angVel;
        desiredVelBL = -angVel;
        desiredVelFR = angVel;
        desiredVelBR = angVel;
    } else if (12 <= time && time < 16){
        //right
        //Serial.println("Right");
        desiredVelFL = angVel;
        desiredVelBL = angVel;
        desiredVelFR = -angVel;
        desiredVelBR = -angVel;

    } else if (16 <= time  && time < 20){
        //stop
        //Serial.println("Stop");
        desiredVelFL = 0;
        desiredVelBL = 0;
        desiredVelFR = 0;
        desiredVelBR = 0;
    }

}

//updates the wheel setpoint based on the current joystick values
void getSetPointJoystick(){


}
//updates the filtered velocity values
//dt is the time in seconds since the last update
void updateVelocity(float dt){
    //store current positions to reference
    lastRadFL = encFLRad;
    lastRadBL = encBLRad;
    lastRadFR = encFRRad;
    lastRadBR = encBRRad;
    //get new positions
    readEncoders();
    //store the change in angle
    dPhiFL = encFLRad - lastRadFL;
    dPhiBL = encBLRad - lastRadBL;
    dPhiFR = encFRRad - lastRadFR;
    dPhiBR = encBRRad - lastRadBR;   

    //get (change in angle)/time
    velFL = dPhiFL/dt;
    velBL = dPhiBL/dt;
    velFR = dPhiFR/dt;
    velBR = dPhiBR/dt;
    //use first order alpha based filter to get filtered velocities
    filtVelFL = alpha*velFL + (1-alpha)*filtVelFR;
    filtVelBL = alpha*velBL + (1-alpha)*filtVelBL;
    filtVelFR = alpha*velFR + (1-alpha)*filtVelFR;
    filtVelBR = alpha*velBR + (1-alpha)*filtVelBR;
}




//returns the command signal for an academic PID loop
//sum_error will be update to maintain the cumulative error sum
//this requires tracking of variables outside of the PID function
float runPID(float error,float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime){
    sumError += error*loopTime;
    //avoid integral windum
    sumError = constrain(sumError, -maxSumError, maxSumError);
    //standard PID configuration
    float P = kp*error;
    float I = ki*sumError;
    float D = kd*(error-last_error)/loopTime;
    return P + I + D;
}