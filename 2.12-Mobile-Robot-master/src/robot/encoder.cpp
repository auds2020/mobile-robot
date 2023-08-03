#include <Arduino.h>
#include <SPI.h>
#include <Encoder_Buffer.h>
#include "encoder.h"

//Chip select pins
//In order to read each encoder separately, one of these
//pins is set to high at a time
#define CS1 4
#define CS2 16
#define CS3 17
#define CS4 21


//Pulses per revolution of the motor
//Differs based on gear ratio
//For the 312 RPM Motor 
#define ENC_PPR 537.7



//Encoder Objects for each motor
Encoder_Buffer EncoderFL(CS1);
Encoder_Buffer EncoderBL(CS2);
Encoder_Buffer EncoderFR(CS3); 
Encoder_Buffer EncoderBR(CS4);

// //Encoder Counts
long encFLCount = 0;
long encBLCount = 0;
long encFRCount = 0;
long encBRCount = 0;

float encFLRad = 0;
float encBLRad = 0;
float encFRRad = 0;
float encBRRad = 0;


//should be called once to initialize encoders
void encoderSetup() {
  //Starts communication protocol to communicate with the encoder breakout
  SPI.begin();
  //Initialize all the encoders
  EncoderFL.initEncoder();
  EncoderBL.initEncoder();
  EncoderFR.initEncoder();
  EncoderBR.initEncoder();
}
//Delay (in milliseconds) between consecutive reads of the encoders
//Prevents constantly

void readEncoders() {
  //check to see if anything changed

  //read all the encoders
  //note that the front right and back right encoders are flipped
  //this is to maintain movement forward as positive for both sides
  encFLCount = EncoderFL.readEncoder();
  encBLCount = EncoderBL.readEncoder();
  encFRCount = -EncoderFR.readEncoder();
  encBRCount = -EncoderBR.readEncoder();

  //store as radians
  encFLRad = encFLCount*2*PI/ENC_PPR;
  encBLRad = encBLCount*2*PI/ENC_PPR;
  encFRRad = encFRCount*2*PI/ENC_PPR;
  encBRRad = encBRCount*2*PI/ENC_PPR;
}

//Clears all of the encoder values 
void clearEncoders(){
  //Encoder Counts
  encFLCount = 0;
  encBLCount = 0;
  encFRCount = 0;
  encBRCount = 0;

  encFLRad = 0;
  encBLRad = 0;
  encFRRad = 0;
  encBRRad = 0;

  EncoderFL.clearEncoderCount();
  EncoderBL.clearEncoderCount();
  EncoderFR.clearEncoderCount();
  EncoderBR.clearEncoderCount();
}



