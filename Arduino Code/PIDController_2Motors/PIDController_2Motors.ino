/*
  motorUtilities.ino
  Code that runs basic functions in the Motors class
  The two motors are refered to as Motor A and Motor B.
  For each motor, there is an Encoder 1 and Encoder 2. 
  Motor PWM range: 0 to 255
*/

#include "Arduino.h"
#include "Motor.h"
#include <PID_v1.h>

#define enablePinA 8
#define dirPinA1 11
#define dirPinA2 10
#define encPinA1 3
#define encPinA2 2

#define enablePinB 9
#define dirPinB1 21
#define dirPinB2 20
#define encPinB1 50
#define encPinB2 51

#define resetPin 6

//#define positionDeg 90  //degrees to move
#define tol 2     //tolerance for position
#define speedA 30
#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT -255
#define ENCODER_LOWER_LIMTI 0
#define ENCODER_UPPER_LIMTI 400

void encoderISRA1();  //why do i need to add these???
void encoderISRA2();
void encoderISRB1();  //why do i need to add these???
void encoderISRB2();

volatile signed int encoderAPos = 0;
volatile signed int positionADeg = 90;
int Apwm =0; 
double AencoderTicksDesired = 0;
int AstopFlag = 0;
int Adir1Flag = 0;
int Adir2Flag = 0; 

volatile signed int encoderBPos = 0;
volatile signed int positionBDeg = 90;
int Bpwm =0; 
double BencoderTicksDesired = 0;
int BstopFlag = 0;
int Bdir1Flag = 0;
int Bdir2Flag = 0; 

//double 

double AInput, AOutput, ASetpoint;
double BInput, BOutput, BSetpoint;

Motor motorA(enablePinA, dirPinA1, dirPinA2);
Motor motorB(enablePinB, dirPinB1, dirPinB2);


//PID AforwardPID(&AInput, &AOutput, &ASetpoint, 0.0101513210748192, 0.0537954476805063, 0.000476396049937738, DIRECT);
PID AforwardPID(&AInput, &AOutput, &ASetpoint, 0.2, 0.0000, 0.0004,  DIRECT);
//PID AforwardPID(&AInput, &AOutput, &ASetpoint, 0.3, 0, 0, DIRECT);

PID BforwardPID(&BInput, &BOutput, &BSetpoint, 0.2, 0.0000, 0.0004,  DIRECT);

void setup() {
  Serial.begin(115200);

  pinMode(encPinA1, INPUT);
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);
  AforwardPID.SetMode(AUTOMATIC);
  AforwardPID.SetSampleTime(1); 
  AforwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  // Set initial motor direction and PWM
  motorA.setPWM(0); 
  motorA.setDir(1);

  pinMode(encPinB1, INPUT);
  pinMode(encPinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinB1), encoderISRB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinB2), encoderISRB2, CHANGE);
  BforwardPID.SetMode(AUTOMATIC);
  BforwardPID.SetSampleTime(1); 
  BforwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  // Set initial motor direction and PWM
  motorB.setPWM(0); 
  motorB.setDir(1);
  
}

void loop(){
  ////Set inputs/outputs
  AencoderTicksDesired = 100;  
  ASetpoint = 100; 
  AInput = encoderAPos;

  AforwardPID.Compute();

  BencoderTicksDesired = 100;  
  BSetpoint = 100; 
  BInput = encoderBPos;
  
  BforwardPID.Compute();

  //////////Direction control
  if (AOutput == 0){
    if(AstopFlag == 0){
      //motorA.hardStop();
      AstopFlag = 1;
      Adir1Flag = 0;
      Adir2Flag = 0; 
    }
    Apwm = 0; 
  }
  
  //else if (Output > 0){
  if (AOutput > 0){
    if(Adir2Flag == 0){
      motorA.setDir(2);
      Adir2Flag=1;
      Adir1Flag=0;
      AstopFlag=0;
      
    }
    Apwm = AOutput;
    Apwm = map(AOutput, 0, PID_UPPER_LIMIT, 30, 200);
      //pwm = map(Output, 0, PID_UPPER_LIMIT, 30, 200);
      //or should this be map(Output, 30, PID_UPPER_LIMIT, 20, 200);
      //lower lim of output is 30???
  }
  else if (AOutput < 0){
    if(Adir1Flag == 0){
      motorA.setDir(1);
      Adir1Flag=1;
      Adir2Flag=0;
      AstopFlag=0; 
    }
    Apwm = -1*AOutput; 
    Apwm = map(AOutput, 0, PID_LOWER_LIMIT, 30, 200);
    //pwm = map(Output, 0, PID_LOWER_LIMIT, 20, 100);
  }
  motorA.setPWM(Apwm);

 // Serial.println(encoderAPos, DEC);

  //Serial.print("  ");
  //Serial.println(pwm);

  if (BOutput == 0){
    if(BstopFlag == 0){
      //motorB.hardStop();
      BstopFlag = 1;
      Bdir1Flag = 0;
      Bdir2Flag = 0; 
    }
    Bpwm = 0; 
  }
  
  //else if (Output > 0){
  if (BOutput > 0){
    if(Bdir2Flag == 0){
      motorB.setDir(2);
      Bdir2Flag=1;
      Bdir1Flag=0;
      BstopFlag=0;
      
    }
    Bpwm = BOutput;
    Bpwm = map(BOutput, 0, PID_UPPER_LIMIT, 30, 200);
      //pwm = map(Output, 0, PID_UPPER_LIMIT, 30, 200);
      //or should this be map(Output, 30, PID_UPPER_LIMIT, 20, 200);
      //lower lim of output is 30???
  }
  else if (BOutput < 0){
    if(Bdir1Flag == 0){
      motorB.setDir(1);
      Bdir1Flag=1;
      Bdir2Flag=0;
      BstopFlag=0; 
    }
    Bpwm = -1*BOutput; 
    Bpwm = map(BOutput, 0, PID_LOWER_LIMIT, 30, 200);
    //pwm = map(Output, 0, PID_LOWER_LIMIT, 20, 100);
  }
  motorB.setPWM(Bpwm);

//end main function
}

////Motor A ISRs
void encoderISRA1()
{
  
  // look for a low-to-high on channel A
  if (digitalRead(encPinA1) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encPinA2) == LOW) {
      encoderAPos = encoderAPos + 1;         // CW
    }
    else {
      encoderAPos = encoderAPos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinA2) == HIGH) {
      encoderAPos = encoderAPos + 1;          // CW
    }
    else {
      encoderAPos = encoderAPos - 1;          // CCW
    }
  }
  //Serial.println(encoder0Pos, DEC);
  // use for debugging - remember to comment out
//  if (encoderAPos == 400) {
//    encoderAPos = 0;
//  }
//  else if (encoderAPos == -1){
//    encoderAPos = 399;
//  }

  //motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, possible change later?

}

void encoderISRA2(){
  // look for a low-to-high on channel B
  if (digitalRead(encPinA2) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encPinA1) == HIGH) {
      encoderAPos = encoderAPos + 1;         // CW
    }
    else {
      encoderAPos = encoderAPos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinA1) == LOW) {
      encoderAPos = encoderAPos + 1;          // CW
    }
    else {
      encoderAPos = encoderAPos - 1;          // CCW
    }
  }
//
//  if (encoderAPos == 400) {
//    encoderAPos = 0;
//  }
//  else if (encoderAPos == -1){
//    encoderAPos = 399;
//  }
  motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, possible change later?

}

////Motor B ISRs
void encoderISRB1()
{
  
  // look for a low-to-high on channel A
  if (digitalRead(encPinB1) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encPinB2) == LOW) {
      encoderBPos = encoderBPos + 1;         // CW
    }
    else {
      encoderBPos = encoderBPos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinB2) == HIGH) {
      encoderBPos = encoderBPos + 1;          // CW
    }
    else {
      encoderBPos = encoderBPos - 1;          // CCW
    }
  }
  //Serial.println(encoder0Pos, DEC);
  // use for debugging - remember to comment out
//  if (encoderBPos == 400) {
//    encoderBPos = 0;
//  }
//  else if (encoderbPos == -1){
//    encoderBPos = 399;
//  }

  //motorB.encoderPos = encoderBPos; //update motor object inside ISR for now, possible change later?

}

void encoderISRB2(){
  // look for a low-to-high on channel B
  if (digitalRead(encPinB2) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encPinB1) == HIGH) {
      encoderBPos = encoderBPos + 1;         // CW
    }
    else {
      encoderBPos = encoderBPos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinB1) == LOW) {
      encoderBPos = encoderBPos + 1;          // CW
    }
    else {
      encoderBPos = encoderBPos - 1;          // CCW
    }
  }
//
//  if (encoderBPos == 400) {
//    encoderBPos = 0;
//  }
//  else if (encoderBPos == -1){
//    encoderBPos = 399;
//  }
  motorB.encoderPos = encoderBPos; //update motor object inside ISR for now, possible change later?

}


