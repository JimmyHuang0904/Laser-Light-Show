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

#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT -255

void encoderISRA1();  //why do i need to add these???
void encoderISRA2();

volatile signed int encoderAPos = 0;
volatile double Time = 0;
volatile signed int Pos = 0;

unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned long timer =0; 

int pwm =0; 
int resetState = 0;
int stopFlag = 0;
int dir1Flag = 0;
int dir2Flag = 0; 
int switchFlag = 0;

double Input, Output, Setpoint;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

PID forwardPID(&Input, &Output, &Setpoint, 0.2, 0.0000, 0.007,  DIRECT);
//PID forwardPID(&Input, &Output, &Setpoint, 0.0268, 0, 0.0011, DIRECT);  // SIMULINK 

void setup() {
  Serial.begin(2000000);

  pinMode(encPinA1, INPUT);
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);

  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(2); 
  forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  // Set initial motor direction and PWM
  Setpoint = 0;
  motorA.setDir(1);
  
}

void loop(){
  timer = millis();
  currentTime = timer - startTime;
  if( currentTime > 150){
    startTime = millis(); 

    forwardPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    forwardPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);  // Set the limits back to normal
    forwardPID.SetMode(MANUAL);
    forwardPID.SetMode(AUTOMATIC);
  
    stopFlag = 0;
    dir1Flag = 0;
    dir2Flag = 0;

    Setpoint = sin(timer)*50;
    Serial.println(encoderAPos, DEC);
//    Serial.println(sin(timer)*25);
//    if ( switchFlag == 0 ){
//      Setpoint -= 50;
//      switchFlag = 1;
//    }
//    else {
//      Setpoint += 50;
//      switchFlag = 0;
//    }
  }
  
  Input = encoderAPos;



  forwardPID.Compute();
  
  if (Output == 0){
    if(stopFlag == 0){
      //motorA.hardStop();
      stopFlag = 1;
      dir1Flag = 0;
      dir2Flag = 0; 
    }
    pwm = 0; 
  }
  
  if (Output > 0){
    if(dir2Flag == 0){
      motorA.setDir(2);
      dir2Flag=1;
      dir1Flag=0;
      stopFlag=0;
      
    }
    //pwm = Output;
    pwm = map(Output, 0, PID_UPPER_LIMIT, 30, 200);
  }
  else if (Output < 0){
    if(dir1Flag == 0){
      motorA.setDir(1);
      dir1Flag=1;
      dir2Flag=0;
      stopFlag=0; 
    }
    //pwm = -1*Output; 
    pwm = map(Output, 0, PID_LOWER_LIMIT, 30, 200);
  }
  motorA.setPWM(pwm);
//
//  Time = micros();
//  Pos = encoderAPos;
//  Serial.print(Time);
//  Serial.print("  ");
//  Serial.println(Pos);

}

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

