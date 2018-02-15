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

#define enablePinA	9
#define dirPinA1 4
#define dirPinA2 5
#define encPinA1 2
#define encPinA2 3

//#define positionDeg 90  //degrees to move
#define tol 2     //tolerance for position
#define speedA 30
#define PID_UPPER_LIMIT 1000
#define PID_LOWER_LIMIT -1000
#define ENCODER_LOWER_LIMTI 0
#define ENCODER_UPPER_LIMTI 400

volatile signed int encoderAPos = 0;
volatile signed int positionDeg = 90;

double Input, Output, Setpoint;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

PID forwardPID(&Input, &Output, &Setpoint, 0.0101513210748192, 0.0537954476805063, 0.000476396049937738, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(encPinA1, INPUT);
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);

  forwardPID.SetMode(AUTOMATIC);

  forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  // Set initial motor direction and PWM
  motorA.setPWM(speedA); 
  motorA.setDir(1);

}

void loop(){
  int pwm;
  double encoderTicksDesired = positionDeg*(2/1.8); 
  Setpoint = encoderTicksDesired;
  Input = encoderAPos;

  forwardPID.Compute();

  Serial.print(Output);
  Serial.print("  ");

  Serial.print(motorA.encoderPos);

  Serial.print("  ");
  Serial.println(pwm);

  if ((encoderAPos <= encoderTicksDesired + 2) && (encoderAPos >= encoderTicksDesired - 2)){
    motorA.hardStop();
    Serial.print("DONE");
  }
  else{
    if (Output > 0){
      motorA.setDir(2);
      pwm = map(Output, 0, PID_UPPER_LIMIT, 30, 200);
    }
    else if (Output < 0){
      motorA.setDir(1);
      pwm = map(Output, 0, PID_LOWER_LIMIT, 30, 200);
    }
    motorA.setPWM(pwm);
  }
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

  motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, possible change later?

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

