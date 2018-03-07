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
#define resetPin 6

//#define positionDeg 90  // Degrees to move
#define tol 2     // Tolerance for position
#define speedA 30
#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT 0
//#define ENCODER_LOWER_LIMTI 255
//#define ENCODER_UPPER_LIMTI 255

void encoderISRA1();  // Why do i need to add these???
void encoderISRA2();

volatile signed int encoderAPos = 0;
volatile signed int positionDeg = 90;

double Input, Output;
double Setpoint = 200;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

//PID forwardPID(&Input, &Output, &Setpoint, 0.0293064240007801, 0.320652760996062, 0.000504972877460821, DIRECT);
PID forwardPID(&Input, &Output, &Setpoint, 0.0377139597446175, 0.0885241838381296, 0.00073000882783854, DIRECT);
//0.2, 0.00001, 0.0004 


void setup() {
  Serial.begin(115200);

  pinMode(resetPin, INPUT);

  pinMode(encPinA1, INPUT);
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);

  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(2);
  // ki = ki * sampletime /1000;
  // kd = kd / sampletime /1000;

  forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  // Set initial motor direction and PWM
  //motorA.setPWM(speedA); 
  motorA.setPWM(0);
  //motorA.setDir(1);

}

// Dir(1) is CCW decrease ticks, Dir(2) is CW incr ticks

void loop(){
  int pwm;
  Input = encoderAPos;

  forwardPID.Compute();

//  if (encoderAPos < Setpoint){
//    motorA.setDir(2);
////    motorA.setPWM(Output);
//  }
//  else {//if (Output < 0){
//    motorA.setDir(1);
////    motorA.setPWM(-1*Output);
//  }
  motorA.setPWM(Output);
  Serial.print(Output);
  Serial.print("  ");

  Serial.println(motorA.encoderPos);
//
//  Serial.print("  ");
//  Serial.println(pwm);
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

//  if (encoderAPos > 399) {
//    encoderAPos = encoderAPos - 400;
//  } else if (encoderAPos < 0) {
//    encoderAPos = encoderAPos + 400;
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

// if (encoderAPos > 399) {
//    encoderAPos = encoderAPos - 400;
//  } else if (encoderAPos < 0) {
//    encoderAPos = encoderAPos + 400;
//  }

  motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, possible change later?

}

