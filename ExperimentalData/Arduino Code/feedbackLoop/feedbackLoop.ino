/*
  motorUtilities.ino
  Code that runs basic functions in the Motors class
  The two motors are refered to as Motor A and Motor B.
  For each motor, there is an Encoder 1 and Encoder 2. 
  Motor PWM range: 0 to 255
*/

#include "Arduino.h"
#include "Motor.h"

#define enablePinA	9
#define dirPinA1 4
#define dirPinA2 5
#define encPinA1 2
#define encPinA2 3

#define positionDeg 90  //degrees to move
#define tol 2     //tolerance for position
#define speedA 30   //pwm for motor

volatile signed int encoderAPos = 0;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

void setup() {
  Serial.begin(9600);

  pinMode(encPinA1, INPUT); 
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);

  // Set initial motor direction and PWM
  motorA.setPWM(speedA); 
  motorA.setDir(1);
  
}

void loop(){
  //motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, but that possibly slows ISR. possible change later?
//  Serial.print("\n encoder values: \n");
//  Serial.print(encoderAPos, DEC);
//  Serial.print(" \n"); 
//  Serial.print(motorA.encoderPos, DEC); 

  double encoderTicks = positionDeg*(2/1.8); 

  Serial.println(motorA.encoderPos);

  if( (encoderTicks - tol) <= encoderAPos && encoderAPos <= (encoderTicks +tol)){
    //motorA.setPWM(0);
    motorA.hardStop(); 
    Serial.print("Reached position");
  }
  else if( encoderAPos <= encoderTicks){
    motorA.setDir(2);
    motorA.setPWM(speedA);
   //Serial.print("Setdir1");
  }
  else if( encoderAPos >= encoderTicks){
    motorA.setDir(1);
    motorA.setPWM(speedA);
   //Serial.print("Setdir2");
  }
  else{
    Serial.print("Error: unexpected position reading");
    motorA.setPWM(0); 
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

  motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, possible change later?

}

