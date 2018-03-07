/*
  motorUtilities.ino
  Code that runs basic functions in the Motors class
  The two motors are refered to as Motor A and Motor B.
  For each motor, there is an Encoder 1 and Encoder 2. 
  Motor PWM range: 0 to 255
*/

#include "Arduino.h"
#include "Motor.h"

#define enablePinA	0
#define dirPinA1 4
#define dirPinA2 5
#define encPinA1 2
#define encPinA2 3

volatile signed int encoderAPos = 0;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

void setup() {
  Serial.begin(9600);

  pinMode(encPinA1, INPUT); 
  pinMode(encPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoderISRA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoderISRA2, CHANGE);

  // Set initial motor direction and PWM
  motorA.setPWM(200); 
  motorA.setDir(1);
  
}

void loop(){
  //motorA.encoderPos = encoderAPos; //update motor object inside ISR for now, but that possibly slows ISR. possible change later?
  Serial.print("\n encoder values: \n");
  Serial.print(encoderAPos, DEC);
  Serial.print(" \n"); 
  Serial.print(motorA.encoderPos, DEC); 

  //test set PWM
  motorA.setDir(1);
  motorA.setPWM(200); 
  delay(1000);
  motorA.setDir(2);
  motorA.setPWM(100);
  delay(1000);
  motorA.setDir(1);
  delay(1000);
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

