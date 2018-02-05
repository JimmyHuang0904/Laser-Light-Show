/*
  Motors.cpp 
  Contains encoder and motor code.
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int enable, int interupt)
{
  pinMode(enable, OUTPUT);
  encoder0Pos = 0; 
  attachInterrupt(digitalPinToInterrupt(interupt), doEncoder, CHANGE);
  //_pin = pin;
}

void Motor::doEncoder()
{
	
	encoder0Pos++; 
  
  /*// look for a low-to-high on channel B
  if (digitalRead(encPinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encPinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  */
}

/*
void Morse::dash()
{
  digitalWrite(_pin, HIGH);
  delay(1000);
  digitalWrite(_pin, LOW);
  delay(250);
}
*/