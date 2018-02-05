/*
  Motors.h 
  Contains encoder and motor code.
*/

#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class Motor
{
  public:
    Motor(int enable, int interupt);		//constructor function
	
	void doEncoder(void);
	void setPWM(int pwm);
	void setDir(int dir);
	void motorOn(void);
	void motorOff(void);
    //void dot();
    //void dash();
  private:
	volatile int encoder0pos; 
    //int _pin;
};

#endif