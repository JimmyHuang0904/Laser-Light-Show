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
    Motor(int enablePin, int dirPin1, int dirPin2);		//constructor function
	
	volatile signed int encoderPos; 
	
	void setPWM(int pwm);	//turns on motor according to pwm: min 0 to max 255. setting PWM to 0 = "free running stop"
	void setDir(int dir);	//changes direction of motor (dir=1 or dir=2). If hard stop has previously been set, setDir turns motor back on. 
	void hardStop(void);  	//"hard stop" of motor according to current drive datasheet. To turn motor back on, call setDir(). 
    long getRotSpeed(int interval, unsigned long deg); 	//return average deg/second over time interval (in milliseconds), assuming each tick is "deg" degrees
	
  private:
	int hardStopFlag; 
    int _enablePin;
	int _dirPin1;
	int _dirPin2; 
};

#endif