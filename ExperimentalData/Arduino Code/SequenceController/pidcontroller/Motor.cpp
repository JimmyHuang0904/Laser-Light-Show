/*
  Motors.cpp 
  Contains encoder and motor code.
  Datasheet for motor/current drive board: http://www.ece.ubc.ca/~eng-services/files/courses/elec391-spring2018/L298_Motor_Driver_Board_Datasheet.pdf
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int enablePin, int dirPin1, int dirPin2)
{
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  
  _enablePin = enablePin;
  _dirPin1 = dirPin1;
  _dirPin2 = dirPin2;
  
  hardStopFlag = 0; 
  //_pin = pin;
}

void Motor::setPWM(int pwm)
{
	if(hardStopFlag == 1){
		//Serial.print("Warning: Hard stop enabled, motor will not spin until setDir() is called \n"); 
	}
	
	analogWrite(_enablePin, pwm);
	
}

void Motor::setDir(int dir)
{
	hardStopFlag = 0; 
	
	if (dir == 1){
		digitalWrite(_dirPin1, HIGH);
		digitalWrite(_dirPin2, LOW);
	} 
	else if (dir == 2){
		digitalWrite(_dirPin1, LOW);
		digitalWrite(_dirPin2, HIGH);
	}
	else{
		Serial.print("Error: Invalid Motor Direction \n");
	}
	
}

void Motor::hardStop()
{
	hardStopFlag = 1; 
	digitalWrite(_dirPin1, LOW);
	digitalWrite(_dirPin2, LOW);
}

long Motor::getRotSpeed(int interval, unsigned long deg)
{
	/*unsigned long time0 = millis();
	pos0 = encoderPos;
	
	while( (millis() - time0) < interval);
	pos1 = encoderPos;
	
	long tickPerMilSec = (pos1 - pos2)/(interval);
	return (tickPerMilSec*deg*1000); 
	*/
}
