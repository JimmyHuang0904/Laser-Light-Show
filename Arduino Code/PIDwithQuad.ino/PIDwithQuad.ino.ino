//Code to test quad decoder
//See last pages for code sample: 
//https://media.digikey.com/pdf/Data%20Sheets/Avago%20PDFs/HCTL-2032,2022.pdf
//http://www.ecsxtal.com/store/pdf/ecs_100.pdf

#include "Arduino.h"
#include "Motor.h"
#include <PID_v1.h>

//typedef unsigned char      uint8_t;
//typedef unsigned short     uint16_t;
//typedef unsigned long      uint32_t;
//typedef unsigned long long uint64_t;

#define bit7 53
#define bit6 52
#define bit5 51
#define bit4 50
#define bit3 49
#define bit2 48
#define bit1 47
#define bit0 46

#define enablePinA 8
#define dirPinA1 11
#define dirPinA2 10

//#define resetPin 41   //have set to always high right now (never reset)
#define sel1Pin 42
#define OEPin 43

#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT -255

byte thirdByte = 0;
byte fourthByte = 0; 
unsigned int result = 0;

// PID Values
double Input, Output;
double Setpoint = 100;

int pwm =0;
int stopFlag = 0;
int dir1Flag = 0;
int dir2Flag = 0;

// Timer
unsigned long timer =0;
unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned int SampleTime = 75;

const float pi = 3.1415926535;

Motor motorA(enablePinA, dirPinA1, dirPinA2);
//int16_t get_Encoder(void);

//PID forwardPID(&Input, &Output, &Setpoint, 0.0268, 0, 0.0011, DIRECT);
PID forwardPID(&Input, &Output, &Setpoint, 0.4, 0.002, 0.02, DIRECT);

void setup() {
  result = 0;

  Serial.begin(2000000);

  digitalWrite(OEPin, 0); //enable OE (set to 0)

  pinMode(bit7, INPUT);
  pinMode(bit6, INPUT);
  pinMode(bit5, INPUT);
  pinMode(bit4, INPUT);
  pinMode(bit3, INPUT);
  pinMode(bit2, INPUT);
  pinMode(bit1, INPUT);
  pinMode(bit0, INPUT);

  //pinMode(resetPin, INPUT);
  pinMode(sel1Pin, OUTPUT);
  pinMode(OEPin, OUTPUT);

  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(2);
  forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);

  motorA.setPWM(0); 
  motorA.setDir(1);
}

void loop() {

  Input = get_Encoder();
  timer = millis();
  currentTime = timer - startTime;
  if( currentTime > SampleTime){
    startTime = millis();

    forwardPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    forwardPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    forwardPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);  // Set the limits back to normal
    forwardPID.SetMode(MANUAL);
    forwardPID.SetMode(AUTOMATIC);

    Setpoint = sin(timer*pi/(SampleTime*10))*50;
//    Serial.println(Setpoint);
    Serial.println(Input);
  }
//  Serial.println(Input);

  forwardPID.Compute();

  if (Output == 0){
    dir1Flag = 0;
    dir2Flag = 0;
    motorA.setPWM(0);
  } else if (Output > 0){
    if(dir2Flag == 0){
      motorA.setDir(2);
      dir2Flag=1;
      dir1Flag=0;
      stopFlag=0;
      
    }
    pwm = map(Output, 0, PID_UPPER_LIMIT, 30, 200);
  } else if (Output < 0){
    if(dir1Flag == 0){
      motorA.setDir(1);
      dir1Flag=1;
      dir2Flag=0;
      stopFlag=0; 
    }
    pwm = map(Output, 0, PID_LOWER_LIMIT, 30, 200);
  }
  motorA.setPWM(pwm);
}

inline int16_t get_Encoder(void){
  int16_t result = 0;

  digitalWrite(sel1Pin,1); 
  bitWrite(result, 0, digitalRead(bit0));
  bitWrite(result, 1, digitalRead(bit1));
  bitWrite(result, 2, digitalRead(bit2));
  bitWrite(result, 3, digitalRead(bit3));
  bitWrite(result, 4, digitalRead(bit4));
  bitWrite(result, 5, digitalRead(bit5));
  bitWrite(result, 6, digitalRead(bit6));
  bitWrite(result, 7, digitalRead(bit7));

  digitalWrite(sel1Pin,0); 
  bitWrite(result, 8 , digitalRead(bit0));
  bitWrite(result, 9 , digitalRead(bit1));
  bitWrite(result, 10, digitalRead(bit2));
  bitWrite(result, 11, digitalRead(bit3));
  bitWrite(result, 12, digitalRead(bit4));
  bitWrite(result, 13, digitalRead(bit5));
  bitWrite(result, 14, digitalRead(bit6));
  bitWrite(result, 15, digitalRead(bit7));

  result = result % 400;

  return result;
}


