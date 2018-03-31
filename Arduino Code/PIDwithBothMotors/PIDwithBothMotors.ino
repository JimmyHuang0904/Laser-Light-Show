//Code to test quad decoder
//See last pages for code sample: 
//https://media.digikey.com/pdf/Data%20Sheets/Avago%20PDFs/HCTL-2032,2022.pdf
//http://www.ecsxtal.com/store/pdf/ecs_100.pdf

#include "Arduino.h"
#include "Motor.h"
#include <PID_v1.h>

#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT -255
#define MOTOR_LOWER_LIMIT 220
#define MOTOR_UPPER_LIMIT 255

//typedef unsigned char      uint8_t;
//typedef unsigned short     uint16_t;
//typedef unsigned long      uint32_t;
//typedef unsigned long long uint64_t;

//A Definitions:
#define bit7A 53
#define bit6A 52
#define bit5A 51
#define bit4A 50
#define bit3A 49
#define bit2A 48
#define bit1A 47
#define bit0A 46

#define enablePinA 8
#define dirPinA1 11
#define dirPinA2 10

//#define resetPin 41   //have set to always high right now (never reset)
#define sel1PinA 42
#define OEPinA 43

//B Definitions:
#define bit7B 53
#define bit6B 52
#define bit5B 51
#define bit4B 50
#define bit3B 49
#define bit2B 48
#define bit1B 47
#define bit0B 46

#define enablePinB 8
#define dirPinB1 11
#define dirPinB2 10

//#define resetPin 41   //have set to always high right now (never reset)
#define sel1PinB 42
#define OEPinB 43

//A variables 
byte thirdByteA = 0;
byte fourthByteA = 0; 
unsigned int resultA = 0;

// PID Values
double InputA, OutputA;
double SetpointA = 100;

int pwmA =0;
int stopFlagA = 0;
int dir1FlagA = 0;
int dir2FlagA = 0;

//B variables 
byte thirdByteB = 0;
byte fourthByteB = 0; 
unsigned int resultB = 0;

// PID Values
double InputB, OutputB;
double SetpointB = 100;

int pwmB =0;
int stopFlagB = 0;
int dir1FlagB = 0;
int dir2FlagB = 0;

// Timer
unsigned long timer =0;
unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned int SampleTime = 10;
const float pi = 3.1415926535;

Motor motorA(enablePinA, dirPinA1, dirPinA2);
Motor motorB(enablePinB, dirPinB1, dirPinB2);
//int16_t get_EncoderA(void);

//PID forwardPID(&Input, &Output, &Setpoint, 0.0268, 0, 0.0011, DIRECT);
//PID forwardPID(&Input, &Output, &Setpoint, 4, 0.002, 0.05, DIRECT);
PID forwardPIDA(&InputA, &OutputA, &SetpointA, 4, 0.002, 0.05, DIRECT);
PID forwardPIDB(&InputB, &OutputB, &SetpointB, 4, 0.002, 0.05, DIRECT);

void setup() {
  Serial.begin(2000000);

  digitalWrite(OEPinA, 0); //enable OE (set to 0)

  pinMode(bit7A, INPUT);
  pinMode(bit6A, INPUT);
  pinMode(bit5A, INPUT);
  pinMode(bit4A, INPUT);
  pinMode(bit3A, INPUT);
  pinMode(bit2A, INPUT);
  pinMode(bit1A, INPUT);
  pinMode(bit0A, INPUT);

  //pinMode(resetPinA, INPUT);
  pinMode(sel1PinA, OUTPUT);
  pinMode(OEPinA, OUTPUT);

  forwardPIDA.SetMode(AUTOMATIC);
  forwardPIDA.SetSampleTime(2);
  forwardPIDA.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  forwardPIDA.nFilter = 1;

  motorA.setPWM(0); 
  motorA.setDir(1);
}

void loop() {

  InputA = get_EncoderA();
  InputB = get_EncoderB(); 
  
  //Snap to 90 code
  /*timer = millis();
  currentTime = timer - startTime; 
  if( currentTime > 2000){
    startTime = millis();
    forwardPIDA.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    forwardPIDA.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    forwardPIDA.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);  // Set the limits back to normal
    forwardPIDA.SetMode(MANUAL);
    forwardPIDA.SetMode(AUTOMATIC);

    SetpointA +=100;
    if(SetpointA >= 400){
      SetpointA = 0; 
    }
  }
  */
  
  //Sin wave code
  /*timer = millis();
  currentTime = timer - startTime;
  if( currentTime > SampleTime){
    startTime = millis();

    forwardPIDA.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    forwardPIDA.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    forwardPIDA.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);  // Set the limits back to normal
    forwardPIDA.SetMode(MANUAL);
    forwardPIDA.SetMode(AUTOMATIC);

    SetpointA = sin(timer*pi/(SampleTime*10))*50;
    Serial.print(SetpointA);
    Serial.print(" ");
    Serial.println(InputA);
  }
//  Serial.println(InputA);
  */

  //Motor A Logic
  forwardPIDA.Compute();

  if (OutputA == 0){
    dir1FlagA = 0;
    dir2FlagA = 0;
    motorA.setPWM(0);
  } else if (OutputA > 0){
    if(dir2FlagA == 0){
      motorA.setDir(2);
      dir2FlagA=1;
      dir1FlagA=0;
      stopFlagA=0;
      
    }
    pwmA = map(OutputA, 0, PID_UPPER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
  } else if (OutputA < 0){
    if(dir1FlagA == 0){
      motorA.setDir(1);
      dir1FlagA=1;
      dir2FlagA=0;
      stopFlagA=0; 
    }
    pwmA = map(OutputA, 0, PID_LOWER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
  }
  motorA.setPWM(pwmA);

  //Motor B Logic:
  forwardPIDB.Compute();

  if (OutputB == 0){
    dir1FlagB = 0;
    dir2FlagB = 0;
    motorB.setPWM(0);
  } else if (OutputB > 0){
    if(dir2FlagA == 0){
      motorB.setDir(2);
      dir2FlagB=1;
      dir1FlagB=0;
      stopFlagB=0;
      
    }
    pwmB = map(OutputB, 0, PID_UPPER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
  } else if (OutputB < 0){
    if(dir1FlagB == 0){
      motorB.setDir(1);
      dir1FlagB=1;
      dir2FlagB=0;
      stopFlagB=0; 
    }
    pwmB = map(OutputB, 0, PID_LOWER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
  }
  motorB.setPWM(pwmA);
}

inline int16_t get_EncoderA(void){
  int16_t result = 0;

  digitalWrite(sel1PinA,1); 
  bitWrite(result, 0, digitalRead(bit0A));
  bitWrite(result, 1, digitalRead(bit1A));
  bitWrite(result, 2, digitalRead(bit2A));
  bitWrite(result, 3, digitalRead(bit3A));
  bitWrite(result, 4, digitalRead(bit4A));
  bitWrite(result, 5, digitalRead(bit5A));
  bitWrite(result, 6, digitalRead(bit6A));
  bitWrite(result, 7, digitalRead(bit7A));

  digitalWrite(sel1PinA,0); 
  bitWrite(result, 8 , digitalRead(bit0A));
  bitWrite(result, 9 , digitalRead(bit1A));
  bitWrite(result, 10, digitalRead(bit2A));
  bitWrite(result, 11, digitalRead(bit3A));
  bitWrite(result, 12, digitalRead(bit4A));
  bitWrite(result, 13, digitalRead(bit5A));
  bitWrite(result, 14, digitalRead(bit6A));
  bitWrite(result, 15, digitalRead(bit7A));

  //result = result % 400;

  return result;
}

inline int16_t get_EncoderB(void){
  int16_t result = 0;

  digitalWrite(sel1PinB,1); 
  bitWrite(result, 0, digitalRead(bit0B));
  bitWrite(result, 1, digitalRead(bit1B));
  bitWrite(result, 2, digitalRead(bit2B));
  bitWrite(result, 3, digitalRead(bit3B));
  bitWrite(result, 4, digitalRead(bit4B));
  bitWrite(result, 5, digitalRead(bit5B));
  bitWrite(result, 6, digitalRead(bit6B));
  bitWrite(result, 7, digitalRead(bit7B));

  digitalWrite(sel1PinB,0); 
  bitWrite(result, 8 , digitalRead(bit0B));
  bitWrite(result, 9 , digitalRead(bit1B));
  bitWrite(result, 10, digitalRead(bit2B));
  bitWrite(result, 11, digitalRead(bit3B));
  bitWrite(result, 12, digitalRead(bit4B));
  bitWrite(result, 13, digitalRead(bit5B));
  bitWrite(result, 14, digitalRead(bit6B));
  bitWrite(result, 15, digitalRead(bit7B));

  //result = result % 400;

  return result;
}


