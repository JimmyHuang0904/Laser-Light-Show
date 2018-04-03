//Code to test quad decoder
//See last pages for code sample: 
//https://media.digikey.com/pdf/Data%20Sheets/Avago%20PDFs/HCTL-2032,2022.pdf
//http://www.ecsxtal.com/store/pdf/ecs_100.pdf

#include "Arduino.h"
#include "Motor.h"
#include "global_vars.h"

#include <PID_v1.h>

/*  PID Declarations  */
/*    Bottom Motor    */
double Input, Output;
double Setpoint;

int pwm = 0;
int stopFlag = 0;
int dir1Flag = 0;
int dir2Flag = 0;

/*     Top Motor      */
double Input1, Output1;
double Setpoint1;

int pwm1 = 0;
int stopFlag1 = 0;
int dir1Flag1 = 0;
int dir2Flag1 = 0;

// Timer
unsigned long timer =0;
unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned int SampleTime = 7;

const float pi = 3.1415926535;

Motor motorA(enablePinA_0, dirPinA1_0, dirPinA2_0);
Motor motorB(enablePinA_1, dirPinA1_1, dirPinA2_1);

// Bottom Motor PID
PID bottomPID(&Input, &Output, &Setpoint, 7, 0.002, 0.1, DIRECT);

// Top Motor PID
//PID topPID(&Input1, &Output1, &Setpoint1, 7, 0.002, 0.08, DIRECT);
//PID topPID(&Input1, &Output1, &Setpoint1, 7, 0.002, 1, DIRECT);
//PID topPID(&Input1, &Output1, &Setpoint1, 7000, 0.4, 10000, DIRECT); // SampleTime = 15, 15 samples per period
//PID topPID(&Input1, &Output1, &Setpoint1, 7000, 0.4, 12000, DIRECT); // SampleTime = 10, 15 samples per period
PID topPID(&Input1, &Output1, &Setpoint1, 90000, 100000, 10000, DIRECT); // SampleTime = 7, 15 samples per period

void setup() {
  Serial.begin(2000000);

  /*              */
  /* Bottom Motor */
  /*              */
  pinMode(bit0_7, INPUT);
  pinMode(bit0_6, INPUT);
  pinMode(bit0_5, INPUT);
  pinMode(bit0_4, INPUT);
  pinMode(bit0_3, INPUT);
  pinMode(bit0_2, INPUT);
  pinMode(bit0_1, INPUT);
  pinMode(bit0_0, INPUT);

  pinMode(sel1Pin_0, OUTPUT);
  pinMode(OEPin_0, OUTPUT);

  pinMode(reset0, INPUT);

  digitalWrite(OEPin_0, 0); //enable OE (set to 0)

  /*              */
  /*   Top Motor  */
  /*              */
  pinMode(bit1_7, INPUT);
  pinMode(bit1_6, INPUT);
  pinMode(bit1_5, INPUT);
  pinMode(bit1_4, INPUT);
  pinMode(bit1_3, INPUT);
  pinMode(bit1_2, INPUT);
  pinMode(bit1_1, INPUT);
  pinMode(bit1_0, INPUT);

  pinMode(sel1Pin_1, OUTPUT);
  pinMode(OEPin_1, OUTPUT);

  pinMode(reset1, INPUT);

  digitalWrite(OEPin_1, 0); //enable OE (set to 0)

  // PID
  bottomPID.SetMode(AUTOMATIC);
  bottomPID.SetSampleTime(1);
  bottomPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);
  bottomPID.nFilter = 1;

  // PID
  topPID.SetMode(AUTOMATIC);
  topPID.SetSampleTime(1);
  topPID.SetOutputLimits(PID_LOWER_LIMIT_1, PID_UPPER_LIMIT_1);
  topPID.nFilter = 1;

  initialize();
}

void loop() {

  Input  = get_Encoder0();
  Input1 = get_Encoder1();
  timer = millis();
  currentTime = timer - startTime;
  if( currentTime > SampleTime){
    startTime = millis();

    bottomPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    bottomPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    bottomPID.SetOutputLimits(PID_LOWER_LIMIT, PID_UPPER_LIMIT);  // Set the limits back to normal
    bottomPID.SetMode(MANUAL);
    bottomPID.SetMode(AUTOMATIC);

    Setpoint = cos(timer*pi/(SampleTime*15))*18 + 100;
    Setpoint1 = sin(timer*pi/(SampleTime*15))*18 + 60;
    
    Serial.print(Setpoint1);
    Serial.print(" ");
    Serial.println(Input1);
//    Serial.println(Output1);
  }

//  Serial.print("Bottom Motor Input: ");
//  Serial.print(Input);
//  Serial.print(" Top Motor Input: ");
//  Serial.println(Input1);

  bottomMotorLogic();
  topMotorLogic();
}

inline void bottomMotorLogic(void){
  bottomPID.Compute();

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
    pwm = map(Output, 0, PID_UPPER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    motorA.setPWM(pwm);
  } else if (Output < 0){
    if(dir1Flag == 0){
      motorA.setDir(1);
      dir1Flag=1;
      dir2Flag=0;
      stopFlag=0; 
    }
    pwm = map(Output, 0, PID_LOWER_LIMIT, MOTOR_LOWER_LIMIT, MOTOR_UPPER_LIMIT);
    motorA.setPWM(pwm);
  }
}

inline void topMotorLogic(void){
  topPID.Compute();

  if (Output1 == 0){
    dir1Flag1 = 0;
    dir2Flag1 = 0;
    motorB.setPWM(0);
  } else if (Output1 > 0){
    if(dir2Flag1 == 0){
      motorB.setDir(2);
      dir2Flag1 =1;
      dir1Flag1 =0;
      stopFlag1 =0;
      
    }
    pwm1 = map(Output1, 0, PID_UPPER_LIMIT_1, MOTOR_LOWER_LIMIT_1, MOTOR_UPPER_LIMIT_1);
    motorB.setPWM(pwm1);
  } else if (Output1 < 0){
    if(dir1Flag1 == 0){
      motorB.setDir(1);
      dir1Flag1 =1;
      dir2Flag1 =0;
      stopFlag1 =0; 
    }
    pwm1 = map(Output1, 0, PID_LOWER_LIMIT_1, MOTOR_LOWER_LIMIT_1, MOTOR_UPPER_LIMIT_1);
    motorB.setPWM(pwm1);

  }
}

inline int16_t get_Encoder0(void){
  int16_t result = 0;

  digitalWrite(sel1Pin_0,1); 
  bitWrite(result, 0, digitalRead(bit0_0));
  bitWrite(result, 1, digitalRead(bit0_1));
  bitWrite(result, 2, digitalRead(bit0_2));
  bitWrite(result, 3, digitalRead(bit0_3));
  bitWrite(result, 4, digitalRead(bit0_4));
  bitWrite(result, 5, digitalRead(bit0_5));
  bitWrite(result, 6, digitalRead(bit0_6));
  bitWrite(result, 7, digitalRead(bit0_7));

  digitalWrite(sel1Pin_0,0); 
  bitWrite(result, 8 , digitalRead(bit0_0));
  bitWrite(result, 9 , digitalRead(bit0_1));
  bitWrite(result, 10, digitalRead(bit0_2));
  bitWrite(result, 11, digitalRead(bit0_3));
  bitWrite(result, 12, digitalRead(bit0_4));
  bitWrite(result, 13, digitalRead(bit0_5));
  bitWrite(result, 14, digitalRead(bit0_6));
  bitWrite(result, 15, digitalRead(bit0_7));

//  Serial.print(result, BIN);
//  Serial.print(" ");
//  Serial.println(result, DEC);
  return result;
}

inline int16_t get_Encoder1(void){
  int16_t result = 0;

  digitalWrite(sel1Pin_1,1); 
  bitWrite(result, 0, digitalRead(bit1_0));
  bitWrite(result, 1, digitalRead(bit1_1));
  bitWrite(result, 2, digitalRead(bit1_2));
  bitWrite(result, 3, digitalRead(bit1_3));
  bitWrite(result, 4, digitalRead(bit1_4));
  bitWrite(result, 5, digitalRead(bit1_5));
  bitWrite(result, 6, digitalRead(bit1_6));
  bitWrite(result, 7, digitalRead(bit1_7));

  digitalWrite(sel1Pin_1,0); 
  bitWrite(result, 8 , digitalRead(bit1_0));
  bitWrite(result, 9 , digitalRead(bit1_1));
  bitWrite(result, 10, digitalRead(bit1_2));
  bitWrite(result, 11, digitalRead(bit1_3));
  bitWrite(result, 12, digitalRead(bit1_4));
  bitWrite(result, 13, digitalRead(bit1_5));
  bitWrite(result, 14, digitalRead(bit1_6));
  bitWrite(result, 15, digitalRead(bit1_7));

//  Serial.print(result, BIN);
//  Serial.print(" ");
//  Serial.println(result, DEC);
  return result;
}

void initialize(void){
  motorB.setPWM(0);

//  motorA.setDir(1);
//  motorA.setPWM(255);
//  while(digitalRead(reset0) == true){
//    delay(1);
//  }
//  motorA.setPWM(0);
//  Serial.println(get_Encoder0());

  motorB.setDir(1);
  motorB.setPWM(255);
  while(digitalRead(reset1) == true){
    delay(1);
  }
  motorB.setPWM(0);
  Serial.println(get_Encoder1());

//  while(true);
//    delay(20);
}




