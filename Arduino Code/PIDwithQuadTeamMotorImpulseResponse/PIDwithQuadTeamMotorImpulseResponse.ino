//Code to test quad decoder
//See last pages for code sample: 
//https://media.digikey.com/pdf/Data%20Sheets/Avago%20PDFs/HCTL-2032,2022.pdf
//http://www.ecsxtal.com/store/pdf/ecs_100.pdf

#include "Arduino.h"
#include "Motor.h"
#include <PID_v1.h>
/*
#define bit7 37
#define bit6 36
#define bit5 35
#define bit4 34
#define bit3 33
#define bit2 32
#define bit1 31
#define bit0 30

#define sel1Pin 26
#define OEPin 27

#define reset 4

// Slot detector
#define enablePinA 8
#define dirPinA1 11
#define dirPinA2 10
*/

#define bit7 53
#define bit6 52
#define bit5 51
#define bit4 50
#define bit3 49
#define bit2 48
#define bit1 47
#define bit0 46

// Sel and enable for 8 bit decoder
#define sel1Pin 42
#define OEPin 43

#define reset1 3

#define enablePinA 7
#define dirPinA1 6
#define dirPinA2 5
unsigned int result = 0;

int pwm =0;

Motor motorA(enablePinA, dirPinA1, dirPinA2);

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

  pinMode(sel1Pin, OUTPUT);
  pinMode(OEPin, OUTPUT);
  motorA.setPWM(255);
  motorA.setDir(2);
}

void loop() {

  Serial.print(get_Encoder());
  Serial.print(" ");
  Serial.println(micros());

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

  //result = result % 400;

  return result;
}


