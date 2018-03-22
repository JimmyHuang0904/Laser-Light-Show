//Code to test quad decoder
//See last pages for code sample: 
//https://media.digikey.com/pdf/Data%20Sheets/Avago%20PDFs/HCTL-2032,2022.pdf
//http://www.ecsxtal.com/store/pdf/ecs_100.pdf

#include "Arduino.h"

#define bit7 53
#define bit6 52
#define bit5 51
#define bit4 50
#define bit3 49
#define bit2 48
#define bit1 47
#define bit0 46

//#define resetPin 41   //have set to always high right now (never reset)
#define sel1Pin 42
#define OEPin 43

byte thirdByte = 0;
byte fourthByte = 0; 
unsigned int result = 0;
//unsigned int current_data = 0;
//unsigned int second_data = 0;
//example: unsigned int b = B0000101;
//int is 16 bits = 2 bytes
//bitWrite(x, n, b) 

void setup() {
  Serial.begin(115200);

  pinMode(bit7, INPUT);
  pinMode(bit6, INPUT);
  pinMode(bit5, INPUT);
  pinMode(bit4, INPUT);
  pinMode(bit3, INPUT);
  pinMode(bit2, INPUT);
  pinMode(bit1, INPUT);
  pinMode(bit0, INPUT);

  //pinMode(resetPin, OUTPUT);
  pinMode(sel1Pin, OUTPUT);
  pinMode(OEPin, OUTPUT);
}

void loop() {
  result = 0;
  Serial.print("loop \n"); 
  
  digitalWrite(OEPin, 1); //disable OE (active low, so set to 1)
  delay(25);

  digitalWrite(OEPin, 0); //enable OE (set to 0)
  //get 3rd byte sel=0,0
  digitalWrite(sel1Pin,0);
  thirdByte = get3rdByte();

  //get last/LSB bye sel=1,0
  digitalWrite(sel1Pin,1); 
  fourthByte = get4thByte();

  digitalWrite(OEPin, 1); //disable OE
  delay(25);
  
  unsigned int Mult = 1;
  //Assign LSB
  unsigned int Temp = fourthByte*Mult;
  result = Temp;
  Mult = Mult*256;
  //Assign 3rd byte
  Temp = thirdByte*Mult;
  result = result+Temp;
  Mult = Mult*256;
  
  Serial.print(result, BIN);
  Serial.print("    ");
  Serial.print(result, DEC); 
  Serial.print("\n");  
 
}

byte get3rdByte(void){
  byte old_data = 0;
  byte new_data = 0;
  byte result = 0; 

  //get current data
  bitWrite(old_data, 0, digitalRead(bit0));
  bitWrite(old_data, 1, digitalRead(bit1));
  bitWrite(old_data, 2, digitalRead(bit2));
  bitWrite(old_data, 3, digitalRead(bit3));
  bitWrite(old_data, 4, digitalRead(bit4));
  bitWrite(old_data, 5, digitalRead(bit5));
  bitWrite(old_data, 6, digitalRead(bit6));
  bitWrite(old_data, 7, digitalRead(bit7));

  //get 2nd data
  bitWrite(new_data, 0, digitalRead(bit0));
  bitWrite(new_data, 1, digitalRead(bit1));
  bitWrite(new_data, 2, digitalRead(bit2));
  bitWrite(new_data, 3, digitalRead(bit3));
  bitWrite(new_data, 4, digitalRead(bit4));
  bitWrite(new_data, 5, digitalRead(bit5));
  bitWrite(new_data, 6, digitalRead(bit6));
  bitWrite(new_data, 7, digitalRead(bit7));

  if(old_data == new_data){
    result = new_data; //get stable data
    //Serial.print("get 3rd: \n");
    //Serial.print(result, DEC); 
    //Serial.write(result);
    return result;
  }
  else{
    Serial.print("error 3rd byte \n");
    get3rdByte(); 
  }
  
}

byte get4thByte(void){
  byte old_data = 0;
  byte new_data = 0;
  byte result = 0; 

  //get current data
  bitWrite(old_data, 0, digitalRead(bit0));
  bitWrite(old_data, 1, digitalRead(bit1));
  bitWrite(old_data, 2, digitalRead(bit2));
  bitWrite(old_data, 3, digitalRead(bit3));
  bitWrite(old_data, 4, digitalRead(bit4));
  bitWrite(old_data, 5, digitalRead(bit5));
  bitWrite(old_data, 6, digitalRead(bit6));
  bitWrite(old_data, 7, digitalRead(bit7));

  //get 2nd data
  bitWrite(new_data, 0, digitalRead(bit0));
  bitWrite(new_data, 1, digitalRead(bit1));
  bitWrite(new_data, 2, digitalRead(bit2));
  bitWrite(new_data, 3, digitalRead(bit3));
  bitWrite(new_data, 4, digitalRead(bit4));
  bitWrite(new_data, 5, digitalRead(bit5));
  bitWrite(new_data, 6, digitalRead(bit6));
  bitWrite(new_data, 7, digitalRead(bit7));

  if(old_data == new_data){
    result = new_data; //get stable data
    return result;
  }
  else{
    Serial.print("error 4th byte \n");
    get4thByte(); 
  }
  
}



