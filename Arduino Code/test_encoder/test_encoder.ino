//http://www.electroschematics.com/10494/arduino-optical-position-rotary-encoder/

#define encPin A0
int encoder = 0;

int count = 0;
bool blocked = false;

void setup() {
  Serial.begin(9600);  

}

void loop() {
  encoder = analogRead(encPin);

  Serial.print("value: \n");
  Serial.println(encoder);

  // Logic for counting the posedge
  if( !encoder & blocked == false){
    Serial.print(count);
    blocked = true;
    count++;
  }

  if ( encoder ){
    blocked = false;
  }

}
