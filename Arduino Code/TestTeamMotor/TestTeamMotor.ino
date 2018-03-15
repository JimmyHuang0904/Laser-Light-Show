//from http://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/
//http://playground.arduino.cc/Main/RotaryEncoders#Intro
//PWM from 0 to 255 for some reason

#define enable 8
#define dir1 11
#define dir2 10
#define pushRst 6
#define pushSpd 5
#define pushDir 4
#define encPinA 3
#define encPinB 2

int encoderA = 0;
int encoderB = 0;

int count = 0;
bool blocked = false;

int rotDirection = 0;
int pressed = false;
int pressedSpd = false;
int pressedRst = false;

int speed = 0;
int pwmOutput= 0; 

volatile signed int encoder0Pos = 0;
volatile unsigned int curTime = 0;
volatile unsigned int prevTime = 0; 
volatile signed int Pos = 0; 

void setup() {
  Serial.begin(2000000);
  
  pinMode(encPinA, INPUT); 
  pinMode(encPinB, INPUT);
   
  pinMode(enable, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  
  pinMode(pushDir, INPUT);
  pinMode(pushSpd, INPUT);
  pinMode(pushRst, INPUT);

   // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);

  // Set initial rotation direction
  digitalWrite(dir2, LOW);
  digitalWrite(dir1, HIGH);

  analogWrite(enable, 80); // Send PWM signal to L298N Enable pin
  prevTime = 0; 
  pwmOutput = 180;
}

void loop() {
  
  curTime = millis();
  Serial.print(prevTime);

  digitalWrite(dir2, LOW);
  digitalWrite(dir1, HIGH);
  Serial.print("   ");
  if( (curTime - prevTime) > 3000){
    prevTime = curTime; 
    pwmOutput += 10; 
    digitalWrite(dir2, LOW);
    digitalWrite(dir1, LOW);
    delay(1000);
    
    if(pwmOutput > 250){
      pwmOutput = 180; 
      digitalWrite(dir2, LOW);
      digitalWrite(dir1, LOW);
      Serial.print("Reset \n");
    }
     
  }
 
  analogWrite(enable, pwmOutput);  
  Pos = encoder0Pos; 
  Serial.println(curTime);
  Serial.print("   "); 
  Serial.print(pwmOutput); 
  //Serial.print("   ");
  //Serial.println(Pos, DEC);

   

}

void doEncoderA() {

  // look for a low-to-high on channel A
  if (digitalRead(encPinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encPinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encPinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println(encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  
  // look for a low-to-high on channel B
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
}
