//from http://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/
//http://playground.arduino.cc/Main/RotaryEncoders#Intro
//PWM from 0 to 255 for some reason

//#define enable 9
//#define dir1 4
#define dir2 5
#define pushRst 6
//#define pushSpd 5
//#define pushDir 4
//#define encPinA 2
//#define encPinB 3

#define enablePinA 8
#define dirPinA1 11
#define dirPinA2 10
#define encPinA1 3
#define encPinA2 2

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
volatile double Time = 0; 
volatile signed int Pos = 0; 

void setup() {
  Serial.begin(57600);
  
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
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);

  analogWrite(enable, 255); // Send PWM signal to L298N Enable pin
}

void loop() {

  Time = micros();
  Pos = encoder0Pos; 
  Serial.print(Time);
  Serial.print("   ");
  Serial.println(Pos, DEC);


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
