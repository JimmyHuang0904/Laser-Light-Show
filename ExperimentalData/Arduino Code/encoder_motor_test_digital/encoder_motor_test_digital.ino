//from http://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/
//http://playground.arduino.cc/Main/RotaryEncoders#Intro
//PWM from 0 to 255 for some reason

#define enable 9
#define dir1 7
#define dir2 1
#define pushSpd 5
#define pushDir 4
#define encPinA 3
#define encPinB 4

int encoderA = 0;
int encoderB = 0;

int count = 0;
bool blocked = false;

int rotDirection = 0;
int pressed = false;
int pressedSpd = false;
int speed = 5; 
int pwmOutput= 0; 

volatile unsigned int encoder0Pos = 0;

void setup() {
  Serial.begin(9600);  
  
  pinMode(encPinA, INPUT); 
  pinMode(encPinB, INPUT);
   
  pinMode(enable, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  
  pinMode(pushDir, INPUT);
  pinMode(pushSpd, INPUT);

   // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  // Set initial rotation direction
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
}

void loop() {

  encoderA = digitalRead(encPinA);
  encoderB = digitalRead(encPinB);

  //Serial.print(encoder0Pos);
  Serial.println(encoder0Pos, DEC);
  Serial.print("value: \n");
  //Serial.println(encoderA);
  //Serial.println(encoderB);

  //debounce and read speed button 
  if (digitalRead(pushSpd) == true) {
    pressedSpd = true;
  }
  while (digitalRead(pushSpd) == true);
  delay(20);
  if (pressedSpd == true) {
    if (speed == 5){
      speed = 0;
    }
    else {
      speed++; 
    }
  }
  pressedSpd = false; 
  //Serial.print("speed: \n");
  //Serial.print(speed); 
  
  pwmOutput = map(speed, 0, 20, 0 , 255); // Map the speed value from 0 to 255
  analogWrite(enable, pwmOutput); // Send PWM signal to L298N Enable pin
  
  // Read button - Debounce
  if (digitalRead(pushDir) == true) {
    pressed = !pressed;
  }
  while (digitalRead(pushDir) == true);
  delay(20);
  // If button is pressed - change rotation direction
  if (pressed == true  & rotDirection == 0) {
    Serial.print("Dir1 \n");
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    rotDirection = 1;
    delay(20);
  }
  // If button is pressed - change rotation direction
  if (pressed == false & rotDirection == 1) {
    Serial.print("dir2 \n"); 
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    rotDirection = 0;
    delay(20);
  }

}

void doEncoderA() {
  //Serial.print("asdf");
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
