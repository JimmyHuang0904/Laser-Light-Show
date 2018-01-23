//from http://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/
//PWM from 0 to 255 for some reason

#define enable 9
#define dir1 6
#define dir2 2
#define pushSpd 3
#define pushDir 4
#define encPin A0

int encoder = 0;

int count = 0;
bool blocked = false;

int rotDirection = 0;
int pressed = false;
int pressedSpd = false;
int speed = 5; 
int pwmOutput= 0; 

void setup() {
  Serial.begin(9600);  
  
  pinMode(enable, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pushDir, INPUT);
  pinMode(pushSpd, INPUT);
  // Set initial rotation direction
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
}

void loop() {

  encoder = analogRead(encPin);

  //Serial.print("value: \n");
  Serial.println(encoder);

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

    // Logic for counting the posedge
  if( !encoder & blocked == false){
    Serial.println(count);
    blocked = true;
    count++;
  }

  if ( encoder ){
    blocked = false;
  }

}
