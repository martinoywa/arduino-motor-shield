#include <Motor.h>
#include <SoftwareSerial.h>


 char c;
 int RXPin = 52;
 int TXPin = 51;
 
 SoftwareSerial serial(RXPin, TXPin);

// initialize the motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int maxSpeed = 255;


// forward
void moveForward() {

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
}

// backward
void moveBackward() {

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  
}

//right
void turnRight() {

  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  
}

//left
void turnLeft() {

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  
}

// stop
void stopBot() {

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  
}

void setup() {
  motor1.setSpeed(maxSpeed);
  motor2.setSpeed(maxSpeed);
  motor3.setSpeed(maxSpeed);
  motor4.setSpeed(maxSpeed);
  // begin communication with Bluetooth module
  Serial.begin(9600);
  serial.begin(9600);
}

void loop() {
  /* 
   *  Read incoming characters if serial communication
   * is available.
   */
  while(serial.available()> 0)
    {
    c = serial.read();
    Serial.println(c);
    if (c == 'f') {
    moveForward();
  }
  if (c == 'b') {
    moveBackward();
  }
  if (c == 's') {
   stopBot();
   }

   if (c == 'r') {
    turnRight();
  }
  if (c == 'l') {
    turnLeft();
  }
    }
}
