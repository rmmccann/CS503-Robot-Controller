// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

AF_DCMotor left(1);
AF_DCMotor right(2);

int count;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  // turn on motor
  left.setSpeed(255);  //starts moving at 54
  left.run(RELEASE);
  
  right.setSpeed(255);  //starts moving at 55
  right.run(RELEASE);
  
  /*
  pinMode(A2, OUTPUT);
  analogWrite(A2, 255);
  delay(500);
  analogWrite(A2, 0);  //one blink
  */
}

void loop() {
  left.run(FORWARD);
  right.run(FORWARD);
  left.setSpeed(242);
  right.setSpeed(255);
  delay(50);
  left.setSpeed(70);  //ramp up by going to lower speeds first
  right.setSpeed(83);
  delay(100);
  left.setSpeed(117);  //actual speed to go after initial boost
  right.setSpeed(130);  
  delay(2000);  //let it get up to speed with 
  
  /*
  analogWrite(A2, 255);
  delay(500);
  analogWrite(A2, 0);
  //after LED goes low again, slowdown code will immediately start
  */
  
  //works for stopping it when it's going left117 right130
  left.run(BACKWARD);
  right.run(BACKWARD);
  left.setSpeed(100);    //ramp down so it doesn't apply full 255 reverse torque right away
  right.setSpeed(100);
  delay(50);
  left.setSpeed(255);    //full reverse to apply max reverse torque -- might cause backward movement if robot was moving too slowly already
  right.setSpeed(255);
  delay(20);
  left.run(FORWARD);  //reset direction to forward
  right.run(FORWARD);
  
  left.setSpeed(0);  //shut motors off
  right.setSpeed(0);
  
  while(1){}  //stop the program by just going into infinite loop so it doesn't hurt itself
  
  
}
