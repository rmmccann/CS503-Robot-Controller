#include <QTRSensors.h>
#include <AFMotor.h>

#define LEFTINT 0  //interrupt 0 is on pin 2

volatile long count = 0;
AF_DCMotor left(1);
unsigned long lastT = 0;
unsigned long curT = 0;

void setup(){
  attachInterrupt(LEFTINT, leftcounter, RISING);
  Serial.begin(9600);
  left.run(FORWARD);
  left.setSpeed(255);
  delay(50);
  left.setSpeed(50);
}

void loop(){
  curT = millis();
  if(curT-lastT > 2000){  //print the count every 2 seconds
     Serial.println(count);
     lastT = curT; 
  }
}

void leftcounter(){
   count++;
}
