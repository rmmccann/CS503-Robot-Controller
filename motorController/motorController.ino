#include <AFMotor.h>

AF_DCMotor motorR(2);
AF_DCMotor motorL(1);
//AF_DCMotor motor2(4);
int count;

void setup()
{
	Serial.begin(9600);
	motorR.setSpeed(0);
	motorR.run(RELEASE);	//start not moving
	motorL.setSpeed(0);
	motorL.run(RELEASE);
	Serial.println("Setup complete");
}

void loop()
{
	//motorL.setSpeed(50);
	//motorL.run(FORWARD);
	/*
	motorR.run(FORWARD);
	for (count=0; count<256; count+=20){
	motorR.setSpeed(count);
	delay(1000);
	}
	*/
	motorR.setSpeed(255);
	motorR.run(FORWARD);
}
