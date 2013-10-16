#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(4);

float radius = 0;

void setup()
{
	motor1.setSpeed(100);
	motor2.setSpeed(100);
	motor1.run(RELEASE);
	motor2.run(RELEASE);
}

void loop()
{
	if(radius == 0)
	{
		motor1.setSpeed(100);
		motor1.run(FORWARD);

		motor2.setSpeed(100);
		motor2.run(BACKWARD);
	}

	/*
		radius == 1/2 width
		r = -l
		r>0
		l<0

		radius == width
		r>0
		l=0

		radius > width
		r>0
		l>0

	*/
}
