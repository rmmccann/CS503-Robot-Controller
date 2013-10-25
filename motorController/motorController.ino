#include <AFMotor.h>

int STATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;

AF_DCMotor motorR(2, MOTOR12_64KHZ);
AF_DCMotor motorL(1);

void setup()
{
	Serial.begin(9600);
	motorR.setSpeed(0);
	motorR.run(RELEASE);
	motorL.setSpeed(0);
	motorL.run(RELEASE);
	Serial.println("Setup complete");
}

void loop()
{
	switch(STATE)
	{
		case 0:
			//MOVING
			movingState();
			break;
		case 1:
			turningLeft();
			break;
		case 2:
			turningRight();
			break;
		default:
			findWall();
	}
}

void movingState()
{
	/*
	left=movingspeed;
	right=movingspeed;

	if dist<=a
		left=0
	else if dist>b
		right=0
	else if dist<target
		left-=10;
	else if dist>target
		right-=10


	//if front < c && a<=right<=b then
	//	STATE=TURNING_LEFT;
	//else if front < c //not sure
	//	STATE=TURNING_RIGHT;
	*/
}
void turningLeft()
{
	/*
	float speed;
	if front_dist<c then
		rightmotor = speed; //forward
		leftmotor= -speed //backward
	if front_dist>c //TODO
		STATE=MOVING;
	*/
}
void turningRight()
{
	/*
	float speed;
	if right_dist>c then
		leftmotor = speed; //forward
		rightmotor= -speed //backward
	if right_dist<b
		STATE=MOVING;
	*/
}
void findWall()
{
	/*
	
	*/
}


void turnRightInPlace()
{

}
void turnLeftInPlace()
{

}

////////////////////
/*
minspeed L: 50
minspeed R: 54
*/
void setLSpeed(float rpm)
{
	//TODO
	//convert the rpm to [0-255]
	// int temp = 
	// motorL.setSpeed()
}
void setRSpeed(float rpm)
{
	//TODO
}
