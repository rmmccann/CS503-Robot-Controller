#include <AFMotor.h>

int STATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;

float speed = 10;

#define frontPing A0
#define rightPing A1

int followDistance = 4;
int tolerance = 1;
int minFollowDist = followDistance-tolerance;
int maxFollowDist = followDistance+tolerance;

int minFrontDist = 6;

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
	int speed_diff = 10;

	motorL.setSpeed(speed);
	motorR.setSpeed(speed);
	motorL.run(FORWARD);
	motorR.run(FORWARD);

	long duration, curSpaceRight, cm, duration2, curSpaceFront, cm2;

	//right ping
	pinMode(rightPing, OUTPUT);
	digitalWrite(rightPing, LOW);
	delayMicroseconds(2);
	digitalWrite(rightPing, HIGH);
	delayMicroseconds(5);
	digitalWrite(rightPing, LOW);

	pinMode(rightPing, INPUT);
	duration = pulseIn(rightPing, HIGH);

	//front ping
	pinMode(frontPing, OUTPUT);
	digitalWrite(frontPing, LOW);
	delayMicroseconds(2);
	digitalWrite(frontPing, HIGH);
	delayMicroseconds(5);
	digitalWrite(frontPing, LOW);

	pinMode(frontPing, INPUT);
	duration2 = pulseIn(frontPing, HIGH);

	curSpaceFront = microsecondsToInches(duration2);
	cm2 = microsecondsToCentimeters(duration2);

	//Correct speed if distance is too close/far
	if(curSpaceRight< followDistance-tolerance)
	{
		motorL.setSpeed(0);
	}
	else if(curSpaceRight> followDistance+tolerance)
	{
		motorR.setSpeed(0);
	}
	else if(curSpaceRight< followDistance)
	{
		motorL.setSpeed(speed-speed_diff);
	}
	else if(curSpaceRight> followDistance)
	{
		motorR.setSpeed(speed-speed_diff);
	}

	//Check if we need to turn and change the state
	if(curSpaceFront < minFrontDist && curSpaceRight > minFollowDist && curSpaceRight < maxFollowDist)
	{
		STATE = TURNING_LEFT;
	}
	else if(curSpaceFront < minFrontDist && curSpaceRight > maxFollowDist)
	{
		STATE = TURNING_RIGHT;
	}
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

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
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
