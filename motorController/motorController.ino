/*
The Wrecking Bot ™
© Team 1
*/

#include <AFMotor.h>
#include <PCM.h>
#include "sounddata.h"

int STATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;

float speed = 80;

#define frontPing A0
#define rightPing A1

int ledPin = 2;

int followDistance = 11;
int tolerance = 4;
int goodTolerance = 2;
int minFollowDist = followDistance-tolerance;
int maxFollowDist = followDistance+tolerance;
int minGoodRange = followDistance-goodTolerance;
int maxGoodRange = followDistance+goodTolerance;

int minFrontDist = 15;

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

long curSpaceRight;
long curSpaceFront;

void setup()
{
	Serial.begin(9600);

	//startPlayback(sound_data, sizeof(sound_data));

	motorR.setSpeed(0);
	motorR.run(RELEASE);
	motorL.setSpeed(0);
	motorL.run(RELEASE);

	Serial.println("Setup complete");
}

void loop()
{
	//First take Ping sensor measurements
	long rightPingDuration = ping(rightPing);
	curSpaceRight = microsecondsToCentimeters(rightPingDuration);

	long frontPingDuration = ping(frontPing);
	curSpaceFront = microsecondsToCentimeters(frontPingDuration);

	//debug
	Serial.print("Ping front: ");
	Serial.print(curSpaceFront);
	Serial.print(", Ping right: ");
	Serial.println(curSpaceRight);

	//Then call the approapriate code for the current state
	switch(STATE)
	{
		case MOVING:
			movingState();
			break;
		case TURNING_LEFT:
			turningLeft();
			break;
		case TURNING_RIGHT:
			turningRight();
			break;
		default:
			findWall(); //uh-oh! this shouldn't happen, should it?
	}
}

long ping(int pingPin)
{
	pinMode(pingPin, OUTPUT);
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pingPin, LOW);

	pinMode(pingPin, INPUT);
	return pulseIn(pingPin, HIGH);
}

int adjust = 0;
void movingState()
{
	int speed_diff = 20;
	int small_speed_diff = 10;

	motorL.setSpeed(speed);
	motorR.setSpeed(speed);
	motorL.run(FORWARD);
	motorR.run(FORWARD);

	//Correct speed if distance is too close/far
//	if(curSpaceRight < minFollowDist)
//	{
//		motorL.setSpeed(0);
//		// motorL.setSpeed(15);
//		// STATE = TURNING_LEFT;
//	}
//	else if(curSpaceRight > maxFollowDist)
//	{
//		motorR.setSpeed(0);
//		// motorR.setSpeed(15);
//		// STATE = TURNING_RIGHT;
//	}

        if(curSpaceRight < maxGoodRange && curSpaceRight > minGoodRange)
        {
                if(adjust <= 0) adjust = 0;
                adjust-=3;
//                adjust = 0;
        }

	else if(curSpaceRight < minFollowDist+6)
	{
		//motorL.setSpeed(speed-small_speed_diff);
                motorL.setSpeed(speed-(adjust+=20));
                  
	}
	else if(curSpaceRight > maxGoodRange)
	{
		//motorR.setSpeed(speed-small_speed_diff);
		motorR.setSpeed(speed-(adjust+=3));
	}
	
//	else if(curSpaceRight < minFollowDist)
//	{
//		motorL.setSpeed(speed-speed_diff);
//	}
//	else if(curSpaceRight > maxFollowDist)
//	{
//		motorR.setSpeed(speed-speed_diff);
//	}

	//Check if we need to turn and change the state
	if(curSpaceFront < minFrontDist && curSpaceRight > minFollowDist && curSpaceRight < maxFollowDist)
	{
		STATE = TURNING_LEFT;
	}
	else if(curSpaceRight > maxGoodRange)
	{
		STATE = TURNING_RIGHT;
	}
}
void turningLeft()
{
	if(curSpaceFront < minFrontDist)
	{
		motorR.setSpeed(speed);
		motorL.setSpeed(speed);
		motorR.run(FORWARD);
		motorL.run(BACKWARD);
	}
	if(curSpaceFront > minFrontDist+5 && curSpaceRight < maxGoodRange) //TODO work on this condition (should wait a little more)
	{
		STATE = MOVING;
	}
}
void turningRight()
{
  /*
		//Pre-programed turning routine
		motorL.setSpeed(speed);
		motorR.setSpeed(speed/4);
		motorL.run(FORWARD);
		motorR.run(FORWARD);
		delay(2000); //turn for 2sec
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.run(FORWARD);
		motorR.run(FORWARD);
		delay(1000); //move forward for 1sec (so the ping will be able to detect the wall)
		STATE = MOVING;
*/

                //Turn in place
		
		motorL.setSpeed(speed);
		motorR.setSpeed(speed);
		motorL.run(FORWARD);
		motorR.run(FORWARD);
		delay(800);
		motorL.run(FORWARD);
		motorR.run(BACKWARD);
		delay(750);
		motorL.run(FORWARD);
		motorR.run(FORWARD);
		delay(900);
		STATE = MOVING;
		
//STATE = MOVING;
/*
	if(curSpaceRight > maxFollowDist+20)
	{
		motorL.setSpeed(speed);
		motorR.setSpeed(speed/4);
		motorL.run(FORWARD);
		motorR.run(FORWARD);
	}
	if(curSpaceRight < followDistance)
	{
		STATE = MOVING;
	}
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
