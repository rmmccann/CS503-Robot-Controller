/*
 The Wrecking Bot ™
 © Team 1
 */

#include <AFMotor.h>
#include <QTRSensors.h>
#include <PinChangeInt.h>
//#include <PCM.h>
//#include "sounddata.h"

int STATE = 0;
int LASTSTATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;

AF_DCMotor motorL(1);  //TODO just testing different frequencies; will be quieter, but what is the effcect on performance?
AF_DCMotor motorR(2);

int speedLstraight = 90;  //approximate values to go straight
int speedRstraight = 105;  //original 99
int speedL = speedLstraight;
int speedR = speedRstraight;

const int mmSideIdeal = 110;  //ideal dist to stay from wall ~11cm
int mmIdealMiss;

int mmFrontCur = 0;  //ping distance history; use two distance readings to know how non-parallel robot is and which way to turn to correct
int mmFrontLast = 0; //make sure to leave a little time between readings so enough distance change happens to be useful data

int mmSideCur = 0;   //+not too long or robot will be far off course
int mmSideLast = 0;

int frontDiff = 0;  //calculated immediately following ping readings for moving state
int sideDiff = 0;

const int maxAdjust = 120;  //maximum adjustment for re-aligning robot; turns quickly enough and won't stall
const int minAdjust = 10;
//idea is that the bigger diff in side pings, the more of an adjustment needs to be made, and vice versa
int adjustL;  //speed adjustment; function of diff between side ping readings
int adjustR;

int LEFT_INT = A0;
int RIGHT_INT = A1;

void setup()
{
	Serial.begin(9600);

	//startPlayback(sound_data, sizeof(sound_data));

	pinMode(A4, OUTPUT);
	pinMode(A5, OUTPUT);
	pinMode(A2, OUTPUT);
	pinMode(A3, OUTPUT);
	digitalWrite(A2, LOW);
	digitalWrite(A3, LOW);
	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

	motorR.setSpeed(0);
	motorR.run(RELEASE);
	motorL.setSpeed(0);
	motorL.run(RELEASE);

	digitalWrite(A5, HIGH);
	delay(250);
	digitalWrite(A5, LOW);
	digitalWrite(A3, HIGH);
	delay(250);
	digitalWrite(A5, HIGH);
	digitalWrite(A3, LOW);
	delay(250);
	digitalWrite(A5, LOW);

	PCintPort::attachInterrupt(LEFT_INT, &countLeft, CHANGE);
	PCintPort::attachInterrupt(RIGHT_INT, &countRight, CHANGE);

	Serial.println("Setup complete");
}

void loop()
{
	motorL.run(FORWARD);
	motorR.run(FORWARD);

	//LASTSTATE = STATE;  //keep history of state to make better judgements
	if((STATE == MOVING) && (mmSideCur-mmSideLast > 100) /*&& (mmSideLast < 300)*/){  //if it's moving and it sees a sudden increase in distance (10cm discontinuity) then assume needs to turn right
		//TODO add verification to turn right here; a few more samples, a delay, something. i dunno. sheeit.

		STATE = TURNING_RIGHT;
	}
	else if(mmFrontCur < 150){
		STATE = TURNING_LEFT;
	}
	else {
		STATE = MOVING;
	}
	//Then call the appropriate code for the current state

/*
	switch(STATE)
	{
		case MOVING:
				digitalWrite(A5, LOW);  //turn off cloudy LED
				digitalWrite(A3, LOW);  //turn off clear LED
			movingState();
			break;
		case TURNING_LEFT:
				digitalWrite(A3, HIGH);  //cloudy LED
			turningLeft();
			break;
		case TURNING_RIGHT:
				digitalWrite(A5, HIGH);  //clear LED
			turningRight();
			break;
		default:
			findWall(); //panic mode, bad news bears; will try and find something to follow again

	}
*/
	moveForward(0);
}

//global variables
int countL = 0;
int countR = 0;
int botDiameter = 76;
float sectorsPerMM = 1; //TODO: need to find the actual value
float acceptable_difference = 1; //TODO: need a logical value here
float radius_b = 457.2;
float radius_d = 304.8;
float radius_f = 304.8;
float length_b = 1436.2;
float length_d = 478.5;
float length_f = 957;
float arcRight = 0;
float arcLeft = 0;
float lengthStripe = 9.98;
float speedRatio = 0;
unsigned long currTimeL = 0;
unsigned long lastTimeL = 0;
unsigned long currTimeR = 0;
unsigned long lastTimeR = 0;
unsigned long angularVelocityDiffL = 0;
unsigned long angularVelocityDiffR = 0;
long velocityDiff = 0;

int mintmp = 0;
int maxtmp = 0;

void moveForward(float dist)
{
	// resetCounters();

/*
	if(getDistTravelled() >= dist)
	{
		//we're done. exit function
		return;
	}
*/

	//TODO: set motor speeds
	motorL.setSpeed(speedL);
	motorR.setSpeed(speedR);

	int mmIdealMiss;

	while(1)
	{
		velocityDiff = angularVelocityDiffL - angularVelocityDiffR;

                Serial.print("angularVelocityDiffL: " );
                Serial.println(angularVelocityDiffL);
                Serial.print("angularVelocityDiffR: " );
                Serial.println(angularVelocityDiffR);
//                Serial.println(velocityDiff);

                if(countL > 100){
                maxtmp = max(maxtmp, velocityDiff);
                mintmp = min(mintmp, velocityDiff);
                }
                //[-35,45]
//                Serial.println(countL);
//                Serial.print("min: ");
//                Serial.print(mintmp);
//                Serial.print(", max: ");
//                Serial.println(maxtmp);
//                Serial.
		if(velocityDiff != 0) //if diff in velocity is > 5ms
		{
                        Serial.print("velocityDiff ");
			Serial.println(velocityDiff);

			//Replace side diff with amount off
			if(velocityDiff < 0){  //cur-last < 0 means getting closer to wall, slow down left wheel
                                Serial.println("velocityDiff < 0");
				adjustL = map(-velocityDiff, 1, 8, minAdjust, maxAdjust);   //have to adjust in_max according to PINGTIME. shorter time between pings = less possible max distance traveled
				if(adjustL > maxAdjust) adjustL = maxAdjust;  //if sideDiff > in_max, adjustL could be set very high, so cap at 30 manually
//				speedL = speedLstraight - adjustL;
                                speedL -= adjustL;
				
				adjustR = map(-velocityDiff, 1, 8, minAdjust, maxAdjust);
				if(adjustR > maxAdjust) adjustR = maxAdjust;
//				speedR = speedRstraight + adjustR;
                                speedR += adjustR;
				
				motorL.setSpeed(speedL);
				motorR.setSpeed(speedR);  //less than ideal speed, speed back up to optimal
                                
                                Serial.print("adjustL: ");
                                Serial.print(adjustL);
                                Serial.print(" adjstR: ");
                                Serial.print(adjustR);
			}
			else if(velocityDiff > 0){
                                Serial.println("velocityDiff > 0");
				adjustR = map(velocityDiff, 1, 8, minAdjust, maxAdjust);  //in_max of 8 is from observational measurements
				if(adjustR > maxAdjust) adjustR = maxAdjust;
//				speedR = speedRstraight - adjustR;
                                speedR -= adjustR;
				
				adjustL = map(velocityDiff, 1, 8, minAdjust, maxAdjust);
				if(adjustL > maxAdjust) adjustL = maxAdjust;
//				speedL = speedLstraight + adjustL;
                                speedL += adjustL;
				
				motorR.setSpeed(speedR);
				motorL.setSpeed(speedL);

                                Serial.print("adjustL: ");
                                Serial.print(adjustL);
                                Serial.print(" adjstR: ");
                                Serial.print(adjustR);
			}
		}
	}
}
/*void turnLeft(float radius, int degree)
{
	resetCounters();

	//TODO: set motor speeds
        arcLeft = 0;
        arcRight = 0;
        SpeedRatio = 0;
       	resetCounters();
        if(degree==180)
        {
          arcRight = (radius + botDiameter/2)* 3.14;
          arcLeft = (radius - botDiameter/2)* 3.14;
          speedRatio = (arcLeft/arcRight);
        }
        
        if(degree==90)
        {
          arcRight = (radius + botDiameter/2)* 3.14/2;
          arcLeft = (radius - botDiameter/2)* 3.14/2;
          speedRatio = (arcLeft/arcRight);
        }  
        
	while(1)
	{
		//TODO
	}
}
void turnRight(float radius, int degree)
{
        resetcounters();
        arcLeft = 0;
        arcRight = 0;
        SpeedRatio = 0;
       	resetCounters();
        if(degree==180)
        {
          arcRight = (radius - botDiameter/2)* 3.14;
          arcLeft = (radius + botDiameter/2)* 3.14;
          speedRatio = (arcRight/arcLeft);
        }
        
        if(degree==90)
        {
          arcRight = (radius - botDiameter/2)* 3.14/2;
          arcLeft = (radius + botDiameter/2)* 3.14/2;
          speedRatio = (arcRight/arcLeft);
        }  
    
          
        
}
*/
void stopAt(float dist)
{
	//TODO
}

void resetCounters()
{
	countL = countR = 0;
}

//interupt handlers
void countLeft()
{
	++countL;
	lastTimeL = currTimeL;
	currTimeL = millis();
	angularVelocityDiffL = currTimeL - lastTimeL;
//        Serial.print("left ticks: ");
//        Serial.println(countL);

//        Serial.print("left: ");
//        Serial.print(angularVelocityDiffL);
//        Serial.print("right: ");
//        Serial.println(angularVelocityDiffR);
}
void countRight()
{
	++countR;
	lastTimeR = currTimeR;
	currTimeR = millis();
	angularVelocityDiffR = currTimeR - lastTimeR;
//        Serial.print("right ticks: ");
//        Serial.println(countR);
}

//float getLeftDistTravelled()
float getLeftDist()
{
	return countL/sectorsPerMM;
}
float getRightDist()
{
	return countR/sectorsPerMM;
}

float getDistTravelled()
{
	return (getLeftDist()/2 + getRightDist()/2);
}

/*
Check if we have gone far enough or reached a stop-at point
*/
/*
void checkDistance()
{
	//CHECK FOR STOP-ATS

	// float[] stopAts = new float[3];
	float[] stopAts = {1.0, 2.0, 3.0};
	for(int i=0; i<3; i++)
	{
		if(getDistTravelled() >= stopAts[i])
		{
			int delayTimeMS = 20000; // 20 seconds
			delay(delayTimeMS);
		}
	}

	//TODO: CHECK FOR WHEN TO CHANGE STATES
	
}
*/

/*
int x, x_last, delta_s, s_l, s_r, theta, delta_theta, R, theta_next, y, y_last

delta_s = (s_l+s_r)/2;
delta_theta = (s_r - s_l)/R;
theta_next = theta + delta_theta;
x = x_last + delta_s*cos(theta+(delta_theta/2));
y = y_last + delta_s*sin(theta+(delta_theta/2));
*/
