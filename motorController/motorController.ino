/*
 The Wrecking Bot ™
 © Team 1
 */

#include <AFMotor.h>
//#include <PCM.h>
//#include "sounddata.h"

int STATE = 0;
int LASTSTATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;
const int STILL_RIGHT = 3;

float speed = 80;

#define FRONTPING A0
#define SIDEPING A1
#define PINGTIME 75  //ms between ping reading updates  //at 300ms and L/R speeds of 90/99, will go at most 3cm closer to wall in that time assuming it stays within 45deg of wall

AF_DCMotor motorL(1);  //TODO just testing different frequencies; will be quieter, but what is the effcect on performance?
AF_DCMotor motorR(2);

int speedLstraight = 90;  //approximate values to go straight
int speedRstraight = 99;
int speedL = speedLstraight;
int speedR = speedRstraight;

unsigned long lastTime = 0; //last time ping readings were taken; used to space out pings so that some distance will accumulate
unsigned long curTime = 0;

const int mmSideIdeal = 110;  //ideal dist to stay from wall ~11cm
const int mmSideTolerance = 20; //can be 2cm off either side before needing to adjust
int mmIdealMiss;

int mmFrontCur = 0;  //ping distance history; use two distance readings to know how non-parallel robot is and which way to turn to correct
int mmFrontCur2 = 0;
int mmFrontLast = 0; //make sure to leave a little time between readings so enough distance change happens to be useful data
int mmFrontLast2 = 0;

int mmSideCur = 0;   //+not too long or robot will be far off course
int mmSideCur2 = 0;
int mmSideLast = 0;
int mmSideLast2 = 0;

int frontDiff = 0;  //calculated immediately following ping readings for moving state
int sideDiff = 0;

const int maxAdjust = 80;  //maximum adjustment for re-aligning robot; turns quickly enough and won't stall
const int minAdjust = 10;
//idea is that the bigger diff in side pings, the more of an adjustment needs to be made, and vice versa
int adjustL;  //speed adjustment; function of diff between side ping readings
int adjustR;

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

	pingSide();
	delay(5);
	pingSide();
	delay(5);
	pingFront();
	delay(5);
	pingFront();
	delay(5);

	digitalWrite(A5, HIGH);
	delay(250);
	digitalWrite(A5, LOW);
	digitalWrite(A3, HIGH);
	delay(250);
	digitalWrite(A5, HIGH);
	digitalWrite(A3, LOW);
	delay(250);
	digitalWrite(A5, LOW);

	Serial.println("Setup complete");
}

void loop()
{
	motorL.run(FORWARD);
	motorR.run(FORWARD);

	curTime = millis();
	if(curTime - lastTime > PINGTIME) {
		pingSide();
		delay(5);
		pingFront();

		lastTime = curTime;
	//Serial.println("ping");
	}

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
	//make it smart, have several modes of trying to find wall given last state before losing wall
	}
}

void pingSide(){
	mmSideLast = mmSideCur;
	mmSideCur = sideMM();
	sideDiff = mmSideCur - mmSideLast;
}

void pingFront(){
	mmFrontLast = mmFrontCur;
	mmFrontCur = frontMM();
	//mmFrontCur = 400;
	frontDiff = mmFrontCur - mmFrontLast;
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
  return pulseIn(pingPin, HIGH);  //returns 
}

int targetDist = 120;
int goodOffset = 40;
int badOffset = 60;
int goodBadRange = badOffset - goodOffset;
int minGoodRange = targetDist - goodOffset;
int maxGoodRange = targetDist + goodOffset;
int minBadRange = targetDist - badOffset;
int maxBadRange = targetDist + badOffset;

void movingState()
{
 //get time it takes robot to get certain distance closer at max supported side reading angle (~40 deg)
  //and make adjustment based on that max speed closer to wall at given PINGTIME (time between pings)
		  if(sideDiff == 0) return;
	  
	  //Serial.println(abs(sideDiff));
		  mmIdealMiss = mmSideIdeal - mmSideCur;  //supposed to be used to proportionately adjust distance from wall

	  if(sideDiff < 0){  //cur-last < 0 means getting closer to wall, slow down left wheel
		adjustL = map(-sideDiff, 1, 8, minAdjust, maxAdjust);   //have to adjust in_max according to PINGTIME. shorter time between pings = less possible max distance traveled
		if(adjustL > maxAdjust) adjustL = maxAdjust;  //if sideDiff > in_max, adjustL could be set very high, so cap at 30 manually
		//speedL -= adjustL;
		speedL = speedLstraight - adjustL;
		//if(speedL<60) speedL = 60;
		
		adjustR = map(-sideDiff, 1, 8, minAdjust, maxAdjust);
		if(adjustR > maxAdjust) adjustR = maxAdjust;
		//speedR += adjustR;  //at the same time, adjust right wheel to go faster for quicker response to go parallel
		speedR = speedRstraight + adjustR;
		//if(speedR>speedRstraight+30) speedR = speedRstraight+30;  //just in case it goes too high
		
		motorL.setSpeed(speedL);
		motorR.setSpeed(speedR);  //less than ideal speed, speed back up to optimal
	  }
	  else if(sideDiff > 0){
		adjustR = map(sideDiff, 1, 8, minAdjust, maxAdjust);  //in_max of 8 is from observational measurements
		if(adjustR > maxAdjust) adjustR = maxAdjust;
		//speedR -= adjustR;
		speedR = speedRstraight - adjustR;
		//if(speedR<60) speedR = 60;  //don't need these checks since adjustR and adjustL are checked against adjustMax anyway
		
		adjustL = map(sideDiff, 1, 8, minAdjust, maxAdjust);
		if(adjustL > maxAdjust) adjustL = maxAdjust;
		//speedL += adjustL;  //at the same time, adjust right wheel to go faster for quicker response to go parallel
		speedL = speedLstraight + adjustL;
		//if(speedL>speedLstraight+30) speedL = speedLstraight+30;  //just in case it goes too high
		
		motorR.setSpeed(speedR);
		motorL.setSpeed(speedL);
	}
}

void turningLeft()
{
	//when front sensor gets response within certain distance
	boolean doneTurning = false;
	while(!doneTurning) {
		pingFront();
		if(mmFrontCur > 150){
			doneTurning = true; 
		}
		else{
			motorR.setSpeed(speedRstraight);
			motorL.setSpeed(speedLstraight);
			motorR.run(FORWARD);
			motorL.run(BACKWARD);
			delay(50);
		}
	}

	//apply reverse torque briefly to brake
	motorR.run(BACKWARD);
	motorL.run(FORWARD);
	delay(75);

	motorR.setSpeed(0);
	motorL.setSpeed(0);
	motorL.run(FORWARD);
	motorR.run(FORWARD);
	motorR.setSpeed(67);
	motorL.setSpeed(60);
	STATE = MOVING;

	pingSide();
	delay(1);
	pingSide();
	delay(1);
	pingFront();
	delay(1);
	pingFront();
	delay(1);
}

void turningRight()
{
//in this function, should just turn slowly until pings find right edge again, then go to MOVING so that it goes forward a bit, loses it, and comes back
//to this function again until a straight wall is found and it just stays in MOVING state
//hypothetically, this method will give accurate 90deg and 180deg turns without any special additional code since it progressively takes the turn
	boolean edgeFound = false;
	boolean wallFound = false;
	
	unsigned long edgeLastTime;  //last time since we lost the edge; if it's been longer than X ms since last losing edge, assume wall found, go back to MOVING to adjust parallelsim and continue
	unsigned long edgeCurTime;
	
	//move forward a little further to clear it, then stop
	motorL.setSpeed(speedLstraight-20);  //realign motors to go straight before delay, otherwise speedL and speedR could have adjustments applied to them, and delaying will make them turn wrong
	motorR.setSpeed(speedRstraight-10);
	delay(725);  //increase this value until robot clears edge enough before stopping
	motorL.run(BACKWARD);
	motorR.run(BACKWARD);
	motorL.setSpeed(speedLstraight+20);
	motorR.setSpeed(speedRstraight);
	delay(50);
	motorL.run(FORWARD);
	motorR.run(FORWARD);
	
	motorL.setSpeed(0);
	motorR.setSpeed(0);
	delay(250);  //long delay to start just to visualize what it's doing, decrease to optimize speed
	
	
	//90 right turn
	motorL.setSpeed(speedLstraight);
	motorR.run(BACKWARD);
	motorR.setSpeed(speedRstraight);
	delay(775);
	motorL.run(BACKWARD);
	motorR.run(FORWARD);
	motorL.setSpeed(100);
	motorR.setSpeed(100);
	delay(50);
	motorL.run(FORWARD);
	motorL.setSpeed(0);
	motorR.setSpeed(0);
   
	delay(250);  //decrease to optimize
	
	//mmSideCur = 500;
	//mmSideLast = 500;
	//now while loop just going straight until wall detected, then go to moving state
	while(!wallFound){
		//while wall not found, move forward. Find wall-->go to moving state; hopefully updates fast enough to catch the edge then loses it to go into turning_right again
		//if it doesn't see a lost edge, then it's just a 90deg turn and MOVING can take over
		motorL.setSpeed(speedLstraight-20);
		motorR.setSpeed(speedRstraight-20);
		pingSide();
		delay(50);

		pingFront(); //hopefully this won't interfere with the other b/c of the delay
		if(mmFrontCur < 100)
		{
			STATE = TURNING_LEFT; // for safety
			return;
		}

		if(mmSideCur < 300){
			//detect close edge, go to moving
			//pingSide();
			//delay(10);
			//pingSide();
			//mmSideCur = 500;
			//mmSideLast = 500;
			STATE = MOVING;
			//LASTSTATE = TURNING_RIGHT;
			//delay(400);
			return;
		}
	}
}

void findWall()
{
	/*
	
	*/
}

int usTOmm(long microseconds)
{
	return (int)(microseconds / 5.8);  //29 uS per cm, so 2.9 uS per mm; (uS/2.9)/2 = uS/5.8; cast to int, units in mm as whole numbers
}

int frontMM()
{
	return usTOmm(ping(FRONTPING)); 
}

int sideMM()
{
	return usTOmm(ping(SIDEPING)); 
}
