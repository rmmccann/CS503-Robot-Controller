/*
The Wrecking Bot ™
 © Team 1
 */

#include <AFMotor.h>
//#include <PCM.h>
//#include "sounddata.h"



int STATE = 0;

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;

float speed = 80;

#define FRONTPING A0
#define SIDEPING A1
#define PINGTIME 75  //ms between ping reading updates  //at 300ms and L/R speeds of 90/99, will go at most 3cm closer to wall in that time assuming it stays within 45deg of wall

//int ledPin = 2;

/*
///
 int followDistance = 11;
 int tolerance = 4;
 int goodTolerance = 2;
 int minFollowDist = followDistance-tolerance;
 int maxFollowDist = followDistance+tolerance;
 int minGoodRange = followDistance-goodTolerance;
 int maxGoodRange = followDistance+goodTolerance;
 
 int minFrontDist = 15;
 ///
 */

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

//long curSpaceRight;
//long curSpaceFront;

int speedLstraight = 90;  //approximate values to go straight
int speedRstraight = 99;
int speedL = speedLstraight;
int speedR = speedRstraight;

unsigned long lastTime; //last time ping readings were taken; used to space out pings so that some distance will accumulate
unsigned long curTime;

const int mmSideIdeal = 110;  //ideal dist to stay from wall ~11cm
const int mmSideTolerance = 20; //can be 2cm off either side before needing to adjust
int mmIdealMiss;

int mmFrontCur;  //ping distance history; use two distance readings to know how non-parallel robot is and which way to turn to correct
int mmFrontLast; //make sure to leave a little time between readings so enough distance change happens to be useful data
int mmSideCur;   //+not too long or robot will be far off course
int mmSideLast;

int frontDiff;  //calculated immediately following ping readings for moving state
int sideDiff;

const int maxAdjust = 30;  //maximum adjustment for re-aligning robot; turns quickly enough and won't stall
//idea is that the bigger diff in side pings, the more of an adjustment needs to be made, and vice versa
int adjustL;  //speed adjustment; function of diff between side ping readings
int adjustR;

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
  /*
	//First take Ping sensor measurements
   	long rightPingDuration = ping(rightPing);
   	curSpaceRight = microsecondsToCentimeters(rightPingDuration);
   
   	long frontPingDuration = ping(frontPing);
   	curSpaceFront = microsecondsToCentimeters(frontPingDuration);
   */
  //motorL.setSpeed(speedLstraight);
  //motorR.setSpeed(speedRstraight);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
  
	//debug
   	//Serial.print("Ping front: ");
        //Serial.print(mmFrontCur);
   	//Serial.print(", Ping right: ");
   	//Serial.println(mmSideCur);
   
   curTime = millis();
   if(curTime - lastTime > PINGTIME) {
     updatePings();  //update ping readings for both sensors, used for moving, turning might need readings more often
     lastTime = curTime;
     //Serial.println("ping");
   }
   
   	//Then call the appropriate code for the current state
   //note to self: make sure nothing will block more than few ms so ping readings can be regularly taken
   //want small, high frequency adjustments, not bold moves
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
   			findWall(); //panic mode, bad news bears; will try and find something to follow again
   //make it smart, have several modes of trying to find wall given last state before losing wall
   	}
   
}

void updatePings()
{
  mmFrontLast = mmFrontCur;
  mmSideLast = mmSideCur;
  mmFrontCur = frontMM();
  mmSideCur = sideMM();

  frontDiff = mmFrontCur - mmFrontLast;
  sideDiff = mmSideCur - mmSideLast;
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

void movingState()
{
 //get time it takes robot to get certain distance closer at max supported side reading angle (~40 deg)
  
  //and make adjustment based on that max speed closer to wall at given PINGTIME (time between pings)
  
  //figure out if turning and which direction while moving forward
  /*
  if(mmSideLast < 300 && mmSideCur >= 300){  //if last reading was reasonable, and current reading is far off, assume turning right
     STATE = TURNING_RIGHT;
     return;
  }
  */
  
  if(sideDiff == 0) return;
  
  Serial.println(abs(sideDiff));
  mmIdealMiss = mmSideIdeal - mmSideCur;  //supposed to be used to proportionately adjust distance from wall

  if(sideDiff < 0){  //cur-last < 0 means getting closer to wall, slow down left wheel
    adjustL = map(-sideDiff, 1, 8, 1, 30);   //pings happening quicker, less possible max diff, so account for it by 3 multiplier
    if(adjustL > 30) adjustL = 30;  //if sideDiff > in_max, adjustL could be set very high, so cap at 30 manually
    speedL -= adjustL;
    if(speedL<60) speedL = 60;
    motorL.setSpeed(speedL);
    motorR.setSpeed(speedRstraight);  //less than ideal speed, speed back up to optimal
  }
  else if(sideDiff > 0){
    adjustR = map(sideDiff, 1, 8, 1, 30);  //in_max of 10 is from observational measurements
    if(adjustR > 30) adjustR = 30;
    speedR -= adjustR;
    if(speedR<60) speedR = 60;
    motorR.setSpeed(speedR);
    motorL.setSpeed(speedLstraight);
  }
  //non-working "keep certain distance from wall adjustment" code
  /*(
  else if(abs(mmIdealMiss) > mmSideTolerance){  //if going mostly straight and not at right dist from wall
    if(mmIdealMiss > 0) {  //too close to wall, increase right wheel speed for a bit
      //adjustR = map(mmIdealMiss, mmSideTolerance, mmSideTolerance+150, 30, 100); //map how much it's missed the ideal by to how much it should compensate
      //motorR.setSpeed(speedR + adjustR);
      motorL.setSpeed(60);  //slow down other wheel simultaneously for quicker adjust response
      delay(90);
      //motorR.setSpeed(speedR - adjustR);  //return to speed that was making it straight
      motorL.setSpeed(speedL);
      //delay(100);  //let it get back up to speed going straight again
    }
    else {  //must be less than 0 since already know diff is outside of tolerance range
      //adjustL = map(mmIdealMiss, mmSideTolerance, mmSideTolerance+150, 30, 100); //map how much it's missed the ideal by to how much it should compensate
      //motorL.setSpeed(speedL + adjustL);
      motorR.setSpeed(60);
      delay(90);
      //motorL.setSpeed(speedL - adjustL);  //return to speed that was making it straight
      motorR.setSpeed(speedR);
      //delay(100);
    }
  }
  */
}

void turningLeft()
{
  /*
	if(mmFrontCur < minFrontDist)
   	{
   		motorR.setSpeed(speed);
   		motorL.setSpeed(speed);
   		motorR.run(FORWARD);
   		motorL.run(BACKWARD);
   	}
   	if(mmFrontCur > minFrontDist+5 && mmSideCur < maxGoodRange) //TODO work on this condition (should wait a little more)
   	{
   		STATE = MOVING;
   	}
   */
}
void turningRight()
{

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

