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

float speed = 80;

#define FRONTPING A0
#define SIDEPING A1
#define PINGTIME 75  //ms between ping reading updates  //at 300ms and L/R speeds of 90/99, will go at most 3cm closer to wall in that time assuming it stays within 45deg of wall

AF_DCMotor motorL(1);
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
int mmFrontLast = 0; //make sure to leave a little time between readings so enough distance change happens to be useful data
int mmSideCur = 0;   //+not too long or robot will be far off course
int mmSideLast = 0;

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
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  

  motorR.setSpeed(0);
  motorR.run(RELEASE);
  motorL.setSpeed(0);
  motorL.run(RELEASE);

  Serial.println("Setup complete");
  updatePings();
  updatePings();  //initialize ping sensors so it doesn't accidentally jump to different states
}

void loop()
{
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
   

     //if MOVING and diff between curPing and lastPing is above certain amount (means that suddenly sees longer range); most likely right turn
     /*
     if((STATE == MOVING && (mmSideCur - mmSideLast) > 100) || (LASTSTATE == TURNING_RIGHT && mmSideCur > 250 && mmSideLast > 250)) {  //either currently moving and see jump in dist, or in middle of right turn looking for wall again
       STATE = TURNING_RIGHT;
     }
     else if (STATE == TURNING_RIGHT && mmSideCur < 250) {
       //currently turning right, and now see wall, so go forward again in MOVING state
       //if it needs to turn right again, will get caught by if statement above and will rotate more until wall found
       //if it's actually the straight wall again, will just stay in MOVING state anyway
       STATE = MOVING;
     }
     */
     LASTSTATE = STATE;  //keep history of state to make better judgements
     if((STATE == MOVING) && (mmSideCur-mmSideLast > 100)){  //if it's moving and it sees a sudden increase in distance (10cm discontinuity) then assume needs to turn right
       STATE = TURNING_RIGHT;
     }
     else {
        STATE = MOVING;
     }
   	//Then call the appropriate code for the current state
   
   switch(STATE)
   	{
   		case MOVING:
                        digitalWrite(A5, LOW);  //turn off to indicate MOVING state
   			movingState();
   			break;
   		case TURNING_LEFT:
   			turningLeft();
   			break;
   		case TURNING_RIGHT:
                        digitalWrite(A5, HIGH);  //indicate that we're in the turning right state
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
  
  if(sideDiff == 0) return;
  
  Serial.println(abs(sideDiff));
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
 //in this function, should just turn slowly until pings find right edge again, then go to MOVING so that it goes forward a bit, loses it, and comes back
 //to this function again until a straight wall is found and it just stays in MOVING state
 //hypothetically, this method will give accurate 90deg and 180deg turns without any special additional code since it progressively takes the turn
    boolean edgeFound = false;
    boolean wallFound = false;
    
    unsigned long edgeLastTime;  //last time since we lost the edge; if it's been longer than X ms since last losing edge, assume wall found, go back to MOVING to adjust parallelsim and continue
    unsigned long edgeCurTime;
    
    //to see how it functions, just full stop with reverse torque to minimize drift from momentum 
    motorL.run(BACKWARD);
    motorR.run(BACKWARD);
    motorL.setSpeed(100);
    motorR.setSpeed(100);
    delay(50);
    motorL.run(FORWARD);
    motorR.run(FORWARD);
    
    motorL.setSpeed(60);  //go a little further to make sure clear of wall edge while turning right, adjust as necessary
    motorR.setSpeed(60);
    delay(50);
    
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    delay(3000);  //long delay to start just to visualize what it's doing, decrease to optimize speed
    
    
    //TODO if it sees wall too close, give longer movement forward before checking again so it's further radius from wall
    
    edgeCurTime = millis();
    edgeLastTime = edgeCurTime;
    
    while(!wallFound){  //keep doing edge finding/moving forward until certain time has passed since last lost edge
      while(!edgeFound){  //while we haven't found the edge yet
         motorL.setSpeed(speedLstraight);
         motorR.run(BACKWARD);
         motorR.setSpeed(speedRstraight);
         delay(25);
         motorL.setSpeed(0);
         motorR.run(FORWARD);
         motorR.setSpeed(0);
         delay(50);
         updatePings();
         if(mmSideCur < 250){  //shouldn't be any other walls within ~25cm of side of robot besides the turn it's tracking; change as needed
           edgeFound = true;
         }
      }
      updatePings();
      updatePings();
      while( !((mmSideCur - mmSideLast) > 100) && (edgeCurTime - edgeLastTime < 1000)){  //while not the case that it sees discontinuity (meaning lost edge) AND that time hasn't expired, keep going forward in here
        motorL.setSpeed(60);
        motorR.setSpeed(67); //guessing at matched value for straightness, doesn't need to be perfect
        updatePings();
        //lastTime = curTime;
        edgeCurTime = millis();
      }
      if(edgeCurTime - edgeLastTime < 1000) {
        //time expired, assume straight wall ahead, set MOVING state and return to main proram loop
        STATE = MOVING;
        return;  //TODO make sure no cleanup/other change of variables needs to take place before leaving function (unlikely)
      }
      else{
        edgeLastTime = millis();  //last time we lost edge was now
        edgeFound = false;  //lost the edge, let the wallFound bool make it go back into !edgeFound loop
      }
         //now move forward until either lose the edge or time expires
           //if lose edge, update time, let it go back to edge finding while loop, change edgeFound = false
           //if time expires, set state to MOVING and return from turningRight() function
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

