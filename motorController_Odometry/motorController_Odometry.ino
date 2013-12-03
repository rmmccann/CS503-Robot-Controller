/*
Interrupt test that simply counts the number of
black->white edges on each wheel asynchronously of
the main program loop and prints a message over serial
every time each wheel turns one revolution.

Note: I set the baud rate to 115200 bps to minimize the time
that data is transferring over serial so hopefully none of the code
blocks even while running the wheels quickly. You have to change the baud rate
of the Serial Monitor window in the Arduino IDE to match that rate or else you
will only see gibberish.
*/

#include <AFMotor.h>

//make interrupt handler faster by telling it that there will be no interrupts on ports B or D (analog pins 0-5 are all on port C on the ATMega328P chip)
#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
//#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts

//now include the PinChangeInt library
#include <PinChangeInt.h>

#include <stdarg.h> //needed for printf

#define LED A5

//the pins that will be watching for interrupt signals
#define LEFTINT A0
#define RIGHTINT A1

#define NUMSEG 24 //number of black segments on the encoder wheel (just used to test counting number of wheel rotations in this example
#define DIAMETER 76.00 // diameter of each wheel
#define WIDTH 125.00 // width of the vehicle, or distance between two rear wheels
#define RESOLUTION 15 // resolution of current sector 15 degrees

#define LSTRAIGHT 90
#define RSTRAIGHT 105

///////////////////////////////////
//parallel wall following things
///////////////////////////////////
#define FRONTPING A2
#define SIDEPING A3
#define PINGTIME 75  //ms between ping reading updates  //at 300ms and L/R speeds of 90/99, will go at most 3cm closer to wall in that time assuming it stays within 45deg of wall

const int MOVING = 0;
const int TURNING_LEFT = 1;
const int TURNING_RIGHT = 2;
int speedLstraight = LSTRAIGHT;  //approximate values to go straight
int speedRstraight = RSTRAIGHT;  //original 99
int speedL = speedLstraight;
int speedR = speedRstraight;


int mmFrontCur = 0;  //ping distance history; use two distance readings to know how non-parallel robot is and which way to turn to correct
int mmFrontLast = 0; //make sure to leave a little time between readings so enough distance change happens to be useful data

int mmSideCur = 0;   //+not too long or robot will be far off course
int mmSideLast = 0;

int frontDiff = 0;  //calculated immediately following ping readings for moving state
int sideDiff = 0;

const int maxAdjust = 120;  //maximum adjustment for re-aligning robot; turns quickly enough and won't stall
const int minAdjust = 10;

int adjustL;  //speed adjustment; function of diff between side ping readings
int adjustR;

unsigned long lastTime = 0; //last time ping readings were taken; used to space out pings so that some distance will accumulate
unsigned long curTime = 0;
//////////////////////////////



//variables to store the number of falling edges seen by the sensors in the interrupts (eg: counts how many times it sees a black->white transition)
volatile long lcount = 0;
volatile long rcount = 0;

//just some other variables used solely for this example of counting wheel rotation
int rrotation = 0;
int lrotation = 0;

//time entry
unsigned long timeCurL = 0;
unsigned long timeLastL = 0;
unsigned long timeLastLastL = 0;
unsigned long timeCurR = 0;
unsigned long timeLastR = 0;
unsigned long timeLastLastR = 0;

// initialization
float forwardRefL = LSTRAIGHT;    // start pwm speed for left wheel; reference speed for move forward of left wheel 
float forwardRefR = RSTRAIGHT;    // start pwm speed for right wheel; reference speed for moving forward of right wheel
float PWM_CurL = forwardRefL;
float PWM_CurR = forwardRefR;

float currentDist; //current cumulative distance of robot using left and right wheel distances in interrupts so it's asynchronosly up to date. Unit: MM

// update variables for vehicle velocities of both wheels
float disCurL;          // current distance of left wheel
float disCurR;          // current distance of right wheel
float disLastL = 0;        // last distance of left wheel
float disLastR = 0;        // last distance of right wheel
float velCurL;          // Current velocity of left wheel
float velCurR;          // Current velocity of right wheel
float velLastL = 0;         // Last velocity of left wheel
float velLastR = 0;         // Last velocity of right wheel
float accCurL;          // Current acceleration of left wheel
float accCurR;          // Current acceleration of right wheel  
float errDis;           // error of current distance of both wheels
float errVel;           // error of current velocity of both wheels
float errVelDiff;       // error of current acceleration of both wheels
long PWM_NextR;        // Current velocity error for right wheel
long PWM_NextL;        // Current velocity error for left wheel

//reference PWM speed for both sides turning of both wheels 
long turnRightRefR = 60;
long turnRightRefL = 79;
long turnLeftRefR = 90;
long turnLeftRefL = 60;

//define the control parameters 
float kd = 4; //1st derivative of heading errors correct factor 0.5
float kp = 3; //2nd derivative of heading errors correct factor 0.03
float ki = 0; // heading error correct factor 0.5

//flags for debug statements
boolean usedLastInterrupt = false;  //so that PDController isn't called multiple times between interrupts (doing so would cause it to calculate the same error adjustment multiple times
                                    //+for the same last/current state of the robot which would likely swing the adjustment of the PWMs to the extremes between interrupts which is incorrect

//distance for loops
//odometer
#define length_a (3352.8 - 20)  //original 3352.8
#define radius_b (457.2 - 5)  //original
#define length_c (1219.2 + 90 - 101.6)  //101.6mm = 4in which is amount late it is going into/coming out of turn d
#define radius_d (304.8 + 30)
#define length_e (1828.8 - 150)
#define radius_f (304.8 - 50)
#define length_g 3962.4
#define length_b radius_b*PI
#define length_d 0.5*radius_d*PI
#define length_f radius_f*PI
#define length_bl (radius_b + WIDTH/2)*PI
#define length_dl 0.5*(radius_d - WIDTH/2)*PI  //center needs to go further by 2" = 50.8mm, so increase rightwheel length and leftwheel length proportionately to how much each of them needs to change
#define length_fl (radius_f - WIDTH/2)*PI
#define length_br (radius_b - WIDTH/2)*PI
#define length_dr 0.5*(radius_b + WIDTH/2)*PI  //if ref point needs to go extra 2", then left needs extra 
#define length_fr (radius_b + WIDTH/2)*PI


float arcRight = 0;
float arcLeft = 0;
#define lengthStripe = 9.95;
float speedRatio = 0;

int state = 0;  //which segment it's currently on so it can use the distances for error correction
float segmentDistances[7] = {
   length_a,   //A
   length_b,
   length_c,
   length_d,
   length_e,
   length_f,
   length_g  //G
};

float segDistL[7] = {
  length_a,
  length_bl,
  length_c,
  length_dl,
  length_e,
  length_fl,
  length_g
};

float segDistR[7] = {
  length_a,
  length_br,
  length_c,
  length_dr,
  length_e,
  length_fr,
  length_g
};

float cumulativeL[7] = {
 0,
 segDistL[0],
 segDistL[0]+segDistL[1],
 segDistL[0]+segDistL[1]+segDistL[2],
 segDistL[0]+segDistL[1]+segDistL[2]+segDistL[3],
 segDistL[0]+segDistL[1]+segDistL[2]+segDistL[3]+segDistL[4],
 segDistL[0]+segDistL[1]+segDistL[2]+segDistL[3]+segDistL[4]+segDistL[5]
};

float cumulativeR[7] = {
 0,
 segDistR[0],
 segDistR[0]+segDistR[1],
 segDistR[0]+segDistR[1]+segDistR[2],
 segDistR[0]+segDistR[1]+segDistR[2]+segDistR[3],
 segDistR[0]+segDistR[1]+segDistR[2]+segDistR[3]+segDistR[4],
 segDistR[0]+segDistR[1]+segDistR[2]+segDistR[3]+segDistR[4]+segDistR[5]
};

//total distances to the end of each section of the course
float dist_a = length_a;
float dist_b = dist_a + length_b;
float dist_c = dist_b + length_c;
float dist_d = dist_c + length_d + 66.0;  //add 3" to the 90deg left circle so it turns more, might mess up some equations
float dist_e = dist_d + length_e - 66.0;  //remove the above 3" so it doesn't accumulate for rest of course
float dist_f = dist_e + length_f + 65.8;
float dist_g = dist_f + length_g - 65.8;

AF_DCMotor left(1);
AF_DCMotor right(2);

boolean firstStopped = false;  //so it knows if it has stopped yet
float firstDist = 609.6 + 76.2; //609.6mm into a given segment the stop point is (add distance as necessary to adjust)

boolean secondStopped = false;
float secondDist = 210.0;

boolean thirdStopped = false;
float thirdDist = 3926.4 - 20;

void setup(){
  Serial.begin(115200);
  Serial.println("~~~~~~~~~~~~~BEGIN~~~~~~~~~~~~~");
  //Note: setup interrupt pins before running motors so all wheel movement is accounted for
 
  //initialize values for ping sensors, do this any time wall following code is used
  pingSide();
  delay(5);
  pingSide();
  delay(5);
  pingFront();
  delay(5);
  pingFront();
  delay(5);

 
  //attach left wheel sensor pin to interrupt, as well as which function to call when an interrupt is triggered on this pin
  pinMode(LEFTINT, INPUT);
  digitalWrite(LEFTINT, LOW);
  PCintPort::attachInterrupt(LEFTINT, &leftInterrupt, FALLING);
  
  //attach right interrupt, etc
  pinMode(RIGHTINT, INPUT);
  digitalWrite(RIGHTINT, LOW);
  PCintPort::attachInterrupt(RIGHTINT, &rightInterrupt, FALLING);
  
  //start the left motor and get it running slowly
  left.run(FORWARD);
//  left.setSpeed(255);
//  delay(25);
  left.setSpeed(forwardRefL);
  
  
  //start the right motor, get it running faster so show that the interrupts are watching each wheel separately
  right.run(FORWARD);
//  right.setSpeed(255);
//  delay(25);
  right.setSpeed(forwardRefR);

  //setup led
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);
  
  //set serial port to run at 115200 bps so less time is spent doing prints
  //make sure to choose the corresponding baud rate in the bottom right of the Arduino IDE's serial monitor or else it will just show gibberish

  while(lcount < 4 && rcount < 4)
  {
    //just let it go straight with forwardRef values until a few interrupts happen so that the calculations are representative without cold-start initialized values causing large errors
  }
}

float dist_a_mm = 5181.6; //mm == 17ft

void loop()
{
  dist_a_mm = 914.4; //3ft
  if(usedLastInterrupt == false){  //haven't used data generated in last interrupt, so recalculate things now and set usedLastInterrupt to true so that it won't get calculated again until new data about the robot's state is determined
    usedLastInterrupt = true;
    Serial.print("current distance: ");
    Serial.println(currentDist);
    
    if(0 < currentDist && currentDist < dist_a_mm) {  // go 17 ft
      Serial.println("AAAA");
      //digitalWrite(RIGHT_LED, LOW);
      state = 0;
      moveForward();
    }
    else if(dist_a_mm <= currentDist && currentDist < 3*dist_a_mm) {
        digitalWrite(LED, HIGH);
        state = 1;
        //flush ping values so it initializes the values when it needs to do wall following
        pingSide();
        delay(5);
        pingSide();
        delay(5);
        while(mmSideCur < 406.4){
          //while right sensor sees within 16in (406.4mm) meaning that it's still in hallway
          timeUpdate();
          wallFollow();
        }
        fullStop();
        // state = NEXT_STATE;
        // state++;
    }
    else if(dist_b <= currentDist && currentDist < dist_c) {  // C
        state = 2;
        if(!firstStopped && (currentDist > (dist_b + firstDist))) {
          firstStopped = true;
          stopBot();  //will full stop for a few sec, then just continue back into these loops
        }
        Serial.println("CCCC");
        moveForward ();
    }
    else if(dist_c <= currentDist && currentDist < dist_d) {  // D
        state = 3;
        Serial.println("DDDD");
        turnLeft (radius_d);
    }
    else if(dist_d <= currentDist && currentDist < dist_e) {  // E
        state = 4;
        moveForward();
    }
    else if(dist_e <= currentDist && currentDist < dist_f) {  // F
        state = 5;
        turnLeft (radius_f);
    }
    else if(dist_f <= currentDist) {  // G
        state = 6;
        if(!secondStopped && (currentDist > (dist_f+secondDist))){
          secondStopped = true;
          stopBot();
        }
        if(!thirdStopped && (currentDist > (dist_f+thirdDist))){
          thirdStopped = true;
          stopBot();
        }
        moveForward();
    }
    Serial.println("");

  }//end of usedLastInterrupt flag-checking if statement

}

/*
///////////////////////////
Wall following functions
///////////////////////////
*/

void timeUpdate()
{
  curTime = millis();
  if(curTime - lastTime > PINGTIME) {
    pingSide();
    delay(10);
    pingFront();
    
    lastTime = curTime;
  }
}

void wallFollow()
{
          if(sideDiff == 0) return;
          if(sideDiff < 0){  //cur-last < 0 means getting closer to wall, slow down left wheel
            adjustL = map(-sideDiff, 1, 8, minAdjust, maxAdjust);   //have to adjust in_max according to PINGTIME. shorter time between pings = less possible max distance traveled
            if(adjustL > maxAdjust)
              adjustL = maxAdjust;  //if sideDiff > in_max, adjustL could be set very high, so cap at 30 manually
            speedL = speedLstraight - adjustL;
           
            adjustR = map(-sideDiff, 1, 8, minAdjust, maxAdjust);
            if(adjustR > maxAdjust)
              adjustR = maxAdjust;
            speedR = speedRstraight + adjustR;
            
            left.setSpeed(speedL);
            right.setSpeed(speedR);
          }
          else if(sideDiff > 0){
            adjustR = map(sideDiff, 1, 8, minAdjust, maxAdjust);  //in_max of 8 is from observational measurements
            if(adjustR > maxAdjust)
              adjustR = maxAdjust;
            speedR = speedRstraight - adjustR;

            adjustL = map(sideDiff, 1, 8, minAdjust, maxAdjust);
            if(adjustL > maxAdjust)
              adjustL = maxAdjust;
            speedL = speedLstraight + adjustL;      
            
            right.setSpeed(speedR);
            left.setSpeed(speedL);
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

int usTOmm(long microseconds) { return (int)(microseconds / 5.8); }  //29 uS per cm, so 2.9 uS per mm; (uS/2.9)/2 = uS/5.8; cast to int, units in mm as whole numbers

int frontMM() { return usTOmm(ping(FRONTPING)); }

int sideMM() { return usTOmm(ping(SIDEPING)); }

/*
////////////////////
Odometry functions
////////////////////
*/

void turnLeft(float radius)
{
        //digitalWrite(LEFT_LED, HIGH);
        //TODO: set motor speeds
        arcLeft = 0;
        arcRight = 0;
        speedRatio = 0;

        arcRight = (radius + WIDTH/2)* PI;
        arcLeft = (radius - WIDTH/2)* PI;
        speedRatio = (arcRight/arcLeft);

        PDController(turnLeftRefL,turnLeftRefR,speedRatio);
        Serial.print("speed ratio = ");
        Serial.println(speedRatio);
}

void turnRight(float radius)
{
        //digitalWrite(RIGHT_LED, HIGH);
        
        arcLeft = 0;
        arcRight = 0;
        speedRatio = 0;
        
        arcRight = (radius - WIDTH/2)* PI;
        arcLeft = (radius + WIDTH/2)* PI;
        speedRatio = (arcRight/arcLeft);

        PDController (turnRightRefL,turnRightRefR,speedRatio);
        Serial.print("speed ratio = ");
        Serial.println(speedRatio);
}

void moveForward(){
  //digitalWrite(LEFT_LED, LOW);
  //digitalWrite(RIGHT_LED, LOW);
//  PDController(forwardRefL,forwardRefR,.00001);
  PDController(forwardRefL,forwardRefR,1);
}

void PDController(long PWM_RefL, long PWM_RefR, float speedRatio){
    if ((velCurL*speedRatio / velCurR != 1) || errVelDiff != 0  ||((disCurL-cumulativeL[state])*speedRatio / (disCurR-cumulativeR[state]) != 1)){
      //Compute distance, velocity errors and acceleration errors of both wheels
      errDis = (disCurL-cumulativeL[state])*speedRatio - (disCurR - cumulativeR[state]);
      errVel = velCurL*speedRatio - velCurR;
      errVelDiff = accCurL*speedRatio - accCurR;
      
      // Feedback error and generate new PWM on both wheels
      
      PWM_NextL = PWM_RefL - (kp*errVel - kd*errVelDiff - ki*errDis);
      PWM_NextR = PWM_RefR + (kp*errVel + kd*errVelDiff + ki*errDis);
      
      //bounds checking PWMs so that expected value of 0-255 goes to setSpeed.
      //setSpeed takes type uint8_t
      PWM_NextL = max(PWM_NextL, 0);  //if pwmnextL < 0, max(pwmnextL, 0) will return 0; etc for the rest
      PWM_NextL = min(255, PWM_NextL);
        
      PWM_NextR = max(PWM_NextR, 0);
      PWM_NextR = min(255, PWM_NextR);
      
      left.setSpeed(PWM_NextL);
      right.setSpeed(PWM_NextR);
      
      //print result
      Serial.print("left wheel velocity = ");
      Serial.println(PWM_NextL);
      Serial.print("right wheel velocity = ");
      Serial.println(PWM_NextR);  
    }
    else{
      PWM_NextL = PWM_CurL;
      PWM_NextR = PWM_CurR;
    }
    
    PWM_CurL = PWM_NextL;
    PWM_CurR = PWM_NextR;
}

void stopBot()
{
  int rcountcur;
  int lcountcur;
  //quickly stop moving and then do nothing
  left.run(BACKWARD);
  right.run(BACKWARD);
  left.setSpeed(175);
  right.setSpeed(175);
  delay(50);
  left.setSpeed(0);
  right.setSpeed(0);
  left.run(FORWARD);
  right.run(FORWARD);
  delay(30000);
  rcountcur = rcount;
  lcountcur = lcount;
  left.setSpeed(forwardRefL);  //go forward again, causing a few interrupts first before letting PDController take over again
  right.setSpeed(forwardRefR);
  while(((rcount - rcountcur) < 4) && ((lcount - lcountcur) < 4)){
    //just wait until each wheel gets a couple of interrupts before using PDController again --> basically the same method to start the robot in setup()
  }
}

void fullStop()
{
  left.run(BACKWARD);
  right.run(BACKWARD);
  left.setSpeed(175);
  right.setSpeed(175);
  delay(50);
  left.setSpeed(0);
  right.setSpeed(0);
  left.run(FORWARD);
  right.run(FORWARD);
  while(1){}
}

void rightInterrupt(){
  rcount++;
  
  disCurR = disLastR + 0.04167*(float)(DIAMETER)*PI;
  timeCurR = millis();
  velCurR = (disCurR - disLastR)*1000/(timeCurR - timeLastR);  // mm/s
  accCurR = (velCurR - velLastR)/ (timeCurR - timeLastLastR); 

  // update variables
  timeLastR = timeCurR;
  disLastR = disCurR;
  velLastR = velCurR;
  if (rcount%2 == 0){
    timeLastLastR = timeCurR;
  }
  currentDist = (disCurR+disCurL)/2;
  usedLastInterrupt = false;   //new interrupt happening now, so next time main loop runs through, allow PDController to run once to generate new error accumulations
}

void leftInterrupt(){
  lcount++;

  disCurL = disLastL + 0.04167*(float)(DIAMETER)*PI;
  timeCurL = millis();
  velCurL = (disCurL - disLastL)*1000/(timeCurL - timeLastL);
  accCurL = (velCurL - velLastL)/ (timeCurL - timeLastLastL);
  
  // update variables
  timeLastL = timeCurL;
  disLastL = disCurL;
  velLastL = velCurL;
  if (rcount%2 == 0){
    timeLastLastL = timeCurL;
  } 
  currentDist = (disCurR+disCurL)/2;
  usedLastInterrupt = false;   //new interrupt happening now, so next time main loop runs through, allow PDController to run once to generate new error accumulations
}

void printf(char *fmt, ... ){
        char tmp[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, 128, fmt, args);
        va_end (args);
        Serial.print(tmp);
}
