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

#include <stdarg.h>

//the pins that will be watching for interrupt signals
#define LEFTINT A0
#define RIGHTINT A1

#define LEFT_LED A3
#define RIGHT_LED A5

#define NUMSEG 24 //number of black segments on the encoder wheel (just used to test counting number of wheel rotations in this example
#define DIAMETER 76 // diameter of each wheel
#define WIDTH 125 // width of the vehicle, or distance between two rear wheels
#define RESOLUTION 15 // resolution of current sector 15 degrees

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
float forwardRefL = 91;    // start pwm speed for left wheel; reference speed for move forward of left wheel 
float forwardRefR = 102;    // start pwm speed for right wheel; reference speed for moving forward of right wheel
float PWM_CurL = forwardRefL;
float PWM_CurR = forwardRefR;

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

// define the coordinate of the robot (x, y, theta)
float curX;
float curY;
float curTheta;
// coordinate need to be updated
float lastX = 0;
float lastY = 0;
float lastTheta = 0;
float deltaS;
float deltaSR = 0;
float deltaSL = 0;
float deltaTheta = 0;

//define the control parameters 
float kd = 4; //1st derivative of heading errors correct factor 0.5
float kp = 2; //2nd derivative of heading errors correct factor 0.03
float ki = 0; // heading error correct factor 1

//flags for debug statements
boolean usedLastInterrupt = false;  //so that PDController isn't called multiple times between interrupts (doing so would cause it to calculate the same error adjustment multiple times
                                    //+for the same last/current state of the robot which would likely swing the adjustment of the PWMs to the extremes between interrupts which is incorrect

//distance for loops
//odometer
float length_a = 3352.8;  //original
float radius_b = 457.2;  //original
float length_c = 1219.2;
float radius_d = 304.8;
float length_e = 1828.8;
float radius_f = 304.8;
float length_g = 3962.4;
float length_b = 1436.2;
//float length_b  = 478.8;  //new length B for 90deg right turn using radius_b
float length_d = 478.5;
float length_f = 957;
float arcRight = 0;
float arcLeft = 0;
float lengthStripe = 9.95;
float speedRatio = 0;

//total distances to the end of each section of the course
float dist_a = length_a;
float dist_b = dist_a + length_b;
float dist_c = dist_b + length_c;
float dist_d = dist_c + length_d;
float dist_e = dist_d + length_e;
float dist_f = dist_e + length_f;
float dist_g = dist_f + length_g;

AF_DCMotor left(1);
AF_DCMotor right(2);

void setup(){
  Serial.begin(115200);
  Serial.println("~~~~~~~~~~~~~BEGIN~~~~~~~~~~~~~");
  //Note: setup interrupt pins before running motors so all wheel movement is accounted for
 
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

  //setup leds
  pinMode(A4, OUTPUT);
  	pinMode(A5, OUTPUT);
	pinMode(A2, OUTPUT);
	pinMode(A3, OUTPUT);
	digitalWrite(A2, LOW);
	digitalWrite(A3, LOW);
	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);
  
  //set serial port to run at 115200 bps so less time is spent doing prints
  //make sure to choose the corresponding baud rate in the bottom right of the Arduino IDE's serial monitor or else it will just show gibberish

  while(lcount < 2 && rcount < 2)
  {
    //just let it go straight with forwardRef values until a few interrupts happen so that the calculations are representative without cold-start initialized values causing large errors
  }
}

void loop()
{
  if(usedLastInterrupt == false){  //haven't used data generated in last interrupt, so recalculate things now and set usedLastInterrupt to true so that it won't get calculated again until new data about the robot's state is determined
    usedLastInterrupt = true;
    float currentDist = (disCurR+disCurL)/2;
    Serial.print("current distance: ");
    Serial.println(currentDist);
    if(currentDist > dist_a) {
      Serial.print("dist_b: ");
      Serial.println(dist_b);
    }
    else{
      Serial.print("dist_a: ");
      Serial.println(dist_a);
    }
    
    if(0 < currentDist && currentDist < dist_a) {  // A
      Serial.println("AAAA");
      digitalWrite(RIGHT_LED, LOW);
      moveForward();
//      turnLeft(radius_d, 90);
//      turnRight(radius_d, 90);
    }
  
    else if(dist_a <= currentDist && currentDist < dist_b) {  // B
        Serial.println("BBBB");
        digitalWrite(RIGHT_LED, HIGH);
        turnRight(radius_b,180);
    }
  
    else if(dist_b <= currentDist && currentDist < dist_c) {  // C
        Serial.println("CCCC");
        moveForward ();
    }
    
   // stopAt();
    else if(dist_c <= currentDist && currentDist < dist_d) {  // D
        Serial.println("DDDD");
        turnLeft (radius_d, 90);
    }
  
    else if(dist_d <= currentDist && currentDist < dist_e) {  // E
        moveForward();
    }
    
    else if(dist_e <= currentDist && currentDist < dist_f) {  // F
        turnLeft (radius_f,180);
    }
    
    else if(dist_f <= currentDist && currentDist < dist_g) {  // G
        moveForward();
    }

  }//end of usedLastInterrupt flag-checking if statement

}

void turnLeft(float radius, int degree)
{
        digitalWrite(LEFT_LED, HIGH);
        
        //TODO: set motor speeds
        arcLeft = 0;
        arcRight = 0;
        speedRatio = 0;

        //TODO: speedRatio doesn't depend on degree of turn.
        if(degree==180)
        {
          arcRight = (radius + WIDTH/2)* PI;
          arcLeft = (radius - WIDTH/2)* PI;
          speedRatio = (arcLeft/arcRight);
        }
        if(degree==90)
        {
          arcRight = (radius + WIDTH/2)* PI/2;
          arcLeft = (radius - WIDTH/2)* PI/2;
          speedRatio = (arcRight/arcLeft);
        }
        PDController(turnLeftRefL,turnLeftRefR,speedRatio);
        Serial.print("speed ratio = ");
        Serial.println(speedRatio);
}

void turnRight(float radius, int degree)
{
        digitalWrite(RIGHT_LED, HIGH);
        
        arcLeft = 0;
        arcRight = 0;
        speedRatio = 0;
        if(degree==180)
        {
          arcRight = (radius - WIDTH/2)* PI;
          arcLeft = (radius + WIDTH/2)* PI;
          speedRatio = (arcRight/arcLeft);
        }
        if(degree==90)
        {
          arcRight = (radius - WIDTH/2)* PI/2;
          arcLeft = (radius + WIDTH/2)* PI/2;
          speedRatio = (arcRight/arcLeft);
        }  
        PDController (turnRightRefL,turnRightRefR,speedRatio);
        Serial.print("speed ratio = ");
        Serial.println(speedRatio);
}

void moveForward(){
  digitalWrite(LEFT_LED, LOW);
  digitalWrite(RIGHT_LED, LOW);
//  PDController(forwardRefL,forwardRefR,.00001);
  PDController(forwardRefL,forwardRefR,1);
}

void PDController(long PWM_RefL, long PWM_RefR, float speedRatio){
    if ((velCurL*speedRatio / velCurR != 1) || errVelDiff != 0){ // ||(disCurL - disCurR != 0)){
      //Compute distance, velocity errors and acceleration errors of both wheels
      errDis = disCurL*speedRatio - disCurR;
      errVel = velCurL*speedRatio - velCurR;
      errVelDiff = accCurL*speedRatio - accCurR;
      
      // Feedback error and generate new PWM on both wheels
      
      PWM_NextL = PWM_RefL - (kp*errVel - kd*errVelDiff - ki*errDis);
      PWM_NextR = PWM_RefR + (kp*errVel + kd*errVelDiff + ki*errDis);
      //bounds checking PWMs so that expected value of 0-255 goes to setSpeed.
      //setSpeed takes type uint8_t
      if(PWM_NextL > 255)
        PWM_NextL = 255;
      else if(PWM_NextL < 0){
        PWM_NextL = 0;
      }
        
      if(PWM_NextR > 255)
        PWM_NextR = 255;
      else if(PWM_NextR < 0){
        PWM_NextR = 0;
      }
      
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
  Serial.println("");
  Serial.print("Thinks it has gone ");
  Serial.print((disCurR+disCurL)/2);
  Serial.println(" mm");
  delay(2000);
  left.setSpeed(forwardRefL);
  right.setSpeed(forwardRefR);
  delay(75);
}

void stopAt(){
 left.setSpeed(0);
 right.setSpeed(0); 
}
//the functions called 
void rightInterrupt(){
  rcount++;
  // update coordinate
  disCurR = disLastR + 0.04167*(float)(DIAMETER)*PI;
  deltaSR = disCurR - disLastR;
  deltaS = 0.5*(deltaSR+deltaSL);
  deltaTheta = (deltaSR - deltaSL)/(float)WIDTH;
  curX = lastX + deltaS*cos(lastTheta + 0.5*deltaTheta);
  curY = lastY + deltaS*sin(lastTheta + 0.5*deltaTheta);
  
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
  lastX = curX;
  lastY = curY;
  lastTheta = curTheta;
  
  usedLastInterrupt = false;   //new interrupt happening now, so next time main loop runs through, allow PDController to run once to generate new error accumulations
}

void leftInterrupt(){
  lcount++;
    // update coordinate
  disCurL = disLastL + 0.04167*(float)(DIAMETER)*PI;
  deltaSL = disCurL - disLastL;
  deltaS = 0.5*(deltaSR+deltaSL);
  deltaTheta = (deltaSR - deltaSL)/(float)WIDTH;
  curX = lastX + deltaS*cos(lastTheta + 0.5*deltaTheta);
  curY = lastY + deltaS*sin(lastTheta + 0.5*deltaTheta);
  
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
