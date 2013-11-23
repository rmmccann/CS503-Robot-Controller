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

//the pins that will be watching for interrupt signals
#define LEFTINT A0
#define RIGHTINT A1

#define NUMSEG 24 //number of black segments on the encoder wheel (just used to test counting number of wheel rotations in this example
#define DIAMETER 76 // diameter of each wheel
#define RESOLUTION 15 // resolution of current sector 15 degrees

//variables to store the number of falling edges seen by the sensors in the interrupts (eg: counts how many times it sees a black->white transition)
volatile long lcount = 0;
volatile long rcount = 0;

//just some other variables used solely for this example of counting wheel rotation
int rrotation = 0;
int lrotation = 0;

//odometry for both wheels

//time entry
unsigned long timeCurL = 0;
unsigned long timeLastL = 0;
unsigned long timeLastLastL = 0;
unsigned long timeCurR = 0;
unsigned long timeLastR = 0;
unsigned long timeLastLastR = 0;

// initialization
float forwardRefL = 89;    // start pwm speed for left wheel; reference speed for move forward of left wheel 89
float forwardRefR = 105;    // start pwm speed for right wheel; reference speed for moving forward of right wheel 103
float PWM_CurL = forwardRefL;
float PWM_CurR = forwardRefR;
 
// updated variables
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
float errVel;           // error of current velocity for both wheels
float errVelDiff;       // error of current acceleration for both wheels
long PWM_NextR;        // Current velocity error for right wheel
long PWM_NextL;        // Current velocity error for left wheel

//reference speed for both sides turning of both wheels 
long turnRightRefR = 60;
long turnRightRefL = 79;
long turnLeftRefR = 90;
long turnLeftRefL = 60;

//define the control parameters
float kd = 0.5; //0.5;
float kp = 0.03; //0.05;

//distance for loops

AF_DCMotor left(1);
AF_DCMotor right(2);

void setup(){
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
 // left.setSpeed(255);
 // delay(25);
  left.setSpeed(forwardRefL);
  
  
  //start the right motor, get it running faster so show that the interrupts are watching each wheel separately
  right.run(FORWARD);
//  right.setSpeed(255);
 // delay(25);
  right.setSpeed(forwardRefR);
  
  
  //set serial port to run at 115200 bps so less time is spent doing prints
  //make sure to choose the corresponding baud rate in the bottom right of the Arduino IDE's serial monitor or else it will just show gibberish
  Serial.begin(115200);
  Serial.println("~~~~~~~~~~~~~BEGIN~~~~~~~~~~~~~");
}

void loop(){
  while(1){
  if(disCurR < 3352 || disCurL < 3352){ // A
      moveForward();
      break;
  }

  if (disCurR < 3972 || disCurL < 4984){  // B
      turnRight();
      break;
  }

  if(disCurR < 5191 || disCurL < 6203) {  // C
      moveForward ();
      break;
  }
  
 // stopAt();
  if(disCurR < 5768 && disCurL < 6583) {  //D
      turnLeft ();
      break;
  }

  if (disCurR < 7596 && disCurL < 8411){  //E
      moveForward();
      break;
  }

  if(disCurR < 8750 && disCurL < 9172) {  //F
      turnLeft ();
      break;
  }

  if (disCurR < 12712 && disCurL < 13134){   //G
      moveForward();
      break;
  }

  }
/*
  //print a message once per revolution of either wheel then reset counter
  if(rcount == NUMSEG){
     Serial.print("Right wheel made full rotation # ");
     Serial.println(++rrotation);  //increment then print the right rotation counter
     rcount = 0;
  }
  if(lcount == NUMSEG){
     Serial.print("Left wheel made full rotation # ");
     Serial.println(++lrotation);  //increment then print the left rotation counter
     lcount = 0;
  }*/
}

void turnRight(){
  left.setSpeed(turnRightRefL);
  right.setSpeed(turnRightRefR);
}
void turnLeft(){
  left.setSpeed(turnLeftRefL);
  right.setSpeed(turnLeftRefR);
}

void moveForward(){
  PDController(forwardRefL,forwardRefR);
}

void PDController(long PWM_RefL, long PWM_RefR){
    if ((velCurL - velCurR != 0) || errVelDiff != 0){
      //Compute velocity errors and acceleration errors of both wheels
      errVel = velCurL - velCurR;
      errVelDiff = accCurL - accCurR;
      Serial.print("left wheel current velocity = ");
      Serial.println(velCurL);
      Serial.print("right wheel current velocity = ");
      Serial.println(velCurR);
      Serial.print("error of wheel velocity = ");
      Serial.println(errVel);
      
      // Feedback error and generate new PWM on both wheels
      PWM_NextL = PWM_RefL - kp*errVel - kd*errVelDiff;
      PWM_NextR = PWM_RefR + kp*errVel + kd*errVelDiff;
      left.setSpeed(PWM_NextL);
      right.setSpeed(PWM_NextR);
      
      //print result
//      Serial.print("left wheel velocity = ");
//      Serial.println(PWM_NextL);
//      Serial.print("right wheel velocity = ");
//      Serial.println(PWM_NextR);  
    }  
    else{
      PWM_NextL = PWM_CurL;
      PWM_NextR = PWM_CurR;
    }
    
    PWM_CurL = PWM_NextL;
    PWM_CurR = PWM_NextR;  
}

void stopAt(){
 left.setSpeed(0);
 right.setSpeed(0); 
}
//the functions called 
void rightInterrupt(){
  rcount++;
  disCurR = disLastR + 0.04167*(float)(DIAMETER)*PI;
  timeCurR = millis();
  velCurR = (disCurR - disLastR)*1000/(timeCurR - timeLastR);  // mm/s
  accCurR = (velCurR - velLastR)/ (timeCurR - timeLastLastR); 
  
//  Serial.print("Right wheel made full rotation # ");
//  Serial.println(rcount); 
//  Serial.print("Right wheel current displacement # ");
//  Serial.println(disCurR);
//  Serial.print("Right wheel last displacement # ");
//  Serial.println(disLastR);
//  Serial.print("Right wheel current velocity # ");
//  Serial.println(velCurR);
//  Serial.print("Right wheel current time # ");
//  Serial.println(timeCurR);

  // update variables
  timeLastR = timeCurR;
  disLastR = disCurR;
  velLastR = velCurR;
  if (rcount%2 == 0){
    timeLastLastR = timeCurR;
  }
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
}
