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

#define TPL 2  //test points for toggling digital pin to verify encoders
#define TPR 1

int TPLstate = LOW;
int TPRstate = LOW;

#define NUMSEG 24 //number of black segments on the encoder wheel (just used to test counting number of wheel rotations in this example

//variables to store the number of falling edges seen by the sensors in the interrupts (eg: counts how many times it sees a black->white transition)
volatile long lcount = 0;
volatile long rcount = 0;

//just some other variables used solely for this example of counting wheel rotation
int rrotation = 0;
int lrotation = 0;

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
  left.setSpeed(255);
  delay(25);
  left.setSpeed(255);
  
  //start the right motor, get it running faster so show that the interrupts are watching each wheel separately
  right.run(FORWARD);
  right.setSpeed(255);
  delay(25);
  right.setSpeed(255);
  
  
  //set serial port to run at 115200 bps so less time is spent doing prints
  //make sure to choose the corresponding baud rate in the bottom right of the Arduino IDE's serial monitor or else it will just show gibberish
  Serial.begin(115200);
  Serial.println("~~~~~~~~~~~~~BEGIN~~~~~~~~~~~~~");
  
  pinMode(TPL, OUTPUT);
  pinMode(TPR, OUTPUT);
  digitalWrite(TPL, TPLstate);
  digitalWrite(TPR, TPRstate);
  
}

void loop(){
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
  }
}

//the functions called 
void rightInterrupt(){
  rcount++;
  if(TPRstate == LOW){
    digitalWrite(TPR, HIGH);
    TPRstate = 1;
  }
  else {
    digitalWrite(TPR, LOW);
    TPRstate = 0;
  }
}

void leftInterrupt(){
  lcount++;
  if(TPLstate == 0){
    digitalWrite(TPL, HIGH);
    TPLstate = 1;
  }
  else{
    digitalWrite(TPL, LOW);
    TPLstate = 0;
  }
}
