import processing.serial.*;

Serial myPort;  //handle for the serial port to check for data from Arduino
String inString;  //string to hold data read on serial port
int lf = 10;  //ASCII linefeed, println generates the linefeed
int oldx, oldy, screen_increment, val = 0;  //some other variables for setting up Processing screen later

void setup()
{
 size(displayWidth-100, 512);         //set size of window where data will be displayed
 println(Serial.list());              //to view all ports available; Device Manager shows COM11 used for Arduino and it's the 2nd one so index 1
 String portName = Serial.list()[1];  //have to change this to to wherever the Arduino COM port is in the list "print(Serial.list())"
 println(portName);                   //print out portName just to double check that the right one is chosen
 myPort = new Serial(this, portName, 115200);  //open a port to the Arduino, set baud rate etc
 myPort.bufferUntil(lf);              //buffer the data coming in until it sees a linefeed (println in arduino will put a linefeed at the end
 //background(208, 24, 24);
 background(255,255,255);             //set RGB background color of the window, 255,255,255 white
}

void draw()
{
 //like the arduino loop 
}

/*
  runs this code any time the port gets a linefeed in the buffer
*/
void serialEvent(Serial myPort)
{
  inString = myPort.readString();  //get data from buffer as a string
  inString = trim(inString);       //"Removes whitespace characters from the beginning and end of a String. In addition to standard whitespace characters such as space, carriage return, and tab, this function also removes the Unicode "nbsp" character."
  val = int(inString)/2;  //div by two since screen size is 512 and values coming in at up to 1024 assuming direct values from ADC
  strokeWeight(3);  //increase line thickness to 6
  stroke(0, 0, 0);  //RGB color of line - 0,0,0 = black
  
  line(oldx, oldy, screen_increment, 512-val);  //1024-val because val comes in from 0 to 1024, and the y axis in processing is 0 at top, higher num on bottom
    //line just draws a line between two points, just incrementing which the last point was so that a new line can be drawn from most recent point to next point of data
  
  oldx = screen_increment;  //update the coord of the last point of data, so a new line can be drawn from it to next point
  oldy = 512-val;
  
  screen_increment = screen_increment+10;  //space out the points in the x-axis by 10
  if(screen_increment > (displayWidth - 100)){  //clear the screen and reset the current point position once the screen goes all the way to the right side
    background(255,255,255);  //clears the screen, resets all to white
    screen_increment=-50;  //set it slightly off the screen to start since reseting the oldy will cause a big line to appear
    oldx = -50;
    oldy = 0;
  }

  println(val);  //just debugging by printing out value to console
}
