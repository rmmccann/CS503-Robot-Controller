void setup()
{
  Serial.begin(115200);  //set speed to transfer data over serial
  randomSeed(analogRead(0));  //seed the random number generator by reading a floating pin on the ADC
}

void loop()
{
  Serial.println((int)(random(0,1024)));  //print a random int between [0,1023]
  delay(50);   //delay so it's not sending a mass amount of data; wouldn't be able to see what's going on in processing if too fast
  /*
  for(int x=0; x<1024; x++){
    Serial.println(x);
    delay(25);
  }
  */
}

