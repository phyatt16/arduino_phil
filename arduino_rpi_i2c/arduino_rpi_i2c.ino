#include "Wire.h"
void setup() 
{
  Wire.begin(6);                // join i2c bus
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turn it off
}

void loop() 
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  while (Wire.available()) 
  {
    char c = Wire.read();
    digitalWrite(LED_BUILTIN, c);   
  }
}

void requestEvent() 
{
    Wire.write(1);
    Wire.write(2);
    Wire.write(3);
}
