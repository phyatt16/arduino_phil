#include "Wire.h"
void setup() 
{
  Wire.begin(6);                // join i2c bus
}

void loop() 
{
  Wire.beginTransmission(6);
  Wire.write(1);
//  Wire.write("x is: ");
//  Wire.write(3);
  Wire.endTransmission();
}
