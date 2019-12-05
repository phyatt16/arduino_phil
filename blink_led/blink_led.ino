#include "Wire.h"
void setup() {
  Wire.begin(6);
  pinMode(LED_BUILTIN,OUTPUT);
  
}

void loop() {
  int ms = 1000;
  digitalWrite(LED_BUILTIN,HIGH);
  delay(ms);
  digitalWrite(LED_BUILTIN,LOW);
  delay(ms);

}
