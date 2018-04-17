//----------------------------------------------------------------------//
/*
Attach From Arduino to MPU6050:
5V - VCC
GND - GND
A4 - SDA
A5 - SCL
*/
//----------------------------------------------------------------------//
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int axlevel=0;
int aylevel=0;
int azlevel=0;
double thetax, thetay, thetax_dot, thetay_dot;
int x_thrust, y_thrust;
double kp=255.0/90.0;
double kd=455.0/90.0;

#define LED_PIN 13

bool blinkState = false;

void setup() 
{
    Serial.begin(9600);
    // initialize device;
    accelgyro.initialize();
    Serial.println('Calibrating...');
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(az>=azlevel)
    {
       azlevel=az/100.0;
       axlevel=ax/100.0;
       aylevel=ay/100.0;
    }
    
    // -76	-2359	1688	0	0	0
    accelgyro.setXAccelOffset(0); //290
    accelgyro.setYAccelOffset(0); //-990
    accelgyro.setZAccelOffset(0); //-819
    accelgyro.setXGyroOffset(0); //22
    accelgyro.setYGyroOffset(0); //-21
    accelgyro.setZGyroOffset(0); //21
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    pinMode(3,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(9,OUTPUT);
}

void loop() {
    delay(50);
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax = ax/100.0 - axlevel;
    ay = ay/100.0 - aylevel;
    az = az/100.0 - azlevel;
    gx = gx/100.0;
    gy = gy/100.0;
    gz = gz/100.0;
    
    thetax = ax*90.0/170.0;
    thetay = ay*90.0/165.0;
    thetax_dot = -gy;
    thetay_dot = gx;
    
    x_thrust = kp*(thetax)-kd*thetax_dot;
    y_thrust = kp*(thetay)-kd*thetay_dot;

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
/*
    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\n");
*/

    Serial.print("\nX Thrust:   ");Serial.print(x_thrust);
    Serial.print("\tY Thrust:   ");Serial.print(y_thrust);
    Serial.print("\tTheta x:   ");Serial.print(thetax);
    Serial.print("\tTheta y:   ");Serial.print(thetay);
    if(x_thrust>0)
    {analogWrite(3,x_thrust);}
    else
    {digitalWrite(3,LOW);}
    if(x_thrust<0)
    {analogWrite(6,-x_thrust);}
    else
    {digitalWrite(6,LOW);}
    if(y_thrust>0)
    {analogWrite(5,y_thrust);}
    else
    {digitalWrite(5,LOW);}
    if(thetay<0)
    {analogWrite(9,-y_thrust);}
    else
    {digitalWrite(9,LOW);}
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
