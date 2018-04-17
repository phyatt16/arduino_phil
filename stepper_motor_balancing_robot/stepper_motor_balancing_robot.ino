// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t ax,ay,az,Tmp,gx,gy,gz;
double ax_off,ay_off,az_off,gx_off,gy_off,gz_off,thetax0, thetax1, thetax2, thetax3,thetay,thetaz,tic,toc,dtime,thetax_accel,thetay_accel;
double balance_ang, deadzone;
int count;


void setup(){
  
  pinMode(4,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(6,OUTPUT); //MS1
  pinMode(7,OUTPUT); //MS2
  
  // MS1  MS2  Resolution
  // LOW  LOW  Full Step
  // HIGH LOW  Half Step
  // LOW  HIGH Quarter Step
  // HIGH HIGH Eighth Step
  
  digitalWrite(6,LOW);
  //digitalWrite(6,HIGH);
  //digitalWrite(7,LOW);
  digitalWrite(7,HIGH);
  
  
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Serial.begin(115200);

  // Read from the IMU while sitting still
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Assign the offsets to be the first values read
  ax_off = ax;
  ay_off = ay;
  az_off = az;

  gx_off = gx;
  gy_off = gy;
  gz_off = gz;
  
  thetax0 = 90.0;
  thetax1 = 90.0;
  thetax2 = 90.0;
  thetax3 = 90.0;
  
  balance_ang = 93.5;
  deadzone = 0.5;
  count = 0;
  
}
void loop(){
  
  tic = micros();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  ay=(Wire.read()<<8|Wire.read());  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=(Wire.read()<<8|Wire.read());  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  //gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  //gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  thetax_accel = 0.9*(atan2(ay,az)*180.0/3.14159) + 0.1*(thetax_accel);
  //thetay_accel = atan2(-ax,az)*180.0/3.14159;

  gx = gx - gx_off;
  //gy = gy - gy_off;
  //gz = gz - gz_off;

  dtime = (tic-toc)/1e9;
  thetax0 = .75*(.9*thetax_accel + 0.1*(thetax0+dtime*gx*180.0/3.14159)) + 0.125*thetax1 + .125*thetax2 + 0*thetax3;
  thetax3 = thetax2;
  thetax2 = thetax1;
  thetax1 = thetax0;
  
  //thetay = .02*thetay_accel + .98*(thetay+dtime*gy);
  //thetaz = thetaz + dtime*gz;

  if(0)
  {
    Serial.print(" | thetax = "); Serial.println(thetax0);
    //Serial.print(" | thetay = "); Serial.print(thetay);
    //Serial.print(" | thetaz = "); Serial.print(thetaz);
    //Serial.print(" | Tmp = "); Serial.println(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    
    //Serial.print(" | GyY = "); Serial.print(gy);
    //Serial.print(" | GyZ = "); Serial.println(gz);
    //Serial.print(" | gx = "); Serial.print(gx);
    //Serial.print(" | gy = "); Serial.print(gy);
    //Serial.print(" | gz = "); Serial.print(gz);
    //Serial.print(" | ax = "); Serial.print(ax);
    //Serial.print(" | ay = "); Serial.print(ay);
    //Serial.print(" | az = "); Serial.println(az);
  }

  //delay(1);
  toc = tic;
  
  // Command the motors
  //if(count==1)
  if(1)
  {
    count = 0;

    if(abs(thetax0-balance_ang)<2.5)
      {
        // Eighth step
        //digitalWrite(6,LOW);
        digitalWrite(6,HIGH);
        //digitalWrite(7,LOW);
        digitalWrite(7,HIGH);
      }
    if(abs(thetax0-balance_ang)>=2.5)
    {
      // Quarter step
      digitalWrite(6,LOW);
      //digitalWrite(6,HIGH);
      //digitalWrite(7,LOW);
      digitalWrite(7,HIGH);
    }
    if(abs(thetax0-balance_ang)>=30.5)
    {
      // half step
      //digitalWrite(6,LOW);
      digitalWrite(6,HIGH);
      digitalWrite(7,LOW);
      //digitalWrite(7,HIGH);
    }
    if(abs(thetax0-balance_ang)>=50.5)
    {
      // Full step
      digitalWrite(6,LOW);
      //digitalWrite(6,HIGH);
      digitalWrite(7,LOW);
      //digitalWrite(7,HIGH);
    }
    
    delay(1);
    
    // Set Direction
    if(thetax0<balance_ang)
    {
      // Set direction
      digitalWrite(2,LOW);
      digitalWrite(3,HIGH);
    }
    if(thetax0>balance_ang)
    {  
      // Set direction
      digitalWrite(2,HIGH);
      digitalWrite(3,LOW);
    }
    delay(1);

    
    // Establish Deadzone
    if(abs(thetax0-balance_ang)>deadzone)
    {
      // Step
      digitalWrite(4,HIGH);
      digitalWrite(4,LOW);
    }
    delay(40);
    //delayMicroseconds(100000000);

  }
  
  
  count = count + 1;
  
}
