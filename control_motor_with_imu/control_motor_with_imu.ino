// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t ax,ay,az,Tmp,gx,gy,gz;
double ax_off,ay_off,az_off,gx_off,gy_off,gz_off,thetax,thetay,thetaz,tic,toc,dtime,thetax_accel,thetay_accel;
double kp1, kp2;


void setup(){
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);

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

  // Motor Kp values
  kp1 = 2;
  kp2 = 2;
}
void loop(){
  tic = millis();
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

  thetax_accel = atan2(ay,az)*180.0/3.14159;
  thetay_accel = atan2(-ax,az)*180.0/3.14159;

  gx = gx - gx_off;
  gy = gy - gy_off;
  gz = gz - gz_off;

  dtime = (tic-toc)/1000/100;
  thetax = .05*thetax_accel + .95*(thetax+dtime*gx);
  thetay = .05*thetay_accel + .95*(thetay+dtime*gy);
  thetaz = thetaz + dtime*gz;

  Serial.print(" | thetax = "); Serial.print(thetax);
  Serial.print(" | thetay = "); Serial.print(thetay);
  Serial.print(" | thetaz = "); Serial.print(thetaz);
  Serial.print(" | Tmp = "); Serial.println(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  /*
  //Serial.print(" | GyY = "); Serial.print(gy);
  //Serial.print(" | GyZ = "); Serial.println(gz);
  Serial.print(" | gx = "); Serial.print(gx);
  Serial.print(" | gy = "); Serial.print(gy);
  Serial.print(" | gz = "); Serial.print(gz);
  Serial.print(" | ax = "); Serial.print(ax);
  Serial.print(" | ay = "); Serial.print(ay);
  Serial.print(" | az = "); Serial.println(az);
  */

  // Control the motor
  if(thetay>0)
  {
    analogWrite(3,kp1*(thetay*255/90));
  }
  if(thetay<0)
  {
    analogWrite(5,kp1*(-thetay*255/90));
  }
  
  delay(1);
  toc = tic;
}
