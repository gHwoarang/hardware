#include <Wire.h>

//using 12C protocol

const int MPU = 0x6B// mpu adresi atanmali 
float p,q,r;// angular speed
float udot,vdot,wdot;//linear acceleration 
float roll,pitch,yaw;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);// degismeli belki
Wire.write(0x00);
Wire.endTransmission(true);
// is 
}

void loop() {
Wire
  
  delay(1000);
}