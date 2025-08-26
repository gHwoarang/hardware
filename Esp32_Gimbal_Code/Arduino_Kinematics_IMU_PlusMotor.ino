//Motor Kodları
#include <SoftwareSerial.h>

//SoftwareSerial rS485Serial(rx,tx);
SoftwareSerial rS485Serial(5,6);

float yawMotorDeg;
float pitchMotorDeg;
float rollMotorDeg;

String yawMotor = "yaw";
String pitchMotor = "pitch";
String rollMotor = "roll";

String decimalToHexa(float decimalnum){
  decimalnum = decimalnum / 0.01;
  long quotient, remainder;
  int i, j = 0;
  char hexadecimalnum[100];
  
  quotient = decimalnum;
  while (quotient != 0)
  {
    remainder = quotient % 16;
    if (remainder < 10)
       hexadecimalnum[j++] = 48 + remainder;
    else
       hexadecimalnum[j++] = 55 + remainder;
    quotient = quotient / 16;
  }
  
  String hexa = String(hexadecimalnum[1]) + String(hexadecimalnum[0]) + String(hexadecimalnum[3]) + String(hexadecimalnum[2]);
  return hexa;
}

// angle değeri 90 ise 90.00 şeklinde giriniz
void motorSend(float angle, String axis, float currentAngle){
  uint8_t message[10];
  message[0] = 0x3E;
  message[1] = 0xA5;
  if(axis == pitchMotor){
    message[2] = 0x01;
  }
  else if(axis == yawMotor){
    message[2] = 0x03;
  }
  else if(axis == rollMotor){
    message[2] = 0x08;
  }
  message[3] = 0x04;
  
  message[4] = 0;
  for(int i=0; i<4; i++){
    message[4] += message[i];
  }

  ///////////////////////////////////////////////////////
  if (angle < 180&&(currentAngle < 180)){
    if((angle - currentAngle) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
  else if (angle > 180&&(currentAngle > 180)) {
        if((angle - currentAngle) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
    else if ((angle > 180&&(currentAngle < 180))&&(angle>270)) {
        if((angle - currentAngle - 360.00) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
    else if ((angle > 180&&(currentAngle < 180))&&(angle<270)) {
        if((angle - currentAngle) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
    else if ((angle < 180&&(currentAngle > 180))&&(angle<90)) {
        if((angle - currentAngle + 360.00) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
    else if ((angle < 180&&(currentAngle > 180))&&(angle>90)) {
        if((angle - currentAngle) > 0){
      message[5] = 0x00;
    }
    else{
      message[5] = 0x01;
    }
  }
  /////////////////////////////////////////////////////////
  
  String angleHexa = decimalToHexa(angle);
  byte byteArray[2];

  for (int i = 0; i < angleHexa.length(); i += 2) {
  byteArray[i / 2] = strtoul(angleHexa.substring(i, i + 2).c_str(), NULL, 16);
  }
  
  message[6] = byteArray[0];
  message[7] = byteArray[1]; 
  
  message[8] = 0x00;

  message[9] = 0;
  for(int j=5; j<9; j++){
    message[9] += message[j]; 
  }

  rS485Serial.write(message, sizeof(message));
  //delay(10);
}

//////////////////////////////////////////////////////////////////////
// Math Library for atan2
#include "Math.h"

// Linear Algebra Library
#include "BasicLinearAlgebra.h"
using namespace BLA;

//////////////////////////////////////////////////////////////////////////////////
// MPU6050 Libraries//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
  // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
  // for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

  // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
  // is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

  // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
  // pitch/roll angles (in degrees) calculated from the quaternions coming
  // from the FIFO. Note this also requires gravity vector calculations.
  // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
  // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

/////////////////////////////////////////////////////////////////
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

  // MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

//*************************************************************************************
//*************************************************************************************
//*************************************************************************************
//GIMBAL VARIABLES*********************************************************************
// Define variables for General 
float psi;    // Yaw (Z) Axis angular value
float theta;  // Pitch Axis (Y) angular value
float gamma;  // Roll Axis (X) angular value
float alpha1_Desired; // BLDC Motor 1 (Yaw Axis)(Z) angular value
float alpha2_Desired; // BLDC Motor 2 (Pitch Axis)(Y) angular value
float alpha3_Desired; // BLDC Motor 3 (Roll Axis)(X) angular value

float palpha1_Desired = 0.00; // BLDC Motor 1 (Yaw Axis)(Z) angular value Previous
float palpha2_Desired = 0.00; // BLDC Motor 2 (Pitch Axis)(Z) angular value Previous
float palpha3_Desired = 0.00; // BLDC Motor 3 (Roll Axis)(Z) angular value Previous

// Define variables for INVERSE KINEMATICS
float targetYaw;   //Target yaw axis
float targetPitch; //Target pitch axis
BLA::Matrix<3,3> targetOrientation;   //Target orientation matrix
BLA::Matrix<3,3> targetOrientation_Base; //Target orientation wrt Base

BLA::Matrix<3,3> baseYaw;     //Base Yaw Axis Rotation Matrix
BLA::Matrix<3,3> basePitch;   //Base Pitch Axis Rotation Matrix
BLA::Matrix<3,3> baseRoll;    //Base Roll Axis Rotation Matrix

BLA::Matrix<3,3> baseYaw_t;     //Base Yaw Axis transpose Rotation Matrix
BLA::Matrix<3,3> basePitch_t;   //Base Pitch Axis transpose Rotation Matrix
BLA::Matrix<3,3> baseRoll_t;    //Base Roll Axis transpose Rotation Matrix

// ROTATION MATRIX GENERATION FUNCTIONS//////////////////////////
// Z-Axis Rotation Matrix
BLA::Matrix<3,3> rotate_Z_so3(double Qz){
  BLA::Matrix<3,3> Rz;
  Rz = {cos(Qz), -sin(Qz), 0,
        sin(Qz),  cos(Qz), 0,
              0,        0, 1};
  return Rz;
  }
// Y-Axis Rotation Matrix
BLA::Matrix<3,3> rotate_Y_so3(double Qy){
  BLA::Matrix<3,3> Ry;
  Ry = { cos(Qy), 0, sin(Qy),
               0, 1,       0,
        -sin(Qy), 0, cos(Qy)};
  return Ry;
  } 
// X-Axis Rotation Matrix
BLA::Matrix<3,3> rotate_X_so3(double Qx){
  BLA::Matrix<3,3> Rx;
  Rx = {1, 0, 0,
        0, cos(Qx),-sin(Qx),
        0, sin(Qx), cos(Qx)};
  return Rx;
  }
//*************************************************************************************
//*************************************************************************************
//*************************************************************************************

  
void setup() {
  
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////  
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); //timeout value in uSec  dehan:3000
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  rS485Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //*************************************************************************************
  //Serial.println(F("Initializing I2C devices..."));
  //*************************************************************************************
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //*************************************************************************************
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //*************************************************************************************

  //*************************************************************************************
  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //Commented for 
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-155);
  mpu.setYAccelOffset(1151);
  mpu.setZAccelOffset(741); // 1688 factory default for my test chip
  mpu.setXGyroOffset(65);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-9);
  //*************************************************************************************
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    //*************************************************************************************
    //Serial.println();
    //mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);


    //*************************************************************************************
    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  
  //Serial.println("Yaw:,Pitch:,Roll:"); //Legend


}
    
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void loop() {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    //Serial.print("YPR in degrees:--->\t");
    //Serial.print("Yaw:"); Serial.print("\t");
    //YAW AND PITCH VALUE REVERSED WITH MINUS SIGN
    //Serial.print("\t");
    Serial.print(-ypr[0] * 180 / M_PI);Serial.print("\t");
    //Serial.print("Pitch:"); Serial.print("\t");
    Serial.print(-ypr[1] * 180 / M_PI);Serial.print("\t");
    //Serial.print("Roll:"); Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);Serial.print("\t");

   
    // IMU to Gimbal connection******************************************************
    psi = -ypr[0]; //Reversed
    theta = -ypr[1]; //Reversed
    gamma = ypr[2];

    //*******************************************************************************

    targetYaw = 10.0*M_PI/180;
    targetPitch = -10.0*M_PI/180;
    //

    // Target Orientation Creation/////////////
    //        After 6 transformation this should be obtained
    targetOrientation = rotate_Z_so3(targetYaw)*rotate_Y_so3(targetPitch);
    //Serial << "targetOrientation: " << targetOrientation << '\n';

    // Base Rotation Matrices and their transpose
    baseYaw = rotate_Z_so3(psi);      //Base yaw(Z) rotation matrix
    baseYaw_t = ~baseYaw;             //  its transpose

    basePitch = rotate_Y_so3(theta);  //Base pitch(Y) rotation matrix
    basePitch_t = ~basePitch;         //  its transpose

    baseRoll = rotate_X_so3(gamma);   //Base roll(X) rotation matrix
    baseRoll_t = ~baseRoll;           //  its transpose

    // Representing Target Orientation wrt Base
    targetOrientation_Base = baseRoll_t*basePitch_t*baseYaw_t*targetOrientation;

    // Inverse Kinematics For BLDC Motor angles
    
    alpha1_Desired = atan2(targetOrientation_Base(1,0), targetOrientation_Base(0,0));
    alpha2_Desired = atan2(-targetOrientation_Base(2,0), sqrt(sq(targetOrientation_Base(0,0))+sq(targetOrientation_Base(1,0))));
    alpha3_Desired = atan2(targetOrientation_Base(2,1), targetOrientation_Base(2,2));

      // MATLAB GONDERME KISMI
//    Serial.print(alpha1_Desired * 180 / M_PI);Serial.print("\t");
//    Serial.print(alpha2_Desired * 180 / M_PI);Serial.print("\t");
//    Serial.print(alpha3_Desired * 180 / M_PI);Serial.print("\t");


    // Motor Control
    yawMotorDeg = alpha1_Desired*180/M_PI;
    pitchMotorDeg = alpha2_Desired*180/M_PI;
    rollMotorDeg = alpha3_Desired*180/M_PI;
            // For motor control negative values are corrected with 360 degree
    if (yawMotorDeg<0){yawMotorDeg = yawMotorDeg +360.00;}
    if (pitchMotorDeg<0){pitchMotorDeg = pitchMotorDeg +360.00;}
    if (rollMotorDeg<0){rollMotorDeg = rollMotorDeg +360.00;}
    
    //Serial.print(yawMotorDeg);Serial.print("\t");
    //Serial.print(pitchMotorDeg);Serial.print("\t");
    //Serial.println(rollMotorDeg);Serial.print("\t");
 
    
    motorSend(yawMotorDeg,yawMotor,palpha1_Desired);
    delay(1);
    motorSend(pitchMotorDeg,pitchMotor,palpha2_Desired);
    delay(1);
    motorSend(rollMotorDeg,rollMotor,palpha3_Desired);
    delay(1);


 
    palpha1_Desired = yawMotorDeg; //Previous angle assignment for upcoming loop
    palpha2_Desired = pitchMotorDeg; //Previous angle assignment for upcoming loop
    palpha3_Desired = rollMotorDeg; //Previous angle assignment for upcoming loop

 


    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
#endif 
}
} 
