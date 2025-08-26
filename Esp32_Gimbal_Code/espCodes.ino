/**

  TOBB ETU Mechanical Engineering Department Senior Design Project, 2022-2023 Spring

  Project Name: Digital twin approach to design and development of a Gimbal
  Partner Company: TEKNOPAR Industrial Automation Inc.
  Supervisor:  Assoc. Prof. Dr. Hakkı Özgür Ünver
  Company Supervisor: Muhammet Furkan Işık

  Code writen by:
  Okan Berhoğlu
  Ömer Dehan Ozboz

 */
#include <HardwareSerial.h>
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KickMath.h"
#include <WiFi.h>
/#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "BasicLinearAlgebra.h"
using namespace BLA;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

AsyncWebServer server(80);

// Network Credentials
const char* ssid = "OkanPC";
const char* password = "123456789";

void handleGetAll(AsyncWebServerRequest *request);
void handleSetTarget(AsyncWebServerRequest *request);
void handleSetPitchDownTargetTen(AsyncWebServerRequest *request);
void handleSetPitchUpTargetTen(AsyncWebServerRequest *request);
void handleSetYawRightTargetTen(AsyncWebServerRequest *request);
void handleSetYawLeftTargetTen(AsyncWebServerRequest *request);
void handleSetPitchDownTargetOne(AsyncWebServerRequest *request);
void handleSetPitchUpTargetOne(AsyncWebServerRequest *request);
void handleSetYawRightTargetOne(AsyncWebServerRequest *request);
void handleSetYawLeftTargetOne(AsyncWebServerRequest *request);
void handleSetPitchDownTargetFive(AsyncWebServerRequest *request);
void handleSetPitchUpTargetFive(AsyncWebServerRequest *request);
void handleSetYawRightTargetFive(AsyncWebServerRequest *request);
void handleSetYawLeftTargetFive(AsyncWebServerRequest *request);


float imuYaw;
float imuPitch;
float imuRoll;
float dtPitchMotor;
float dtYawMotor;
float dtRollMotor;
float setedTargetPitch = 0;
float setedTargetYaw = 0;

float yawMotorDeg;
float pitchMotorDeg;
float rollMotorDeg;

float palpha1_Desired = 220.00; // BLDC Motor 1 (Yaw Axis)(Z) angular value Previous
float palpha2_Desired = 215.00; // BLDC Motor 2 (Pitch Axis)(Z) angular value Previous
float palpha3_Desired = 90.00; // BLDC Motor 3 (Roll Axis)(Z) angular value Previous

String yawMotor = "yaw";
String pitchMotor = "pitch";
String rollMotor = "roll";

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
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

//GIMBAL VARIABLES*********************************************************************
// Define variables for General
float psi;    // Yaw (Z) Axis angular value
float theta;  // Pitch Axis (Y) angular value
float gama;  // Roll Axis (X) angular value
float alpha1_Desired; // BLDC Motor 1 (Yaw Axis)(Z) angular value
float alpha2_Desired; // BLDC Motor 2 (Pitch Axis)(Y) angular value
float alpha3_Desired; // BLDC Motor 3 (Roll Axis)(X) angular value

// Define variables for INVERSE KINEMATICS
float targetYaw;   //Target yaw axis
float targetPitch; //Target pitch axis

BLA::Matrix<3, 3> targetOrientation;  //Target orientation matrix
BLA::Matrix<3, 3> targetOrientation_Base; //Target orientation wrt Base

BLA::Matrix<3, 3> baseYaw;    //Base Yaw Axis Rotation Matrix
BLA::Matrix<3, 3> basePitch;  //Base Pitch Axis Rotation Matrix
BLA::Matrix<3, 3> baseRoll;   //Base Roll Axis Rotation Matrix

BLA::Matrix<3, 3> baseYaw_t;    //Base Yaw Axis transpose Rotation Matrix
BLA::Matrix<3, 3> basePitch_t;  //Base Pitch Axis transpose Rotation Matrix
BLA::Matrix<3, 3> baseRoll_t;   //Base Roll Axis transpose Rotation Matrix

// ROTATION MATRIX GENERATION FUNCTIONS//////////////////////////
// Z-Axis Rotation Matrix
BLA::Matrix<3, 3> rotate_Z_so3(double Qz) {
  BLA::Matrix<3, 3> Rz;
  Rz = {cos(Qz), -sin(Qz), 0,
        sin(Qz),  cos(Qz), 0,
        0,        0, 1
       };
  return Rz;
}
// Y-Axis Rotation Matrix
BLA::Matrix<3, 3> rotate_Y_so3(double Qy) {
  BLA::Matrix<3, 3> Ry;
  Ry = { cos(Qy), 0, sin(Qy),
         0, 1,       0,
         -sin(Qy), 0, cos(Qy)
       };
  return Ry;
}
// X-Axis Rotation Matrix
BLA::Matrix<3, 3> rotate_X_so3(double Qx) {
  BLA::Matrix<3, 3> Rx;
  Rx = {1, 0, 0,
        0, cos(Qx), -sin(Qx),
        0, sin(Qx), cos(Qx)
       };
  return Rx;
}

//////////////////////////////////////////////////////////////////////////////////

// angle değeri 90 ise 90.00 şeklinde giriniz
void motorSend(float angle, String axis, float currentAngle) {
  uint8_t message[10];
  message[0] = 0x3E;
  message[1] = 0xA5;
  if (axis == pitchMotor) {
    message[2] = 0x01;
  }
  else if (axis == yawMotor) {
    message[2] = 0x03;
  }
  else if (axis == rollMotor) {
    message[2] = 0x08;
  }
  message[3] = 0x04;

  message[4] = 0;
  for (int i = 0; i < 4; i++) {
    message[4] += message[i];
  }

  // 0x00 -> clockwise
  // 0x01 -> Counterclockwise
  if (angle < 180 && (currentAngle < 180)) {
    if ((angle - currentAngle) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }
  else if (angle > 180 && (currentAngle > 180)) {
    if ((angle - currentAngle) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }
  else if ((angle > 180 && (currentAngle < 180)) && (angle > 270)) {
    if ((angle - currentAngle - 360.00) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }
  else if ((angle > 180 && (currentAngle < 180)) && (angle < 270)) {
    if ((angle - currentAngle) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }
  else if ((angle < 180 && (currentAngle > 180)) && (angle < 90)) {
    if ((angle - currentAngle + 360.00) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }
  else if ((angle < 180 && (currentAngle > 180)) && (angle > 90)) {
    if ((angle - currentAngle) > 0) {
      message[5] = 0x00;
    }
    else {
      message[5] = 0x01;
    }
  }

  String angleHexa = decimalToHexa(angle);
  byte byteArray[2];

  for (int i = 0; i < angleHexa.length(); i += 2) {
    byteArray[i / 2] = strtoul(angleHexa.substring(i, i + 2).c_str(), NULL, 16);
  }

  message[6] = byteArray[0];
  message[7] = byteArray[1];

  message[8] = 0x00;

  message[9] = 0;
  for (int j = 5; j < 9; j++) {
    message[9] += message[j];
  }

  Serial.write(message, sizeof(message));
}

String decimalToHexa(float decimalnum) {
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

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  initWiFi();

  mpu.initialize();

  while (Serial.available() && Serial.read()); // empty buffer
  //Commented for
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(26);
  mpu.setYAccelOffset(1156);
  mpu.setZAccelOffset(740);
  mpu.setXGyroOffset(658);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-9);
  //*************************************************************************************

  if (devStatus == 0) {

    mpu.setDMPEnabled(true);

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  server.on("/getAll", HTTP_GET, handleGetAll);
  
  server.on("/setPitchUpTargetTen", HTTP_GET, handleSetPitchUpTargetTen);
  server.on("/setPitchDownTargetTen", HTTP_GET, handleSetPitchDownTargetTen);
  server.on("/setYawRightTargetTen", HTTP_GET, handleSetYawRightTargetTen);
  server.on("/setYawLeftTargetTen", HTTP_GET, handleSetYawLeftTargetTen);

  server.on("/setPitchUpTargetOne", HTTP_GET, handleSetPitchUpTargetOne);
  server.on("/setPitchDownTargetOne", HTTP_GET, handleSetPitchDownTargetOne);
  server.on("/setYawRightTargetOne", HTTP_GET, handleSetYawRightTargetOne);
  server.on("/setYawLeftTargetOne", HTTP_GET, handleSetYawLeftTargetOne);

  server.on("/setPitchUpTargetFive", HTTP_GET, handleSetPitchUpTargetFive);
  server.on("/setPitchDownTargetFive", HTTP_GET, handleSetPitchDownTargetFive);
  server.on("/setYawRightTargetFive", HTTP_GET, handleSetYawRightTargetFive);
  server.on("/setYawLeftTargetFive", HTTP_GET, handleSetYawLeftTargetFive);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();

  delay(1000);
  motorSend(220.00, yawMotor, 359.99);
  delay(1000);
  motorSend(215.00, pitchMotor, 359.99);
  delay(1000);
  motorSend(0.00, rollMotor, 90.00);
  delay(5000);
}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //Serial.print("YPR in degrees:--->\t");
    //Serial.print("Yaw:"); Serial.print("\t");
    //YAW AND PITCH VALUE REVERSED WITH MINUS SIGN
    //Serial.print("\t");
    //Serial.print(-ypr[0] * 180 / M_PI); Serial.print("\t");
    //Serial.print("Pitch:"); Serial.print("\t");
    //Serial.print(-ypr[1] * 180 / M_PI); Serial.print("\t");
    //Serial.print("Roll:"); Serial.print("\t");
    //Serial.println(ypr[2] * 180 / M_PI); Serial.print("\t");


    // IMU to Gimbal connection******************************************************
    psi = -ypr[0];
    theta = ypr[2];
    gama = ypr[1];

    targetYaw = setedTargetYaw * M_PI / 180;
    targetPitch = setedTargetPitch * M_PI / 180;

    // Target Orientation Creation/////////////
    targetOrientation = rotate_Z_so3(targetYaw) * rotate_Y_so3(targetPitch);

    // Base Rotation Matrices and their transpose
    baseYaw = rotate_Z_so3(psi);      //Base yaw(Z) rotation matrix
    baseYaw_t = ~baseYaw;             //  its transpose

    basePitch = rotate_Y_so3(theta);  //Base pitch(Y) rotation matrix
    basePitch_t = ~basePitch;         //  its transpose

    baseRoll = rotate_X_so3(gama);   //Base roll(X) rotation matrix
    baseRoll_t = ~baseRoll;           //  its transpose

    // Representing Target Orientation wrt Base
    targetOrientation_Base = baseRoll_t*basePitch_t*baseYaw_t*targetOrientation;

    // Inverse Kinematics For BLDC Motor angles

    alpha1_Desired = atan2(targetOrientation_Base(1, 0), targetOrientation_Base(0, 0));
    alpha2_Desired = -atan2(-targetOrientation_Base(2, 0), sqrt(sq(targetOrientation_Base(0, 0)) + sq(targetOrientation_Base(1, 0))));
    alpha3_Desired = atan2(targetOrientation_Base(2, 1), targetOrientation_Base(2, 2));

    dtPitchMotor = alpha2_Desired;
    dtYawMotor = alpha1_Desired;
    dtRollMotor = alpha3_Desired;

    alpha1_Desired = alpha1_Desired - (140 * M_PI / 180);
    alpha2_Desired = alpha2_Desired - (145 * M_PI / 180);
    alpha3_Desired = alpha3_Desired - (270 * M_PI / 180);

    // Motor Control
    yawMotorDeg = alpha1_Desired * 180 / M_PI;
    pitchMotorDeg = alpha2_Desired * 180 / M_PI;
    rollMotorDeg = alpha3_Desired * 180 / M_PI;

    // For motor control negative values are corrected with 360 degree
    if (yawMotorDeg < 0) {
      yawMotorDeg = yawMotorDeg + 360.00;
    }
    if (pitchMotorDeg < 0) {
      pitchMotorDeg = pitchMotorDeg + 360.00;
    }
    if (rollMotorDeg < 0) {
      rollMotorDeg = rollMotorDeg + 360.00;
    }

    //Serial.print(yawMotorDeg);Serial.print("\t");
    //Serial.print(pitchMotorDeg);Serial.print("\t");
    //Serial.println(rollMotorDeg);Serial.print("\t");

    motorSend(yawMotorDeg, yawMotor, palpha1_Desired);
    delay(100);
    motorSend(pitchMotorDeg, pitchMotor, palpha2_Desired);
    delay(100);
    motorSend(rollMotorDeg, rollMotor, palpha3_Desired);
    delay(100);

    // Previous angles
    palpha1_Desired = yawMotorDeg;
    palpha2_Desired = pitchMotorDeg;
    palpha3_Desired = rollMotorDeg;

    imuYaw = psi;
    imuPitch = theta;
    imuRoll = gama;

    Serial.print(setedTargetPitch);
    Serial.print(" ");
    Serial.print(dtPitchMotor * 180 / M_PI);
    Serial.print(" ");
    Serial.print(psi * 180 / M_PI);
    Serial.print(" ");
    Serial.print(theta * 180 / M_PI);
    Serial.print(" ");
    Serial.println(gama * 180 / M_PI);

  }
}

void handleGetAll(AsyncWebServerRequest *request) {

  DynamicJsonDocument jsonDoc(256);
  JsonObject jsonData = jsonDoc.to<JsonObject>();

  jsonData["rollMotor"] = dtRollMotor;
  jsonData["yawMotor"] = dtYawMotor;
  jsonData["pitchMotor"] = dtPitchMotor;
  jsonData["imuYaw"] = imuYaw;
  jsonData["imuRoll"] = imuRoll;
  jsonData["imuPitch"] = imuPitch;
  jsonData["pitchTarget"] = targetPitch * 180 / M_PI;
  jsonData["yawTarget"] = targetYaw * 180 / M_PI;

  String jsonString;
  serializeJson(jsonData, jsonString);

  request->send(200, "application/json", jsonString);
}

void handleSetPitchUpTargetTen(AsyncWebServerRequest *request) {
  setedTargetPitch += 10;
  request->send(200, "text/plain", "value updated");
}

void handleSetPitchDownTargetTen(AsyncWebServerRequest *request) {
  setedTargetPitch -= 10;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawRightTargetTen(AsyncWebServerRequest *request) {
  setedTargetYaw += 10;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawLeftTargetTen(AsyncWebServerRequest *request) {
  setedTargetYaw -= 10;
  request->send(200, "text/plain", "value updated");
}

void handleSetPitchUpTargetOne(AsyncWebServerRequest *request) {
  setedTargetPitch += 1;
  request->send(200, "text/plain", "value updated");
}

void handleSetPitchDownTargetOne(AsyncWebServerRequest *request) {
  setedTargetPitch -= 1;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawRightTargetOne(AsyncWebServerRequest *request) {
  setedTargetYaw += 1;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawLeftTargetOne(AsyncWebServerRequest *request) {
  setedTargetYaw -= 1;
  request->send(200, "text/plain", "value updated");
}
void handleSetPitchUpTargetFive(AsyncWebServerRequest *request) {
  setedTargetPitch += 5;
  request->send(200, "text/plain", "value updated");
}

void handleSetPitchDownTargetFive(AsyncWebServerRequest *request) {
  setedTargetPitch -= 5;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawRightTargetFive(AsyncWebServerRequest *request) {
  setedTargetYaw += 5;
  request->send(200, "text/plain", "value updated");
}

void handleSetYawLeftTargetFive(AsyncWebServerRequest *request) {
  setedTargetYaw -= 5;
  request->send(200, "text/plain", "value updated");
}
