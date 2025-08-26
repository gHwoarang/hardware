#include <ezButton.h>

ezButton limitSwitch(2);

#include "NewPing.h"   // Include NewPing Library
#define TRIGGER_PIN 5  // Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define ECHO_PIN 4
#define MAX_DISTANCE 19                              // Maximum distance we want to ping for (in centimeters).
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
float duration, distance;

#include <Stepper.h>
const int stepsPerRevolution = 290;  // Step motorun bir tam turda yapacağı adım sayısı
const int motorPin1 = 8;             // Step motorun 1. pini
const int motorPin2 = 9;             // Step motorun 2. pini
const int motorPin3 = 10;            // Step motorun 3. pini
const int motorPin4 = 11;            // Step motorun 4. pini

Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

float M1Value;
float differentialGap;
float desiredLocation;


void setup() {
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);

 
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Enter M1 value:");
  while (!Serial.available()) {}
  M1Value = Serial.parseFloat();

  Serial.println("Enter differential gap:");
  while (!Serial.available()) {}
  differentialGap = Serial.parseFloat();

  Serial.println("Enter desired location:");
  while (!Serial.available()) {}
  desiredLocation = Serial.parseFloat();
}

void loop() {
  //swıtch 
  limitSwitch.loop(); // MUST call the loop() function first
  if(limitSwitch.isPressed()){
    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
     myStepper.step(0);
    Serial.println("arac 1.konumda");
    
}
    
  //sonar 
  distance = sonar.ping_cm();  // Send ping, get distance in cm
  Serial.print("Mesafe = ");   // Send results to Serial Monitor
  if (distance >= 18 || distance <= 3) {
    Serial.println("Out of range");
    myStepper.step(0);
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(500);

  //BANGBANG
  myStepper.setSpeed(30);  // Step motorun hızını ayarlıyoruz.


  if (desiredLocation - distance > 0) {
    if (desiredLocation - distance < differentialGap) {
      myStepper.step(0);  // Stop the actuator when desired location is reached
    } else {
      // Apply Bang-Bang control based on the distance to the desired location
      if (distance < desiredLocation) {
        myStepper.step(-M1Value);  // Move towards the right (or increase location value)
      } else if (distance > desiredLocation) {
        myStepper.step(M1Value);  // Move towards the left (or decrease location value)
      }
    }
  } else if (desiredLocation - distance < 0) {
    if (desiredLocation - distance > -differentialGap) {
      myStepper.step(0);  // Stop the actuator when desired location is reached
    } else {
      // Apply Bang-Bang control based on the distance to the desired location
      if (distance > desiredLocation) {
        myStepper.step(M1Value);  // Move towards the right (or increase location value)
      } else {
        myStepper.step(-M1Value);  // Move towards the left (or decrease location value)
      }
    }
  }



  
}