#include "NewPing.h"// Include NewPing Library
#define TRIGGER_PIN 6 // Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define ECHO_PIN 7
#define MAX_DISTANCE 400  // Maximum distance we want to ping for (in centimeters).
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);// NewPing setup of pins and maximum distance.
float duration, distance;

#include <Stepper.h> 
const int stepsPerRevolution = 290; // Step motorun bir tam turda yapacağı adım sayısı
const int motorPin1 = 8; // Step motorun 1. pini
const int motorPin2 = 9; // Step motorun 2. pini
const int motorPin3 = 10; // Step motorun 3. pini
const int motorPin4 = 11; // Step motorun 4. pini
int buton2 = 2;                 //Butonu 7.pine bağladık.
int buton2durum = 0;            //Buton durumunu başlangıç için 0 yaptık. 
int buton1 = 3;              //Butonu 7.pine bağladık.
int buton1durum = 0;            //Buton durumunu başlangıç için 0 yaptık. 
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

float M1Value;
float differentialGap;
float desiredLocation;


void setup() 
{
  Serial.begin(9600);
  pinMode(buton1,INPUT);
  pinMode(buton2,INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN,INPUT);

Serial.println("Enter M1 value:");
  while (!Serial.available()) { }
  M1Value = Serial.parseFloat();

  Serial.println("Enter differential gap:");
  while (!Serial.available()) { }
  differentialGap = Serial.parseFloat();

  Serial.println("Enter desired location:");
  while (!Serial.available()) { }
  desiredLocation = Serial.parseFloat();

  
}

void loop() 
{
  distance = sonar.ping_cm();// Send ping, get distance in cm
  Serial.print("Mesafe = ");// Send results to Serial Monitor
  if (distance >= 400 || distance <= 2) 
  {
    Serial.println("Out of range");
  }
  else 
  {
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(500);

//BANGBANG
myStepper.setSpeed(30); // Step motorun hızını ayarlıyoruz.


if (desiredLocation-distance>0) 
  {
    if (desiredLocation-distance <differentialGap) {
    myStepper.step(0);// Stop the actuator when desired location is reached
  } else {
    // Apply Bang-Bang control based on the distance to the desired location
    if (distance < desiredLocation) {
     myStepper.step(M1Value); // Move towards the right (or increase location value)
    } else if(distance>desiredLocation) {
      myStepper.step(-M1Value ); // Move towards the left (or decrease location value)
    }
  }
  }
  else if( desiredLocation-distance<0)
  {
    if (desiredLocation-distance >-differentialGap) {
    myStepper.step(0);// Stop the actuator when desired location is reached
  } else {
    // Apply Bang-Bang control based on the distance to the desired location
    if (distance > desiredLocation) {
     myStepper.step(-M1Value); // Move towards the right (or increase location value)
    } else {
      myStepper.step(M1Value); // Move towards the left (or decrease location value)
    }
  }
  }


//SWITCH
buton1durum = digitalRead(buton1);    //Butonun bağlı olduğu port değerini butondurum değişkenine aktardık                  
buton2durum = digitalRead(buton2);    //Butonun bağlı olduğu port değerini butondurum değişkenine aktardık            
if ( buton2durum==1 ) {
    myStepper.step(0);
    Serial.println("arac 2 konumda");
  }
  if ( buton1durum==1 ) {
    myStepper.step(0);
    Serial.println("arac 1.konumda");
  }
}