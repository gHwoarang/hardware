/*
 *  PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS: https://www.youtube.com/channel/UCjiVhIvGmRZixSzupD0sS9Q
 *  Tutorial: http://electronoobs.com/eng_arduino_tut100.php
 *  Code: http://electronoobs.com/eng_arduino_tut100_code1.php
 *  Scheamtic: http://electronoobs.com/eng_arduino_tut100_sch1.php 
 *  3D parts: http://electronoobs.com/eng_arduino_tut100_stl1.php   
 */
#include <Wire.h>
#include <Servo.h>
///////////////////////Inputs/outputs///////////////////////
const int trigPin = 9; // TRIG pini
const int echoPin = 10; // ECHO pini
Servo myservo;  // create servo object to control a servo, later attatched to D9
////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////PID constants///////////////////////
float kp=1 ; //Mine was 8
float ki=1 ;//Mine was 0.2
float kd=10000; //Mine was 3100
float distance_setpoint = 19;    //Should be the distance from sensor to the middle of the bar in cm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////
void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object
  myservo.write(0); //Put the servo at angle 125, so the balance is in the middle
  delay(2000);
  myservo.write(108); //Put the servo at angle 125, so the balance is in the middle
  time = millis();
  delay(500);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  if (millis() > time+period)
  {
    time = millis();    
    distance = get_dist();   
    Serial.println(distance );

    //p icin hesap 
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    
    // d icin hesap 
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
    
    // i icin hesap
    //if(-3 < distance_error && distance_error < 3) // 
   // {
      PID_i = PID_i + (ki * distance_error);
   // }
    //else
   // {
     // PID_i = 0;
   // }


    //PID_total = PID_p ;// yalnizca p kontrolcu
    //PID_total = PID_p +  PID_d; // pd 
    //PID_total = PID_p + PID_i + ; // pi 
    PID_total = PID_p + PID_i + PID_d; // pid  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total+30); //derece angular location 
    distance_previous_error = distance_error;
  }
}

float get_dist()
{ long duration;
  float distance_cm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance_cm = duration * 0.034 / 2; // Sesin hızı yaklaşık 34 cm/ms olduğu için hesap yapılır
  return(distance_cm);
}
