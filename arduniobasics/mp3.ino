#include <Stepper.h>

const int potPin = A0; // Potansiyometrenin Analog 1 pinine bağlı olduğunu tanımlıyoruz.
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


int previousPotValue = 0; // Önceki potansiyometre değerini saklamak için bir değişken
void setup() {
  Serial.begin(9600); // Seri haberleşmeyi başlatıyoruz.
  myStepper.setSpeed(25); // Step motorun hızını ayarlıyoruz.
  Serial.begin(9600); // Seri haberleşmeyi başlatıyoruz.
  pinMode(buton1,INPUT);
pinMode(buton2,INPUT);

}

void loop() {

  int potValue = analogRead(potPin); // Potansiyometre değerini okuyoruz.
  int mappedValue = map(potValue, 0, 1023, 0, stepsPerRevolution); // Potansiyometre değerini adım sayısına dönüştürüyoruz.
  
  Serial.print("Potansiyometre degeri: ");
  Serial.println(potValue); // Potansiyometre değerini seri porta yazdırıyoruz.
  
  myStepper.step(mappedValue - previousPotValue); // Step motoru hareket ettiriyoruz (fark kadar)
  previousPotValue = mappedValue; // Önceki potansiyometre değerini güncelliyoruz.
  
  delay(100); // Bekleme süresi

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

