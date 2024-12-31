#include <Servo.h>

Servo myservo; // Servo nesnesi oluşturulur

void setup() {
  myservo.attach(5); // Servo motor 5 numaralı pine bağlı
  myservo.write(0); // pozisyon girdisi derece aci 
  delay(2000);
  myservo.write(108); // pozisyon girdisi derece aci 
  delay(1000);       // 1 saniye bekle
}

void loop() {
}
