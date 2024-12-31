const int trigPin = 9; // TRIG pini
const int echoPin = 10; // ECHO pini

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration * 0.034 / 2; // Sesin hızı yaklaşık 34 cm/ms olduğu için hesap yapılır

  Serial.print("Uzaklık: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(1000); // 1 saniye bekle
}
