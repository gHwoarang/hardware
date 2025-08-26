void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
  int potValue = analogRead(A0);
  int mappedValue = map(potValue, 0, 1023, 0, 100);  // 100 arasında mapledım 100 u uce boldum
                                                     // led 1 icin
  if (mappedValue > 0) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }
  // led 2 ıcın
  if (mappedValue > 33) {
    analogWrite(5, mappedValue - 32);  // 33 ten sonra led 2 yı en bastan baslatması ıcın
  } else {
    analogWrite(5, 0);
  }
  // led 3 icin
  if (mappedValue > 66) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  delay(500);
  Serial.println("The brightness of LED 1=");
  if (mappedValue > 0) {
    Serial.println("%100");
  } else {
    Serial.println("%0");
  }
  delay(500);
  Serial.println("The brightness of LED 2=%");
  // led 2 ıcın bı duzenleme yaptım cheat gıbı
  if (mappedValue > 66) {
    Serial.println(mappedValue);
  } else if (mappedValue > 33) {
    Serial.println(mappedValue - 32);
  } else {
    Serial.println("0");
  }

  delay(500);
  Serial.println("The brightness of LED 3 =");
  if (mappedValue > 66) {
    Serial.println("%100");
  } else {
    Serial.println("%0");
  }
  // potansiyometre degerını yazdırır
  delay(500);
  Serial.println("The potantiometer value= ");
  Serial.println(potValue);
}
