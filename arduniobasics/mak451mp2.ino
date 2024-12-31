
#define trigPin 4  //trigpini sinyali tetikler
#define echoPin 2  // echo alır.
long sure, mesafe;
////////////////////
const int switchPin1 = 5;  // mıcro switchler icin
const int switchPin2 = 3;
int homePosition = 0;
int currentPosition = 0;
///////////////////
const int enA = 10;                // Hız kontrolü için PWM sinyali pin
const int in1 = 9;                 // Motor yön kontrolü için pin
const int in2 = 8;                 // Motor yön kontrolü için pin
const float wheelDiameter = 50.0;  // Tekerlek çapı (cm)
const int motorSpeed = 255;        // Motor hızı (0-255 arasında değer)



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //
  //////////////////////////////////////////////
  //HC-SR04 2-400 cm arasını olcer.Digital sinyalleri kullanır.Ses sinyali kullanır.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Arduino İle Mesafe Sensörü Uygulaması Başlatılıyor...");
  delay(3000);
  ////////////////////////////////////////////////7
  pinMode(switchPin1, INPUT);
  pinMode(switchPin2, INPUT);
  digitalWrite(switchPin1, HIGH);  // switchlerin dirençlerini etkinleştir
  digitalWrite(switchPin2, HIGH);
  ,
    /////////////////////////////////////////
    pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /////////////////////////////
  digitalWrite(trigPin, LOW);  //sinyal verişi durdur
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);  // sinyal verişi başlat
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  sure = pulseIn(echoPin, HIGH);
  mesafe = (sure / 2) * 0.0343;
  Serial.print(mesafe);
  Serial.println(" cm uzaklıkta homedan");
  delay(500);

  Serial.println("Please Write Desired Location in terms of cm");
  dLocation = Serial.read();
  dLocation- mesafe= motorMesafe;
  moveSystem(motorMesafe);

  //////////////////////////////////
  int switchState1 = digitalRead(switchPin1);
  int switchState2 = digitalRead(switchPin2);
  if (switchState1 == LOW) {  // birinci switch home olarak ayarlandı.
    homePosition = 0;
    currentPosition = 0;
  }

  if (switchState2 == LOW) {
    currentPosition = 100;  // Burada 100, sınıra ulaşıldığını temsil eden bir değerdir.
    // Ek olarak yapılması gereken işlemleri buraya ekleyin
  }
  if (currentPosition == 100 || currentPosition == 0) {
    stopMotor()  // Sistemi durdurmak için
  }


  void moveSystem(x) {
    // Motoru belirli bir süre çalıştırma
    float targetCircumference = wheelDiameter * PI;
    float targetSteps = x / targetCircumference;

    // Motoru belirli bir adım sayısı kadar çalıştırma
    for (int i = 0; i < targetSteps; i++) {
      digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, motorSpeed);
      // Adım başına beklenen süre (isteğe bağlı)
      delay(10);

      // Motoru durdurma
     
       stopMotor();

      // Adım sonrası bekleme süresi (isteğe bağlı)
      delay(100);
    }

    // Sistemi hareket ettirme tamamlandıktan sonra motoru durdurma
    stopMotor();
  }

  void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  }
