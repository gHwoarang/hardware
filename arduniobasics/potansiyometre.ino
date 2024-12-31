int pmetre=A1; //Potansiyometrenin Anolog 1 pininde oldugunu tanımlıyoruz. 
int deger=0; //Deger adında bir degişken tanımlıyoruz ve bu degeri 0 sayısına eşitliyoruz. 
void setup() {
  Serial.begin(9600); //Seri haberleşme Aktif hale getiriyoruz. 
}
void loop() {
  deger = analogRead(pmetre); //Deger adlı degişkeni potansiyometreden gelen degerlere tanımlıyoruz.
  Serial.print("Potansiyometreden gelen deger: "); // Serial ekrana Potansiyometreden gelen deger: yazısını yazdırıyoruz. 
  Serial.println(deger); //Deger degişkenini serial ekrana yazdırıyoruz. 
  delay(100); //Yukarıdaki işlemleri 100 milisaniyede bir tekrar etmesini sağlıyoruz. 
}

ardunio kartta hata olabilir 