int array[6];
int i = 0;
int sum = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Serial.println("If you  want lighten the led press 1 or to see sum enter 2 ");
  
while (Serial.available() == 0) {
  }
  int taken = Serial.read();
  
  switch (taken) {
    case 1:
      Serial.println("number to binary number ");
      while (Serial.available() == 0) {
  }
      int taken2 = Serial.read();
        
      if (taken2 >= 64) {
       
        int taken2 = Serial.read();
      }
      array[i] = taken2;
      if (taken2 % 2 == 0) {
        digitalWrite(2, 0);

      } else {
        digitalWrite(2, 1);
      }

      taken2 = taken2 / 2;

      if (taken2 % 2 == 1) {
        digitalWrite(3, 1);

      } else {
        digitalWrite(3, 0);
      }
      taken2 = taken2 / 2;
      if (taken2 % 2 == 1) {
        digitalWrite(4, 1);

      } else {
        digitalWrite(4, 0);
      }
      taken2 = taken2 / 2;
      if (taken2 % 2 == 1) {
        digitalWrite(5, 1);
      } else {
        digitalWrite(5, 0);
      }
      if (taken2 % 2 == 1) {
        digitalWrite(6, 1);

      } else {
        digitalWrite(6, 0);
      }
      taken2 = taken2 / 2;
      if (taken2 % 2 == 1) {
        digitalWrite(7, 1);
      } else {
        digitalWrite(7, 0);
      }

      i++;
    break;
    case 2:
      Serial.println("Here is the summation of numbers entered");
      for (i = 0; i < 5; i++) {
        sum = array[i] + sum;
        Serial.println(sum);
      }
      break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  
}
