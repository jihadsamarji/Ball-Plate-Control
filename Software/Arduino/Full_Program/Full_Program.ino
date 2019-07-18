#include <Servo.h>

Servo servoA;
Servo servoB;


int ledTemoin = 8;
float angleA = 90;
float angleB = 90;


void setup() {
  Serial.begin(19200);
  pinMode(ledTemoin, OUTPUT); // led temoin
  digitalWrite(ledTemoin,HIGH);
  servoA.attach(9, 500, 2300);      //servo A
  servoB.attach(10, 600, 2200);    //servo B

  delay(1000);
  servoA.write(angleA);
  servoB.write(angleB);
  delay(1000);
}

int count = 0;

void loop() {
  digitalWrite(ledTemoin , millis() / 500 % 2 ); // led temoin clignotement
  //Serial.println(getValue("135.0,90.0\n", ',', 1).toFloat()); //for testing
  if(Serial.available() > 0) {
    String a = Serial.readStringUntil('\n');
    if(a == "compactPlate"){
      angleA = 0;
      angleB = 0;
    }else{
      //a.remove(0,1);
      //a.remove(a.length() - 1,1); 
      angleA = getValue(a, ',', 0).toFloat();  
      angleB = getValue(a, ',', 1).toFloat();      
    }
    servoA.write(angleA);
    servoB.write(angleB);
    delay(5);
  }

}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
