#include <Servo.h>

Servo servoA;
Servo servoB;


float angleA = 90;
float angleB = 90;


void setup() {
  Serial.begin(19200);
  servoA.attach(9);
  servoB.attach(10);
  delay(1000);
  angleA = map(angleA,20,160,0,180);
  servoA.write(angleA);
  angleB = map(angleB,20,160,0,180);
  servoB.write(angleB);
  delay(1000);
}

int count = 0;

void loop() {
  //digitalWrite(ledTemoin , millis() / 500 % 2 ); // led temoin clignotement
  if(Serial.available() > 0) {
    String a = Serial.readStringUntil('\n');
    //Serial.read();
    angleA = getValue(a, ',', 0).toFloat();  
    angleB = getValue(a, ',', 1).toFloat();      
    //angleA = map(angleA,10,170,0,180);
    //angleB = map(angleB,10,170,0,180);
    //angleA = map(angleA,20,160,0,180);
    servoA.write(angleA);
    //angleB = map(angleB,20,160,0,180);
    servoB.write(angleB);
    //delay(3);
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
