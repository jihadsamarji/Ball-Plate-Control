/* MG 946R Servo Calibration
 by Jihad Alsamarji 
 This example code is in the public domain.

 modified 20 Mar 2019
 by Jihad Alsamarji
*/

/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo1, myservo2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos1 = 0;    // variable to store the servo position
int pos2 = 0;    // variable to store the servo position

void setup() {
  myservo1.attach(9,470, 2200);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10, 500, 2500);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {

  int sensorValue1 = analogRead(A0);
  //Serial.println(sensorValue1); 
  int input1 = map(sensorValue1,0,1024,135,45);
  Serial.println(input1);     
  int sensorValue2 = analogRead(A1);
  //Serial.println(sensorValue2); 
  int input2 = map(sensorValue2,0,1024,45,135);
  //Serial.println(input2);
  myservo1.write(input1);
  myservo2.write(input2);
  delay(10);
}
