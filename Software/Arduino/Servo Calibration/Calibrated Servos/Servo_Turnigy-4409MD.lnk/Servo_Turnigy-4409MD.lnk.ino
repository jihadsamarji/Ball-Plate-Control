/* MG 946R Servo Calibration
 by Jihad Alsamarji 
 This example code is in the public domain.

 modified 20 Mar 2019
 by Jihad Alsamarji
*/



#include <Servo.h>

Servo servoA;  // create servo object to control a servo
Servo servoB;
// twelve servo objects can be created on most boards

int angleA = 90;    // variable to store the servo position
int angleB = 90;

void setup() {
  //angleA = map(angleA,20,160,0,180);
  //angleB = map(angleB,20,160,0,180);
  servoA.attach(10,500,2600); 
  servoB.attach(6,500,2600);
  servoA.write(0);
  servoB.write(angleB);
  delay(5000);
  servoA.write(90);
  delay(50);
  servoA.write(180);

}

void loop() {

}
