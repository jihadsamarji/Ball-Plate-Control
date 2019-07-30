/* MG 946R Servo Calibration
 by Jihad Alsamarji 
 This example code is in the public domain.

 modified 20 Mar 2019
 by Jihad Alsamarji
*/


#include <time.h>
#include <Servo.h>

Servo servoA;  // create servo object to control a servo
Servo servoB;
// twelve servo objects can be created on most boards

float pos = 90;    // variable to store the servo position
int angleB = 135;

void setup() {
  pos = map(pos,20,160,0,180);
  //servoA.attach(9); 
  servoB.attach(10);
  delay(1000);
  servoB.write(angleB); 
  //servoA.write(pos);
  //delay(6000);
  //pos = map(96.8,20,160,0,180);
  //servoA.write(pos);
}

void loop() {

}
