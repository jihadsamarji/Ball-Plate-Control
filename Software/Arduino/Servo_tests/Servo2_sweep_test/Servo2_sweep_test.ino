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

Servo myservo;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int angleA = 0;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);
  myservo2.write(90);
    for (pos = 90; pos <= 170; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    angleA = map(pos,10,170,0,180);
    myservo.write(angleA);              // tell servo to go to position in variable 'pos'
    myservo2.write(angleA);
    delay(50);                       // waits 15ms for the servo to reach the position
  }

    for (pos = 170; pos >= 10; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    angleA = map(pos,10,170,0,180);
    myservo.write(angleA);              // tell servo to go to position in variable 'pos'
    myservo2.write(angleA);
    delay(50);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 10; pos <= 90; pos += 1) { // goes from 180 degrees to 0 degrees
    angleA = map(pos,10,170,0,180);
    myservo.write(angleA);              // tell servo to go to position in variable 'pos'
    myservo2.write(angleA);
    delay(50);                       // waits 15ms for the servo to reach the position
  }
}

void loop() {

}
