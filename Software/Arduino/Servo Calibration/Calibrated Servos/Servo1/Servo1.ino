/* MG 946R Servo Calibration
 by Jihad Alsamarji 
 This example code is in the public domain.

 modified 20 Mar 2019
 by Jihad Alsamarji
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 90;    // variable to store the servo position

void setup() {
  myservo.attach(9, 500, 2300);  // attaches the servo on pin 9 to the servo object with Calibrated values of pulse width (Nb: max angle is less than 180)
  //myservo.attach(9);  // attaches the servo on pin 9 to the servo object with default pulse width
  myservo.write(pos);
  delay(1000); 
  myservo.detach();
}

void loop() {

}
