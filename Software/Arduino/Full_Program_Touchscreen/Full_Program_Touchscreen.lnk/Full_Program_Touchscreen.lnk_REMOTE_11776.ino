
/*=================================
This code demostrates 4-Wire Touch screen 
interfacing with Arduino

blog.circuits4you.com
www.circuits4you.com

4- Wire Touchscreen Connections
A0=====X+
A1=====X-
A2=====Y+
A3=====Y-

Make sure you put pull-down resistors (10Kohms) on A0 and A2
=================================*/
//Define your Touch screen connections
#define X1 A0
#define X2 A1
#define Y1 A2
#define Y2 A3
//Define your screen resolution as per your Touch screen (Max: 1024)
#define Xresolution 234 //128
#define Yresolution 175 //64

#include <Servo.h>

Servo servoA;
Servo servoB;

int readX()
{
   float X = 0;
   pinMode(Y1,INPUT);
   pinMode(Y2,INPUT);  
   digitalWrite(Y2,LOW);
   pinMode(X1,OUTPUT);
   digitalWrite(X1,HIGH);
   pinMode(X2,OUTPUT);
   digitalWrite(X2,LOW);
   delay(2);
   X += analogRead(Y1);
//   if (X != 0)
//   {
//    X = map(X, 40, 870 , - Xresolution / 2, Xresolution / 2); //Reads X axis touch position
//   }
   return X;
}


int readY()
{
   int Y = 0;
   pinMode(X1,INPUT);
   pinMode(X2,INPUT);
   digitalWrite(X2,LOW);
   pinMode(Y1,OUTPUT);
   digitalWrite(Y1,HIGH);
   pinMode(Y2,OUTPUT);
   digitalWrite(Y2,LOW);
   delay(2);
   Y = analogRead(X1);
//   if (Y != 0){
//    Y = map(Y, 70, 835 , - Yresolution / 2, Yresolution / 2); 
//   }
   return Y;
}


void setup()
{
   Serial.begin(9600);
  
   servoA.attach(9, 500, 2300);      //servo A
   servoB.attach(10, 600, 2200);    //servo B
   

}

float X = 346, Y= 306;
float prevX = 0 , prevY = 0;
float Kp = 0.025, Ki = 0.001, Kd = 0.15;
float totalErrorX = 0, totalErrorY = 0;
int refX = 450, refY = 450;
float Ix = 0, Iy = 0;
float errorX = 0, errorY = 0;
float dc = 6.2 ,dm = 1.7 ;
float K = dc / dm;
float angleA = 90;
float angleB = 90;
float windupX = 0;
float windupY = 0;

void loop()
{
  
  //Display X and Y on Serial Monitor
   prevX = X;
   prevY = Y;
   X = 0;
   Y = 0;
   //int cnt ;

   X = readX();
   if ((X - prevX > 300) || (X - prevX < -300) || ((X - prevX < 10) && (X - prevX > -10))) X = prevX;
   Y = readY();
   if ((Y - prevY > 300) || (Y - prevY < -300) || ((Y - prevY < 10) && (Y - prevY > -10))) Y = prevY;
   
//   for (cnt = 0; cnt < 5; cnt++){
//       X += readX();
//       Y += readY();
//       delay(2);
//   }

//   X = X/cnt;
//   Y = Y/cnt;
   
   errorX = refX - X;
   errorY = refY - Y;
   
   windupX = Ki * (totalErrorX); // anti windupX
   
   windupY = Ki * (totalErrorY); // anti windupY
   
   Ix = Kp * (errorX) + windupX + Kd * (prevX - X);
   Iy = Kp * (errorY) + windupY + Kd * (prevY - Y);
   angleA = K * Ix;
   angleB = K * Iy;
   angleA = map(angleA, -80, 80 , 10, 170);
   angleB = map(angleB, -80, 80 , 10, 170);

   if (angleA > 170) angleA = 170;
   if (angleA < 10) angleA = 10;
   if (angleB > 170) angleB = 170;
   if (angleB < 10) angleB = 10;
   servoA.write( angleA);
   servoB.write( angleB);

   totalErrorX += errorX;
   totalErrorY += errorY;
   if (totalErrorX > 500) totalErrorX = 500;
   if (totalErrorX < -500) totalErrorX = -500;
   if (totalErrorY > 500) totalErrorY = 500;
   if (totalErrorY < -500) totalErrorY = -500;
   
   Serial.print("X = ");  
   Serial.print(X);
   Serial.print(" ,Y = ");
   Serial.println(Y);
   //Serial.print(" ,Ix = ");  
   //Serial.print(Ix);
   //Serial.print(" ,Iy = ");
   //Serial.print(Iy);
   //Serial.print(" ,angleA = ");
   //Serial.print(angleA);
   //Serial.print(" ,angleB = ");
   //Serial.println(angleB);
   
}
