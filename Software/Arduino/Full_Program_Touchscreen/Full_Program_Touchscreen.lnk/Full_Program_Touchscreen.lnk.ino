
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
#include <time.h>

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
   delay(5);
   X += analogRead(Y1);
   if (X != 0)
   {
    X = map(X, 50, 860 , - Xresolution / 2, Xresolution / 2); //Reads X axis touch position
   }
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
   delay(5);
   Y = analogRead(X1);
   if (Y != 0){
    Y = map(Y,100, 820 , - Yresolution / 2, Yresolution / 2); 
   }
   return Y;
}


void setup()
{
   Serial.begin(19200);
  
   servoA.attach(10, 500, 2600);      //servo A
   servoB.attach(6, 500, 2600);    //servo B
   

}

float X = 0, Y= 0;
float prevX = 0 , prevY = 0;
float Kp = 0.035, Ki = 0.000, Kd = 0.015;
float totalErrorX = 0, totalErrorY = 0;
float prevErrorX = 0, prevErrorY=0;
float prevDerivX = 0, prevDerivY = 0;
float Cdx=0, Cdy=0;
int refX = 0, refY = 0;
float Ix = 0, Iy = 0;
float errorX = 0, errorY = 0;
float dc = 6.2 ,dm = 1.7 ;
float K = dc / dm;
float angleA = 90;
float angleB = 90;
float windupX = 0;
float windupY = 0;
float Ts = 0;
float Time = 0;
int N = 20;

void loop()
{
  Time = millis();
  //Display X and Y on Serial Monitor
   prevX = X;
   prevY = Y;
   prevDerivX = Cdx;
   prevDerivY = Cdy;
   prevErrorX = errorX;
   prevErrorY = errorY;

   X = readX();
   Y = readY();
   //X = 0.9048*prevX + 0.09516*X;
   //Y = 0.9048*prevY + 0.09516*Y;
   
//   for (cnt = 0; cnt < 5; cnt++){
//       X += readX();
//       Y += readY();
//       delay(2);
//   }

//   X = X/cnt;
//   Y = Y/cnt;
   
   errorX = refX - X;
   errorY = refY - Y;
   
   //windupX = Ki * (totalErrorX); // anti windupX
   
   //windupY = Ki * (totalErrorY); // anti windupY

   Cdx =  (Kd*N*(errorX-prevErrorX)+prevDerivX)/(1+N*Ts);
   Cdy =  (Kd*N*(errorY-prevErrorY)+prevDerivY)/(1+N*Ts);
   
   Ix = Kp * (errorX) + windupX + Cdx;
   Iy = Kp * (errorY) + windupY + Kd *(prevY - Y)/Ts;
   angleA = (K * Ix)+90;
   angleB = -(K * Iy)+90;


   if (angleA > 180) angleA = 180;
   if (angleA < 0) angleA = 0;
   if (angleB > 180) angleB = 180;
   if (angleB < 0) angleB = 0;
   servoA.write( angleA);
   servoB.write(angleB);

   totalErrorX += errorX;
   totalErrorY += errorY;
   
   Serial.print("X = ");  
   Serial.print(X);
   Serial.print(" ,Y = ");
   Serial.print(Y);
   Serial.print(" ,Ts = ");  
   Serial.println(Ts);
   //Serial.print(" ,errorX = ");  
   //Serial.print(errorX);
   //Serial.print(" ,errorY = ");
   //Serial.print(errorY);
   //Serial.print(" ,angleA = ");
   //Serial.print(angleA);
   //Serial.print(" ,angleB = ");
   //Serial.println(angleB);
   Ts = (millis() - Time)/1000;
}
