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

int readX()
{
   int X;
   pinMode(Y1,INPUT);
   pinMode(Y2,INPUT);  
   digitalWrite(Y2,LOW);
   pinMode(X1,OUTPUT);
   digitalWrite(X1,HIGH);
   pinMode(X2,OUTPUT);
   digitalWrite(X2,LOW);
   delay(10);
   X = analogRead(Y1);
   if (X != 0)
   {
    X = map(X, 40, 870 , - Xresolution / 2, Xresolution / 2); //Reads X axis touch position
   }
   return X;
}


int readY()
{
   int Y;
   pinMode(X1,INPUT);
   pinMode(X2,INPUT);
   digitalWrite(X2,LOW);
   pinMode(Y1,OUTPUT);
   digitalWrite(Y1,HIGH);
   pinMode(Y2,OUTPUT);
   digitalWrite(Y2,LOW);
   delay(10);
   Y = analogRead(X1);
   if (Y != 0){
    Y = map(Y, 70, 835 , - Yresolution / 2, Yresolution / 2); 
   }
   return Y;
}


void setup()
{
   Serial.begin(9600);
     

}

void loop()
{
 
  //Display X and Y on Serial Monitor
   Serial.print("X = ");  
   Serial.print(readX());
   Serial.print(" Y = ");
   Serial.println(readY());
}
