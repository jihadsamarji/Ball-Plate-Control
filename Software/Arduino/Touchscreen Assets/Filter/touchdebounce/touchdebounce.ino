/*
 * Quick experiment to demonstrate touch screen debouncing/smoothing
 */

//#include <Elegoo_GFX.h>    // Core graphics library
//#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#define YP A2  // must be an analog pin, use "An" notation!
#define XM A1  // must be an analog pin, use "An" notation!
#define YM A3   // can be a digital pin
#define XP A0   // can be a digital pin

//Touch For New ILI9341 TP
#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70
#define TS_MAXY 920

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 50);




// Assign human-readable names to some common 16-bit color values:

int x = 0, y = 0, z = 0, trace_x = 0;




struct ExpFilter
{
  float k;
  int x_old;
  ExpFilter(float _k) { k = _k; x_old = 0; }

  void reset() { x_old = 0; }
  int filter(float x)
  {
    x_old = k * x + (1.0 - k) * x_old;
    return x_old;
  }
  
};

ExpFilter f(0.1);





void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  Serial.print("X = ");
  Serial.print(p.x);
  Serial.print(", Y = ");
  Serial.println(p.y);

  //delay(20);
}
