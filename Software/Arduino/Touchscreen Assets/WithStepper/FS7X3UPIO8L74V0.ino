/*
  Matthew M
  Ball on plat with 3 DOF
  Mega 2560

  microsteps: 32
  motor_steps_per_rev: 200
  -----------------------------------------------------------------
*/
#include "RunningMedian.h"
#include "RunningAverage.h"

RunningMedian samples_x = RunningMedian(10);
RunningMedian samples_y = RunningMedian(10);

RunningMedian samples_angle_x = RunningMedian(8); //10
RunningMedian samples_angle_y = RunningMedian(8); //10

RunningAverage average_a(15); //15
RunningAverage average_b(15); //15
RunningAverage average_c(15); //15

RunningAverage ACC_x(3);
RunningAverage ACC_y(3);

# define cycle_time 5 //run loop every ___ ms



bool compute = 0;
bool fast_compute = 0;
uint16_t  old_micros;
uint16_t  compute_time;

uint8_t loop_count;

//-----------------------------------------------------------
//interface knob---------------------------------------------
//---------------------------------------------------------
# define knob_a_pin 32
# define knob_b_pin 47
# define knob_btn_pin 45

uint8_t knob_a_last = 0;
uint8_t knob_a = 0;

uint8_t knob_b_last = 0;
uint8_t knob_b = 0;

uint8_t knob_btn = 1;
uint8_t knob_btn_last = 1;

uint8_t select_mode = 0;
float hold_count = 0;

uint8_t pattern = 0;
uint8_t pattern_rate = 20;
uint8_t pattern_rate_counter = 0;
uint16_t pattern_counter = 0;

bool loop_direction = 0;

int8_t sin_LUT[256] = {0,  3,  6,  9,  12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88, 90, 92, 94, 96, 98, 100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116,  117,  118,  120,  121,  122,  122,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127,  127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  122,  122,  121,  120,  118,  117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,  98, 96, 94, 92, 90, 88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 49, 46, 43, 40, 37, 34, 31, 28, 25, 22, 19, 16, 12, 9,  6,  3,  0,  -3, -6, -9, -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46,  -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88,  -90,  -92,  -94,  -96,  -98,  -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116, -117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -122, -122, -121, -120, -118, -117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100, -98,  -96,  -94,  -92,  -90,  -88,  -85,  -83,  -81,  -78,  -76,  -73,  -71,  -68,  -65,  -63,  -60,  -57,  -54,  -51,  -49,  -46,  -43,  -40,  -37,  -34,  -31,  -28,  -25,  -22,  -19,  -16,  -12,  -9, -6, -3};
int8_t cos_LUT[256] = {127, 127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  122,  122,  121,  120,  118,  117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,  98, 96, 94, 92, 90, 88, 85, 83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 49, 46, 43, 40, 37, 34, 31, 28, 25, 22, 19, 16, 12, 9,  6,  3,  0,  -3, -6, -9, -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46,  -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88,  -90,  -92,  -94,  -96,  -98,  -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116, -117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127, -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -122, -122, -121, -120, -118, -117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100, -98,  -96,  -94,  -92,  -90,  -88,  -85,  -83,  -81,  -78,  -76,  -73,  -71,  -68,  -65,  -63,  -60,  -57,  -54,  -51,  -49,  -46,  -43,  -40,  -37,  -34,  -31,  -28,  -25,  -22,  -19,  -16,  -12,  -9, -6, -3, 0,  3,  6,  9,  12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88, 90, 92, 94, 96, 98, 100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116,  117,  118,  120,  121,  122,  122,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127};


//-----------------------------------------------------------------
//touch screen stuff
//-----------------------------------------------------------------
//     test screen at A,B,C,D,E
//
//      - - -T- - -       Y
//     |-|-|-|-|-|-|     /|
//     L- - -C- - -R      | -->X
//     |-|-|-|-|-|-|
//      - - -B- - -
// 
//enter values from measured_x_pos and measured_y_pos for initial screen calibration
//uncomment "Serial.print(measured_x_pos) Serial.println(measured_y_pos)" at bottom of main loop to display values


# define test_C_x 493
# define test_C_y 520

# define test_L_x 900
# define test_R_x 89

# define test_T_y 835
# define test_B_y 205

#define x_5v A0
#define x_gnd A1
#define y_5v  A2
#define y_gnd  A3

int16_t ball_x_pos;
int16_t ball_y_pos;
int16_t measured_x_pos;
int16_t corrected_x_pos;
int16_t measured_y_pos;
int16_t corrected_y_pos;

int8_t x_offset = 0;
int8_t y_offset = 0;

bool touch = 0;
int8_t detect_touch_filter = 0;
//----------------------------------------------------------------------
//PID stuff for ball position
//----------------------------------------------------------------------
int16_t set_x_pos = -300;
int16_t set_y_pos = 0;

int16_t x_pos_array [79];
int16_t y_pos_array [79];

float x_avg [16];
float y_avg [16];

int16_t FPGA_x_angle = 0;
int16_t FPGA_y_angle = 0;

float p_gain = 0.35;
float d_gain = 2.0;//550
float i_gain = 0.00;

float error_x = 0;
float error_y = 0;

float i_x = 0;
float i_y = 0;

float d_x = 0;
float d_y = 0;

float d_x_avg = 0;
float d_y_avg = 0;

float d_x_pos = 0;
float d_x_neg = 0;
float d_y_pos = 0;
float d_y_neg = 0;

int i;
int j;
//----------------------------------------------------------------------
//PID stuff for stepper motors
//----------------------------------------------------------------------
#include "PID_v1.h"

double set_angle_a, set_angle_b, set_angle_c;  //set values to PID
double measured_angle_a, measured_angle_b, measured_angle_c;  //actual measured values to send to PID
double set_speed_a, set_speed_b, set_speed_c;  //set speed from PID

double Kp = 0.03;  //proportional gain
double Ki = 0.0;
double Kd = 0.0;

#define max_speed 4  //max output value from PID
#define min_speed -4  //min output value from PID
#define PID_sample_time 10  //how often the PID algorithm evaluates in ms
#define deadband 3        // +/- number of counts to not respond 6

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID PID_a(&measured_angle_a, &set_speed_a, &set_angle_a, Kp, Ki, Kd, DIRECT);
PID PID_b(&measured_angle_b, &set_speed_b, &set_angle_b, Kp, Ki, Kd, DIRECT);
PID PID_c(&measured_angle_c, &set_speed_c, &set_angle_c, Kp, Ki, Kd, DIRECT);

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void setup()
{
  Serial.begin(230400);
  PID_setup();

  timer_setup();
  touch_screen_setup();


  set_angle_a = 0;
  set_angle_b = 0;
  set_angle_c = 0;
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void loop()
{

  if (fast_compute == 1)
  {
    ball_patterns_and_display();
    fast_compute = 0;
  }



  if (compute == 1)
  {
    old_micros = micros();

    touch_screen();

    ball_PID();

    samples_angle_x.add(FPGA_x_angle);
    samples_angle_y.add(FPGA_y_angle);

    set_angle_a = 450.0 + samples_angle_y.getMedian();
    set_angle_b = 450.0 - float(samples_angle_y.getMedian()) * 0.52 + float(samples_angle_x.getMedian()) * 0.866;
    set_angle_c = 450.0 - float(samples_angle_y.getMedian()) * 0.52 - float(samples_angle_x.getMedian()) * 0.866;


    average_a.addValue(set_angle_a);
    set_angle_a = average_a.getAverage();
    average_b.addValue(set_angle_b);
    set_angle_b = average_b.getAverage();
    average_c.addValue(set_angle_c);
    set_angle_c = average_c.getAverage();

    //limit movement of motors
    if (set_angle_a < 5) set_angle_a = 5;
    else if (set_angle_a > 900) set_angle_a = 900;
    else set_angle_a = set_angle_a;

    if (set_angle_b < 5) set_angle_b = 5;
    else if (set_angle_b > 900) set_angle_b = 900;
    else set_angle_b = set_angle_b;

    if (set_angle_c < 5) set_angle_c = 5;
    else if (set_angle_c > 900) set_angle_c = 900;
    else set_angle_c = set_angle_c;

    calc_PID();






    //display values based on selected variable
    switch (select_mode)
    {
      case 1:
        Serial.print("kp ");
        Serial.println(p_gain);
        break;
      case 2:
        Serial.print("kd ");
        Serial.println(d_gain);
        break;
      case 3:
        Serial.print("ki ");
        Serial.println(i_gain);
        break;
      case 4:
        Serial.print(x_offset);
        Serial.println(" x offset");
        break;
      case 5:
        Serial.print(y_offset);
        Serial.println(" y offset");
        break;
      case 6:
        Serial.print(pattern);
        Serial.println(" pattern");
        break;
      case 7:
        Serial.print(pattern_rate);
        Serial.println(" pattern rate");
        break;
      case 8:
        Serial.print(loop_direction);
        Serial.println(" pattern direction");
        break;

      default:
        Serial.print(compute_time);
        Serial.println(" uS");
        break;
    }


    compute = 0;
    compute_time = micros() - old_micros;
  }
  else
  {
    //uncomment for screen calibration
    Serial.print(measured_x_pos);
    Serial.println(measured_y_pos);
  }
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

//----------------------------------------------------------------------
//quadrature control knob
//----------------------------------------------------------------------
void enc_knob_setup()
{
  pinMode (knob_a_pin, INPUT_PULLUP);
  pinMode (knob_b_pin, INPUT_PULLUP);
  pinMode (knob_btn_pin, INPUT_PULLUP);
}

float enc_knob(float count)
{
  knob_a = digitalRead(knob_a_pin);
  knob_b = digitalRead(knob_b_pin);
  knob_btn = digitalRead(knob_btn_pin);

  if ((knob_btn == LOW) && (knob_btn_last == HIGH))
  {
    if (select_mode < 8) select_mode = select_mode + 1;
    else select_mode = 0;
  }

  if ((knob_a_last == LOW) && (knob_a == HIGH))
  {
    if (knob_b == LOW)
    {
      count += 0.01;
    }
    else
    {
      count -= 0.01;
    }
  }
  else if ((knob_b_last == HIGH) && (knob_b == LOW))
  {
    if (knob_a == LOW)
    {
      count += 0.01;
    }
    else
    {
      count -= 0.01;
    }
  }

  knob_a_last = knob_a;
  knob_b_last = knob_b;
  knob_btn_last = knob_btn;
  return count;
}


//----------------------------------------------------------------------
//PID calculations for ball position
//----------------------------------------------------------------------
void ball_PID()
{
  error_x = (set_x_pos - corrected_x_pos) ;
  error_y = (set_y_pos - corrected_y_pos) ;

  //fill ball history array
  for (i = 0; i < 79; i ++)
  {
    x_pos_array [i + 1] = x_pos_array [i];
    y_pos_array [i + 1] = y_pos_array [i];
  }
  x_pos_array [0] = corrected_x_pos;
  y_pos_array [0] = corrected_y_pos;

  /*
    for (i = 0; i < 16; i ++)
    {
      x_avg[i] = float(x_pos_array [5 * i] + x_pos_array [5 * i + 1] + x_pos_array [5 * i + 2] + x_pos_array [5 * i + 3] + x_pos_array [5 * i + 4])/5.0;
      y_avg[i] = float(y_pos_array [5 * i] + y_pos_array [5 * i + 1] + y_pos_array [5 * i + 2] + y_pos_array [5 * i + 3] + y_pos_array [5 * i + 4])/5.0;
    }
  */


  for (i = 0; i < 16; i ++)
  {
    x_avg[i] = x_pos_array [i * 5]; //5
    y_avg[i] = y_pos_array [i * 5]; //5
  }

  d_x_pos = 322.0 * x_avg[0] + 217.0 * x_avg[1] + 110.0 * x_avg[2] + 35.0 * x_avg[3] + 25.0 * x_avg[13] + 98.0 * x_avg[14] + 203.0 * x_avg[15];
  d_y_pos = 322.0 * y_avg[0] + 217.0 * y_avg[1] + 110.0 * y_avg[2] + 35.0 * y_avg[3] + 25.0 * y_avg[13] + 98.0 * y_avg[14] + 203.0 * y_avg[15];

  d_x_neg = -42.0 * x_avg[4] - 87.0 * x_avg[5] - 134.0 * x_avg[6] - 149.0 * x_avg[7] - 166.0 * x_avg[8] - 151.0 * x_avg[9] - 138.0 * x_avg[10] - 93.0 * x_avg[11] - 50.0 * x_avg[12];
  d_y_neg = -42.0 * y_avg[4] - 87.0 * y_avg[5] - 134.0 * y_avg[6] - 149.0 * y_avg[7] - 166.0 * y_avg[8] - 151.0 * y_avg[9] - 138.0 * y_avg[10] - 93.0 * y_avg[11] - 50.0 * y_avg[12];

  d_x = (d_x_pos + d_x_neg) * d_gain / 28.56;
  d_y = (d_y_pos + d_y_neg) * d_gain / 28.56;

  //d_x = ((322.0 * x_avg[0] + 217.0 * x_avg[1] + 110.0 * x_avg[2] + 35.0 * x_avg[3] + 25.0 * x_avg[13] + 98.0 * x_avg[14] + 203.0 * x_avg[15] - 42.0 * x_avg[4] - 87.0 * x_avg[5] - 134.0 * x_avg[6] - 149.0 * x_avg[7] - 166.0 * x_avg[8] - 151.0 * x_avg[9] - 138.0 * x_avg[10] - 93.0 * x_avg[11] - 50.0 * x_avg[12]) * d_gain) / 2856.0;
  //d_y = ((322.0 * y_avg[0] + 217.0 * y_avg[1] + 110.0 * y_avg[2] + 35.0 * y_avg[3] + 25.0 * y_avg[13] + 98.0 * y_avg[14] + 203.0 * y_avg[15] - 42.0 * y_avg[4] - 87.0 * y_avg[5] - 134.0 * y_avg[6] - 149.0 * y_avg[7] - 166.0 * y_avg[8] - 151.0 * y_avg[9] - 138.0 * y_avg[10] - 93.0 * y_avg[11] - 50.0 * y_avg[12]) * d_gain) / 2856.0;



  //ACC_x.addValue(d_x);
  //ACC_y.addValue(d_y);

  d_x_avg = .5 * d_x + .5 * d_x_avg;
  d_y_avg = .5 * d_y + .5 * d_y_avg;


  if (error_x < 200 && error_x > -200) i_x += i_gain * error_x / 10.0;
  else i_x = 0;
  if (error_y < 200 && error_y > -200) i_y += i_gain * error_y / 10.0;
  else i_y = 0;


  if (i_gain == 0)
  {
    i_x = 0;
    i_y = 0;
  }

  if (i_x > 400.0) i_x = 400.0;
  else if (i_x < -400.0) i_x = -400.0;

  if (i_y > 400.0) i_y = 400.0;
  else if (i_y < -400.0) i_y = -400.0;

  FPGA_x_angle = error_x * p_gain + i_x - d_x_avg + x_offset;
  FPGA_y_angle = error_y * p_gain + i_y - d_y_avg + y_offset;


}

//----------------------------------------------------------------------
//PID calculations for stepper motors
//----------------------------------------------------------------------
void PID_setup()
{
  PID_a.SetMode(AUTOMATIC);
  PID_b.SetMode(AUTOMATIC);
  PID_c.SetMode(AUTOMATIC);

  PID_a.SetOutputLimits(min_speed, max_speed);
  PID_b.SetOutputLimits(min_speed, max_speed);
  PID_c.SetOutputLimits(min_speed, max_speed);

  PID_a.SetSampleTime(PID_sample_time);
  PID_b.SetSampleTime(PID_sample_time);
  PID_c.SetSampleTime(PID_sample_time);
}

void calc_PID()
{


  PID_a.Compute();
  PID_b.Compute();
  PID_c.Compute();

  //add deadbamd that the motors do nothing
  if ( measured_angle_a - set_angle_a < deadband && measured_angle_a - set_angle_a > -deadband) set_speed_a = 0;
  if ( measured_angle_b - set_angle_b < deadband && measured_angle_b - set_angle_b > -deadband) set_speed_b = 0;
  if ( measured_angle_c - set_angle_c < deadband && measured_angle_c - set_angle_c > -deadband) set_speed_c = 0;
}

//-------------------------------------------------------------------------
// touch screen measurements
//--------------------------------------------------------------------------
void touch_screen()
{
  detect_touch();  //is the screen being touched?

  //touch = 1;

  if (touch == 0)  //set position to (0,0) if not touched
  {
    ball_x_pos = test_C_x;
    ball_y_pos = test_C_y;
  }

  if (touch == 1) measure_touch();  //measure touch position if touched

  samples_x.add(ball_x_pos);
  samples_y.add(ball_y_pos);

  measured_x_pos = samples_x.getMedian();
  measured_y_pos = samples_y.getMedian();

  //normalize screen position to 0,0 at center and each inch is 100 count
  corrected_x_pos = measured_x_pos * 1200.0 / (test_R_x - test_L_x)  - test_C_x * 1200.0 / (test_R_x - test_L_x);
  corrected_y_pos = measured_y_pos * 800.0 / (test_T_y - test_B_y)   - test_C_y * 800.0 / (test_T_y - test_B_y);
}

void touch_screen_setup()
{
  //setup for "detect_touch()"
  pinMode(x_5v,  INPUT_PULLUP);
  pinMode(x_gnd,  INPUT_PULLUP);
  pinMode(y_5v,  INPUT);
  pinMode(y_gnd,  OUTPUT);

  digitalWrite(y_gnd, LOW);
}

void detect_touch()
{
  //debounce touch detection
  if (digitalRead(x_5v))
  {
    detect_touch_filter += 1;
    if (detect_touch_filter >= 3)
    {
      detect_touch_filter = 3;
      touch = 0;
    }
  }
  else
  {
    detect_touch_filter -= 1;
    if (detect_touch_filter <= -3)
    {
      detect_touch_filter = -3;
      touch = 1;
    }
  }
}

void measure_touch()
{
  //read x-position
  pinMode(x_5v,  OUTPUT);
  pinMode(x_gnd,  OUTPUT);
  pinMode(y_5v,  INPUT);
  pinMode(y_gnd,  INPUT);

  digitalWrite(x_5v, HIGH);
  digitalWrite(x_gnd, LOW);

  ball_x_pos = analogRead(y_5v);

  //read y-position
  pinMode(x_5v,  INPUT);
  pinMode(x_gnd,  INPUT);
  pinMode(y_5v,  OUTPUT);
  pinMode(y_gnd,  OUTPUT);

  digitalWrite(y_5v, HIGH);
  digitalWrite(y_gnd, LOW);

  ball_y_pos = analogRead(x_5v);

  //setup for "detect_touch()"
  pinMode(x_5v,  INPUT_PULLUP);
  pinMode(x_gnd,  INPUT_PULLUP);
  pinMode(y_5v,  INPUT);
  pinMode(y_gnd,  OUTPUT);

  digitalWrite(y_gnd, LOW);
}

//-----------------------------------------------------------------------
//loop timing stuff
//------------------------------------------------------------------------
void timer_setup()
{
  // Timer0 is already used for "millis()", just interrupt somewhere
  // in the middle to toggle the "compute" values
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

//timer interrupt from timer0 every ~1ms
SIGNAL(TIMER0_COMPA_vect)
{
  fast_compute = 1;
  if (loop_count < cycle_time) loop_count++;
  else
  {
    compute = 1;
    loop_count = 0;
  }
}

//------------------------------------------------------------------------
//stuff to control the stepper motors
//------------------------------------------------------------------------
void stepper_setup()
{




  cli();//stop interrupts



  sei(); // Start Interrupt

}

void stepper_run( uint8_t motorNumber, float motor_speed)
{
  bool dir;  //motor direction
  uint16_t PWM_count;

  //determine needed motor direction and make speed pos.
  if (motor_speed < 0)
  {
    dir = 0;
    motor_speed = abs(motor_speed);
  }
  else dir = 1;

  if (motor_speed > 0)
  {
    PWM_count = 5000 / motor_speed;  //calculate needed compare value for speed
  }
  else PWM_count = 0;  //set compare value to 0 to keep stepper output from toggling when zero velocity needed



}

void motor_power(bool power)
{
  if (power == 1)
  {

  }
  else
  {

  }
}

//-------------------------------------------------------------------------------
//ball patterns!!!
//------------------------------------------------------------------------------

void ball_patterns_and_display()
{
  if (pattern_rate_counter < pattern_rate) pattern_rate_counter ++;
  else
  {
    
    if (loop_direction == 1) pattern_counter ++;
    else pattern_counter --;
    pattern_rate_counter = 0;
  }

  //calculate parametric ball patterns
  switch (pattern)
  {
    case 1:
      if (uint8_t(pattern_counter) < 64) 
      {
        set_x_pos = 300;
        set_y_pos = 300;
      }
      else if (uint8_t(pattern_counter) < 128) 
      {
        set_x_pos = 300;
        set_y_pos = -300;
      }
      else if (uint8_t(pattern_counter) < 192) 
      {
        set_x_pos = -300;
        set_y_pos = -300;
      }
      else 
      {
        set_x_pos = -300;
        set_y_pos = 300;
      }
      break;

    case 2:
      set_x_pos = cos_LUT[uint8_t(pattern_counter)];
      set_y_pos = sin_LUT[uint8_t(pattern_counter)];
      break;
    case 3:
      set_x_pos = 2 * cos_LUT[uint8_t(pattern_counter)];
      set_y_pos = 2 * sin_LUT[uint8_t(pattern_counter)];
      break;
    case 4:
      set_x_pos = 3 * cos_LUT[uint8_t(pattern_counter)];
      set_y_pos = 3 * sin_LUT[uint8_t(pattern_counter)];
      break;
    case 5:
      set_x_pos = 3 * sin_LUT[uint8_t(pattern_counter/2 +64)];
      set_y_pos = 3 * sin_LUT[uint8_t(pattern_counter)];
      break;
    case 6:
      set_x_pos = 3 * sin_LUT[uint8_t(pattern_counter/2+64)];
      set_y_pos = 3 * sin_LUT[uint8_t(pattern_counter/3 )];
      break;
    case 7:
      set_x_pos = 3 * sin_LUT[uint8_t(pattern_counter/4+64)];
      set_y_pos = 3 * sin_LUT[uint8_t(pattern_counter/3 )];
      break;
    case 8:
      set_x_pos = 3 * sin_LUT[uint8_t(pattern_counter/4+64)];
      set_y_pos = 3 * sin_LUT[uint8_t(pattern_counter/5 )];
      break;

    default:
      set_x_pos = 0;
      set_y_pos = 0;
      break;
  }

  //adjust variables with quadrature control knob 
  switch (select_mode)
  {
    case 1:
      p_gain = enc_knob(p_gain);
      break;
    case 2:
      d_gain = enc_knob(d_gain);
      break;
    case 3:
      i_gain = enc_knob(i_gain);
      break;
    case 4:
      x_offset = x_offset - 100 * x_offset + 100 * enc_knob(x_offset);
      break;
    case 5:
      y_offset = y_offset - 100 * y_offset + 100 * enc_knob(y_offset);
      break;
    case 6:
      pattern = pattern - 100 * pattern + 100 * enc_knob(pattern);
      if (pattern > 8) pattern = 0;
      break;
    case 7:
      pattern_rate = pattern_rate - 100 * pattern_rate + 100 * enc_knob(pattern_rate);
      break;
    case 8:
      loop_direction = loop_direction - 100 * loop_direction + 100 * enc_knob(loop_direction);
      break;

    default:
      hold_count = enc_knob(hold_count);
      break;
  }
}
