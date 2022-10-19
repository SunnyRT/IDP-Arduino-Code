/* include all libraries */
#include "functions.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

// calculate distance from ultrasonic sensors
float us_measure(trig_pin, echo_pin){
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
     // measure duration of pulse from ECHO pin
    float duration_us = pulseIn(echo_pin, HIGH);
    // return distance (*v_sound /2)
    return duration_us * 0.017;
}


float moving_average(float new_reading)
{

  const int nvalues = 10;     // Moving average window size
  static int current = 0;     // Index for current value
  static int value_count = 0; // Count of values read (<= nvalues)
  static float sum = 0;       // Rolling sum
  static float values[nvalues];

  sum += new_reading;

  // If the window is full, adjust the sum by deleting the oldest value
  if (cvalues == nvalues)
    sum -= values[current];

  values[current] = value; // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (cvalues < nvalues)
    cvalues += 1;

  return sum / cvalues;
}

// collects sensor readings (will be run every loop)
void sensor_read(){
  // light/line sensors:
    ldr = analogRead(ldr_pn);
    hall = analogRead(hall_pn);
    ir1 = analogRead(ir1_pn);
    ir2 = analogRead(ir2_pn);
    push = digitalRead(push_pn);
    l0 = digitalRead(l0_pn);
    l1 = digitalRead(l1_pn);
    l2 = digitalRead(l2_pn);
    l3 = digitalRead(l3_pn);
    us1_distance = us_measure(us1T_pn, us1E_pn);
    us2_distance = us_measure(us2T_pn, us2E_pn); 
};


bool update_onoff()
{
  if (millis() - lastTimeButtonStateChanged > debounceDuration)
  {
    byte buttonState = digitalRead(push_pn);
    if (buttonState != lastButtonState)
    {
      lastTimeButtonStateChanged = millis();
      lastButtonState = buttonState;
      if (buttonState == LOW)
      {
        // do an action, for example print on Serial
        Serial.println("Button released");
        onoff = !onoff;
      }
    }
  }
  return onoff
}


void line_follow()
{
  // copy content in line_follow_v2.ino here
  flag_line = "on";
};
void move_forward()
{
    flag_nav = "F";
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(motor_speed);
}

void adjust_left()
{
  flag_nav = "L";
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
}

void adjust_right()
{
  flag_nav = "R";
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}
void stop_move()
{
  flag_nav = "P";
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}

void turn_90left()
{
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
  delay(duration_steer);
};
void turn_90right(){
    Rwheel->run(RELEASE);
    Rwheel->setSpeed(0);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(motor_speed0);
  delay(duration_steer); 
};

// side == 0;
void start_route();

// side == 1;
void ramp_up();
void ramp_down();

// side == 2;
void blk_fn();

void blk_approach();
void blk_magnet_identify();
void blk_magnet_indicate();
void blk_collect();
void blk_retriet();

// side == 3;
void tunnel();

// side == 4;
void box_find();

void box_delivery();




// how to read from different sensors
void IR_read(){

}
