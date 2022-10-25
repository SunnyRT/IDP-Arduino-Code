/* include all libraries */
#include "functions.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"



// calculate distance from ultrasonic sensors
float us_measure(int trig_pin, int echo_pin)
{
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(echo_pin, HIGH);
  // return distance (*v_sound /2)
  return duration_us * 0.017;
}


float moving_avg(float new_reading){

  const int nvalues = 20;               // Moving average window size
  static int current = 0;               // Index for current value
  static int value_count = 0;           // Count of values read (<= nvalues)
  static float sum = 0;                  // Rolling sum
  static float values[nvalues];

  sum += new_reading;

  // If the window is full, adjust the sum by deleting the oldest value
  if (value_count == nvalues)
    sum -= values[current];

  values[current] = new_reading;          // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (value_count < nvalues)
    value_count += 1;

  return sum/value_count;
}

// collects sensor readings (will be run every loop)
void sensor_read()
{
  // light/line sensors:
  ldr = analogRead(ldr_pn);
  hall = analogRead(hall_pn);
  ir1 = analogRead(ir1_pn);
  ir2 = analogRead(ir2_pn);
  update_onoff();
  l0 = digitalRead(l0_pn);
  l1 = digitalRead(l1_pn);
  l2 = digitalRead(l2_pn);
  l3 = digitalRead(l3_pn);
  us1_distance = us_measure(us1T_pn, us1E_pn);
  us2_distance = us_measure(us2T_pn, us2E_pn);

  // calculate averages for distance readings
  ir1_avg = moving_avg(ir1);
  ir2_avg = moving_avg(ir2);
  us1_avg = moving_avg(us1_distance);
  us2_avg = moving_avg(us2_distance);

//  //identify which side we are on:
//  side_identify(us1_avg); //Change depending on which sensor is on the right
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
        flag_onoff = !flag_onoff;
      }
    }
  } 
}

void line_follow()
{
  // copy content in line_follow_v2.ino here
};


void move_forward(int speedR, int speedL)
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(speedR);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(speedL);
}

void adjust_left()
{
  flag_nav = 'L';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}
void stop_move()
{
  flag_nav = 'P';
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
}

void turn_90right()
{
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer);
}

// side == 0;
void start_route(); //done

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
void tunnel(); //done --> tunnel PID control

// side == 4;
void box_find();

void box_delivery();
