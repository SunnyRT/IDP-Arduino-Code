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
void sensor_read(){
  // light/line sensors:
  ldr = analogRead(ldr_pn);
  hall = analogRead(hall_pn);
  ir1 = analogRead(ir1_pn);
  ir2 = analogRead(ir2_pn);
  onoff = update_onoff();
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

  //identify which side we are on:
  side_identify(us1_avg); //Change depending on which sensor is on the right
};

// identify which side we are on
void side_identify(RH_sensor){
  float side0r_ub = 100; //experimentally determine
  float side0r_lb = 0;
  float side1r_ub = 100; //experimentally determine
  float side1r_lb = 0;
  float side2r_ub = 100; //experimentally determine
  float side2r_lb = 0;
  float side3r_ub = 100; //experimentally determine
  float side3r_lb = 0;
  float side4r_ub = 100; //experimentally determine
  float side4r_lb = 0;

  // if ir1 within Rside0 range AND the flag_side is not already 0
  if(side0r_lb<RH_sensor<side0r_ub && flag_side != 0){
    // raise side0 flag
    flag_side = 0;
  }
  else if(side1r_lb<RH_sensor<side1r_ub && flag_side != 1){
    // raise side0 flag
    flag_side = 1;
  }
  else if(side2r_lb<RH_sensor<side2r_ub && flag_side != 2){
    // raise side0 flag
    flag_side = 2;
  }
  else if(side3r_lb<RH_sensor<side3r_ub && flag_side != 3){
    // raise side0 flag
    flag_side = 3;
  }
  else if(side4r_lb<RH_sensor<side3r_ub && flag_side != 3){
    // raise side0 flag
    flag_side = 4;
  }

}

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
    flag_nav = 'F';
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(motor_speed);
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

void move_servo(){
  for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
