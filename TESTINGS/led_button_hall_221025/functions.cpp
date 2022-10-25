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

float moving_avg(float new_reading)
{

  const int nvalues = 20;     // Moving average window size
  static int current = 0;     // Index for current value
  static int value_count = 0; // Count of values read (<= nvalues)
  static float sum = 0;       // Rolling sum
  static float values[nvalues];

  sum += new_reading;

  // If the window is full, adjust the sum by deleting the oldest value
  if (value_count == nvalues)
    sum -= values[current];

  values[current] = new_reading; // Replace the oldest with the latest

  if (++current >= nvalues)
    current = 0;

  if (value_count < nvalues)
    value_count += 1;

  return sum / value_count;
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
  // l3 = digitalRead(l3_pn);
  // us1_distance = us_measure(us1T_pn, us1E_pn);
  // us2_distance = us_measure(us2T_pn, us2E_pn);

  // // calculate averages for distance readings
  // ir1_avg = moving_avg(ir1);
  // ir2_avg = moving_avg(ir2);
  // us1_avg = moving_avg(us1_distance);
  // us2_avg = moving_avg(us2_distance);

  //  //identify which side we are on:
  //  side_identify(us1_avg); //Change depending on which sensor is on the right
};

bool update_onoff()
{
  if (millis() - lastTimeButtonStateChanged > debounceDuration)
  {
    byte buttonState = digitalRead(button_pn);
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
  Serial.println(flag_onoff);
}

void line_follow(){
    // copy content in line_follow_v2.ino here
};

void ledA_flash()
{
  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledAState == LOW)
    {
      ledAState = HIGH;
    }
    else
    {
      ledAState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledA_pn, ledAState);
  }
}

// side == 0;
void start_route(); // done

// side == 1;
void ramp_up();
void ramp_down();

// side == 2;
void blk_fn();




void blk_magnet_identify()
{
    if (hall <= 5) // the threshold value here requires measurement & calibration
    {
        // magnet is detected in the blk
        flag_magnet = 1;
        box_intend = 3; // blk is to be delivered in the red box

        // light up the red LED for 5 sec
        digitalWrite(ledR_pn, HIGH);
        delay(5000);
    }
    else
    {
        // no magnet in the blk
        flag_magnet = 0;
        box_intend = 1; // blk is to be delivered in the green box

        // light up the green LED for 5 sec
        digitalWrite(ledG_pn, HIGH);
        delay(5000);
    }
}

void blk_collect()
{
    while (ldr >= 0) // the threshold value here requires measurement & calibration
    {
        // continue to approach the blk until it touches.
        line_follow();
    }

    // touch the blk i.e. distance = 0 from the blk
    stop_move();

    // move the claw by the servo to trap the blk
    servo_claw.write(180); // the value here requires alibration
    delay(1000);

    flag_blk = true; //the blk has been collected
}

// side == 3;
void tunnel(); // done --> tunnel PID control

// side == 4;
void box_find();

void box_delivery();
