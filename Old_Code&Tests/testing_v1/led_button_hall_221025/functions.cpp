/* include all libraries */
#include "functions.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 


// Push button to turn robot on or off
bool update_onoff() {
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

// calculate distance from ultrasonic sensors
void us_measure()
{
  digitalWrite(us1T_pn, LOW);
  digitalWrite(us2T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us1T_pn, HIGH);
  digitalWrite(us2T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us1T_pn, LOW);
  digitalWrite(us2T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float us1_duration = pulseIn(us1E_pn, HIGH);
  float us2_duration = pulseIn(us2E_pn, HIGH);
  // get distance values(*v_sound /2)
  us1_distance = us1_duration * 0.017;
  us2_distance = us2_duration * 0.017;
}

float moving_avg(float new_reading)
{
  const int window_size = 20;     // Moving average window size
  
  static int current_index = 0;     // Index for current value
  static float sum = 0;       // Rolling sum
  static float values_array[window_size]; //array
  float average = 0.0;

  // remove oldest value
  sum -= values_array[current_index];
  // replace oldest with new reading
  values_array[current_index] = new_reading;
  sum =+ new_reading;

  // move to next position (which is now the oldest value) in array
  current_index =+ 1;

  // if this is the last position in array
  if(current_index==window_size){ 
    current_index = 0; // wrap round to start of window szie
  }
  // return average
  return sum/window_size;
}

// collects sensor readings (will be run every loop)
void sensor_read()
{
  ldr = analogRead(ldr_pn);
  hall = digitalRead(hall_pn);
  ir1 = analogRead(ir1_pn);
  ir2 = analogRead(ir2_pn);
  update_onoff();
  l0 = digitalRead(l0_pn);
  l1 = digitalRead(l1_pn);
  l2 = digitalRead(l2_pn);
  us_measure();
  ir1_avg = moving_avg(ir1);
  ir2_avg = moving_avg(ir2);
};


// amber LED flashes at 2Hz while robot is running
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
    // toggle LED
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




void line_follow()
{
  if (l2 == LOW && flag_nav != 'P')
  {
    Serial.println("Junctions detected!");
    stop_move();
    delay(1000);
  }

  else if (l2 == HIGH)
  {
    // neither l0 or l1 detects the line
    if ((l0 == HIGH && l1 == HIGH) && flag_nav != 'F')
    {
      // forward
      move_forward();
      Serial.println("MOVE FORWARD!");
    }

    // l0 detects the line
    else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'L')
    {
      // adjust left slightly
      adjust_left();
    }

    // l1 detects the line
    else if ((l0 == HIGH && l1 == LOW) && flag_nav != 'R')
    {
      // adjust right slightly
      adjust_right();
    }

    // l0 and l1 both detect the line
    else if ((l0 == LOW && l1 == LOW) && flag_nav != 'P')
    {
      // forward
      stop_move();
    }
  }
}



// side == 0;
void start_route()
{
  if (l0 == HIGH && l1 == HIGH && l2 == HIGH && flag_nav != 'F')
  {
    move_forward();
  }
  else if ((l0 == LOW && l1 == LOW) || l2 == LOW)
  {
    turn_90right();
    flag_started = true; // exit start_route
  }
}

// side == 1;
void ramp_up();
void ramp_down();
// side == 2;
void blk_fn()
{
  // ensunre the car is not stationary.
  stop_move();
  // magnetc identification
  if (hall != 0) // hall == 1 if magnet is present
  {
    // magnet is detected in the blk
    flag_magnet = 1;
    box_intend = 3; // blk is to be delivered in the red box

    // light up the red LED for 5 sec
    digitalWrite(ledR_pn, HIGH);
    delay(5000);
    digitalWrite(ledR_pn, LOW);
  }
  else // hall == 0 if magnet is not present
  {
    // no magnet in the blk
    flag_magnet = 0;
    box_intend = 1; // blk is to be delivered in the green box

    // light up the green LED for 5 sec
    digitalWrite(ledG_pn, HIGH);
    delay(5000);
    digitalWrite(ledG_pn, LOW);
  }

  // blk collection
  // move the claw by the servo to trap the blk
  blk_collect();
  delay(1000);

  flag_blk = true; // the blk has been collected
}

// side == 3;
void tunnel(); // done --> tunnel PID control

// side == 4;
void box_find();

void box_delivery();
