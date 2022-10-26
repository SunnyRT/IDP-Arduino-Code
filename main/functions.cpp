/* include all libraries */
#include "functions.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

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
  if (hall < 400) // hall == 0 if magnet is present
  {
    // magnet is detected in the blk
    flag_magnet = 1;
    box_intend = 3; // blk is to be delivered in the red box

    // light up the red LED for 5 sec
    digitalWrite(ledR_pn, HIGH);
    delay(5000);
  }
  else // hall == 900 if magnet is not present
  {
    // no magnet in the blk
    flag_magnet = 0;
    box_intend = 1; // blk is to be delivered in the green box

    // light up the green LED for 5 sec
    digitalWrite(ledG_pn, HIGH);
    delay(5000);
  }

  // blk collection
  // move the claw by the servo to trap the blk
  servo_claw.write(180); // the value here requires alibration
  delay(1000);

  flag_blk = true; // the blk has been collected
}

// side == 3;
void tunnel(); // done --> tunnel PID control

// side == 4;
void box_find();

void box_delivery();
