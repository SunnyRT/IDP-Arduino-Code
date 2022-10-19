#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "functions.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

/*defining pins and variables*/

#define push_pn 2 // switch
#define l0_pn 3   // left
#define l1_pn 4   // right
#define l2_pnn 5  // far right

// Variables will change:
int button_state = HIGH;            // the current reading from the input pin
int last_button_state = HIGH; // the previous reading from the input pin
bool flag_started = false;
bool flag_onoff = false;
bool push;
char flag_nav = 'P';

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers

void setup()
{
  pinMode(push_pn, INPUT);

  // set initial state to off
  flag_onoff = false;
  Serial.begin(9600);
}

void loop()
{

  // sensor_read(); //--> read push button and store the value into "push"
  push = digitalRead(push_pn);
  Serial.print("push: ");
  Serial.println(push);
  /**********************************************************************************/
  /* push button debounce function */
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (push != last_button_state)
  {
    // reset the debouncing timer
    lastDebounceTime = millis();
    Serial.println(lastDebounceTime);
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (push != button_state)
    {
      button_state = push;
      Serial.print("button_state: ");
      Serial.println(button_state);

      // only toggle the LED if the new button state is HIGH
      if (button_state == LOW)
      {
        flag_onoff != flag_onoff;
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastbutton_state:
  last_button_state = push;

  /**********************************************************************************/
  Serial.print("flag_onoff: ");
  Serial.println(flag_onoff);
  if (flag_onoff == false && flag_nav != 'P')
  {
    Lwheel->run(RELEASE);
    Lwheel->setSpeed(0);
    flag_nav = 'P';
  }
  else if (flag_onoff == true && flag_nav != 'F')
  {
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(150);
    flag_nav = 'F';
  }
  else
  {
    //continue what it has been doing.
  }
}

/***********************************************************/
// define functions
void stop_move()
{
  flag_nav = "P";
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}
