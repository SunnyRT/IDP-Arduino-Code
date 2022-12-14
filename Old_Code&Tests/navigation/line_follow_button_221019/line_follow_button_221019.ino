#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

/*defining pins and variables*/

#define BUTTON_PIN 2 // switch
#define l0_pn 3      // left
#define l1_pn 4      // right
#define l2_pn 5      // far right

// Variables will change:
bool flag_started = false;
bool flag_onoff = false;
char flag_nav = 'P';
int motor_speed = 250;
int l0, l1, l2;

// button variable
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  pinMode(l0_pn, INPUT); // light sensor on the left
  pinMode(l1_pn, INPUT); // light sensor on the right
  pinMode(l2_pn, INPUT); // light sensor on the far right
  Serial.begin(9600);
  AFMS.begin(); // Connect to the controller
}

void loop()
{
  // read and store light sensor values
  l0 = digitalRead(l0_pn);
  l1 = digitalRead(l1_pn);
  l2 = digitalRead(l2_pn);

  // print values of the light sensors to the serial monitor
  Serial.print("Sensor Readings: ");
  Serial.print(l0);
  Serial.print(l1);
  Serial.println(l2);

  Serial.print("Flag States: ");
  Serial.println(flag_nav);

  // button debounce
  if (millis() - lastTimeButtonStateChanged > debounceDuration)
  {
    byte buttonState = digitalRead(BUTTON_PIN);
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
  Serial.print("ONOFF: ");
  Serial.println(flag_onoff);

  if (flag_onoff == true)
  {
    line_follow();
  }

  else if (flag_onoff == false && flag_nav != 'P')
  {
    stop_move();
  }
}

/***********************************************************/
// define functions
void line_follow()
{

  if (l2 == LOW && flag_nav != 'P')
  {
    Serial.println("Junctions detected!");
    stop_move();
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

// functions
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
  Lwheel->setSpeed(200);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(200);
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