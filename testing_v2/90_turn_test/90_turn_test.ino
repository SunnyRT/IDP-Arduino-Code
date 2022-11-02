#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

const int button_pn = 5;


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
int motor_speed = 200;
int duration_steer = 1400; // require testing to determine value

bool flag_onoff = false;
char flag_nav = 'P';

// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(button_pn, INPUT);
  AFMS.begin(); // Connect to the motor controller
  Serial.begin(9600);


}

void loop() {
  /** On-off Push Button:*/
  // if the time elapsed is greater than time for debounce:
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    // read button state
    byte buttonState = digitalRead(button_pn);
    // if button state has changed
    if (buttonState != lastButtonState) {
      // update time
      lastTimeButtonStateChanged = millis();
      // update lastButton state
      lastButtonState = buttonState;
      // while being pressed
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

  if (flag_onoff == false) {
    stop_move();
  }
  else {
    turn_180();
  }
  stop_move();
  flag_onoff = false;
}

void turn_90left()
{
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer -75);

}

void turn_90right()
{
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer -25);
}

void turn_180() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(2800); // How long it will turn for!

}

void stop_move() {
  flag_nav = 'P';
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}
