#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Create the Adafruit_MotorShield object

//Create the DC motor object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);  //RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);  //LEFT


void setup() {
  AFMS.begin(); //Connect to the Controller
  Rwheel->setSpeed(200); 
  Lwheel->setSpeed(200); //Set default speed

}

void loop() {
  Rwheel->run(FORWARD); 
  Lwheel->run(FORWARD); //Run the motor

}
