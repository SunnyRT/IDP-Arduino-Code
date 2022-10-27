#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
int motor_speed = 250;
int duration_steer = 2000; // require testing to determine value
Servo servo_claw; 

// Pins Set-up:
//Analog:
const int ldr_pn = A0;
const int hall_pn = A1;
const int ir1_pn = A4; // short range
const int ir2_pn = A5; // long range
//Digital:
const int button_pn=1; 
const int l0_pn=2; //left
const int l1_pn=3; //right
const int l2_pn=4; //far right (for juntion counting)

//us1: on the right
const int us1E_pn=6; // yellow wire
const int us1T_pn=7; // green wire
const int ledG_pn=8;
const int ledR_pn=9;
const int ledA_pn=10;
const int servo1_pn=11;

//us2: on the left
const int us2E_pn=12;
const int us2T_pn=13;

// Sensor Values
int l0, l1, l2, hall;
float us1_distance, us2_distance;

// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

//ledA_flash variables
int ledAState = LOW;  // ledState used to set the LED
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 250;  // 2hz --> 0.5 second interval at which to blink (milliseconds)



