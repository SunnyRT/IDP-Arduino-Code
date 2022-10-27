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
#define ledG_pn A2
#define ledR_pn A3  //use analog as digital output to light up leds

//Digital:

const int l0_pn = 2; //left
const int l1_pn = 3; //right
const int l2_pn = 4; //far right (for juntion counting)
const int button_pn = 5;

//us1: on the front
const int us1E_pn = 6; // yellow wire
const int us1T_pn = 7; // green wire
const int hall_pn = 8;

const int ledA_pn = 10;
const int servo1_pn = 11;

//us2: on the left
const int us2E_pn = 12;
const int us2T_pn = 13;



//flags
bool flag_onoff;
bool flag_started;
char flag_nav;
bool flag_blk;
bool flag_magnet;

// sensor readings
bool hall;
float us1_distance;

int box_intend;
int box_pass;


// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;



void setup() {
  Serial.begin(9600);
  pinMode(button_pn, INPUT);
  pinMode(hall_pn, INPUT);
  pinMode(us1E_pn, INPUT);
  pinMode(us1T_pn, OUTPUT);
  pinMode(ledG_pn, OUTPUT);
  pinMode(ledR_pn, OUTPUT);
  pinMode(servo1_pn, OUTPUT);
  AFMS.begin(); // Connect to the motor controller
  servo_claw.attach(servo1_pn);  // connect to servo
}

void loop() {
  hall = digitalRead(hall_pn);
  Serial.print("hall: ");
  Serial.println(hall);
  us1_measure();
  Serial.print("us1: ");
  Serial.println(us1_distance);

  if (us1_distance < 2) {
    // approach slowly until us1_distance = 2cm
    stop_move();
    Serial.println("Block Detected");
    delay(1000);
    Serial.println("Start detecting if magnet");
    /* blk function */
    if (hall == 0) {
      // magnet
      Serial.println("Magnet");
      flag_magnet = 1;
      box_intend = 3; // blk is to be delivered in the red box
      // RED LED for 5sec
      analogWrite(ledR_pn, 255);
      delay(5000);
      analogWrite(ledR_pn, 0);
    }
    else {
      // no magnet
      Serial.println("Not Magnet");
      flag_magnet = 0;
      box_intend = 1; // bl
      // Green LED for 5sec
      analogWrite(ledG_pn, 255);
      delay(5000);
      analogWrite(ledG_pn, 0);
    }

    /** claw  code */
    servo_claw.write(180); // the value here requires alibration
    delay(1000);
    flag_blk = true;
    stop_move();
    flag_onoff = false;
  }
  else //continue approach the blk when the distance is far. (>2cm)
  {
    Serial.println("moving forwards");
    move_forward();
  }

}



//functions
void us1_measure()
{
  digitalWrite(us1T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us1T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us1T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long us1_duration = pulseIn(us1E_pn, HIGH);
  // get distance values(*v_sound /2)
  us1_distance = us1_duration * 0.017;
}


void move_forward()
{
    flag_nav = 'F';
    Rwheel->run(FORWARD);
    Rwheel->setSpeed(motor_speed);
    Lwheel->run(FORWARD);
    Lwheel->setSpeed(motor_speed);
}


void stop_move(){
    flag_nav = 'P';
    Rwheel->run(RELEASE);
    Rwheel->setSpeed(0);
    Lwheel->run(RELEASE);
    Lwheel->setSpeed(0);
}
