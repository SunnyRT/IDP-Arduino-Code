// include header files:
#include "functions.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT

// setup servo object
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
const int us1E_pn=6; // yellow wire
const int us1T_pn=7; // green wire
const int ledG_pn=8;
const int ledR_pn=9;
const int ledA_pn=10;
const int servo1_pn=11;
const int us2E_pn=12;
const int us2T_pn=13;
int motor_speed = 250;




// Sensor Values
int ldr, l0, l1, l2, l3, ir1, ir2, hall, push;
float us1_distance, us2_distance;
float ir1_avg, ir2_avg, us1_avg, us2_avg;
int duration_steer; // require testing to determine value

// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

//ledA_flash variables
int ledAState = LOW;  // ledState used to set the LED
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 250;  // 2hz --> 0.5 second interval at which to blink (milliseconds)

// array of averages
float ir1_avg_arr[10]; // 10=window size for moving average
int ir1_avg_arr_index = 0;


// flags
int side; // 0,1,2,3,4,5
int count = 0;
bool flag_onoff = false;
bool flag_started;
bool flag_line;
int flag_side; 



char flag_nav; 
  //"P": stop
  //"L": adjust left
  //"R": adjust right
  //"B": backwards
  //"F": forwards

bool flag_blk;
bool flag_magnet;
int box_intend;




void setup(){
  Serial.begin(9600);
  // set pins as inputs
  pinMode(button_pn, INPUT);
  pinMode(ldr_pn, INPUT);
  pinMode(hall_pn, INPUT);
  pinMode(ir1_pn, INPUT); 
  pinMode(ir2_pn, INPUT);
  pinMode(l0_pn, INPUT);
  pinMode(l1_pn, INPUT);
  pinMode(l2_pn, INPUT);
  pinMode(us1E_pn, INPUT);
  pinMode(us1T_pn, OUTPUT);
  pinMode(ledG_pn, OUTPUT);
  pinMode(ledA_pn, OUTPUT);
  pinMode(ledR_pn, OUTPUT);
  pinMode(servo1_pn, OUTPUT);
  pinMode(us2E_pn, INPUT);
  pinMode(us2T_pn, OUTPUT);
  AFMS.begin(); // Connect to the controller
}

void loop(){
  //update all sensor readings & determine which side we're on
  update_onoff();
//  sensor_read();
  ldr = analogRead(ldr_pn);
  hall = digitalRead(hall_pn);
  Serial.print("ldr: ");
  Serial.println(ldr);
  Serial.print("hall effect: ");
  Serial.println(hall);

  if (flag_onoff == true){
    ledA_flash(); // flash the amble LED at 2Hz frequency;

    if (ldr > 50) // ldr == 20~30 for touching the blk
    {
//      line_follow(); // continue to move forward (towards the block) until ldr changes to 500
      Serial.print("moving forwawrd....");
    }
    else {
      Serial.print("block found!!");
      delay(1000);
      blk_fn(); //start detecting the magnet and collect the blk
      Serial.print("magnet present: ");
      Serial.println(flag_magnet);
      Serial.print("box_intend: ");
      Serial.println(box_intend);
    }
  }



}



void blk_collect(){
  servo_claw.write(180); // the value here requires calibration
}









/*******************************************************************************
 * functions on motor
 *******************************************************************************/
void move_forward(int speedR, int speedL)
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(speedR);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(speedL);
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
}

void turn_90right()
{
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer);
}
