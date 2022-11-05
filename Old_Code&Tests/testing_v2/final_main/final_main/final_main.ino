#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "motion_fns.h"

/* setup the motor and create the DC motor object*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the Adafruit_MotorShield object
Adafruit_DCMotor *Rwheel = AFMS.getMotor(1);        // RIHGT
Adafruit_DCMotor *Lwheel = AFMS.getMotor(2);        // LEFT
int motor_speed = 200;
int duration_steer = 1400; // require testing to determine value
int duration_start_forward = 1400;
Servo servo_claw;

// Pins Set-up:
//Analog:
const int hall_pn = 8;
//Digital:
const int button_pn = 5;
const int l0_pn = 2; //left
const int l1_pn = 3; //right
const int l2_pn = 4; //far right (for juntion counting)

//us1: on the front
const int us1E_pn = 6; // yellow wire
const int us1T_pn = 7; // green wire
const int ledG_pn = A2; // now analog
const int ledR_pn = A3; // now analog
const int ledA_pn = 10;
const int servo1_pn = 9;

//us2: on the left
const int us2E_pn = 12;
const int us2T_pn = 13;

// Sensor Values
int l0, l1, l2, hall;
int loop_count = 0;
float us1_distance = 200.0;
float us2_distance = 200.0;

// button debounce variables
unsigned long currentTime = 0;
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

// time variable
unsigned long started_time = 0;
unsigned long return_home_time = 0;
int return_home_duration = 6500;

// PID tunnel navigation variables
float Kp = 10; // related to the proportional control term;
float P;
float distance_tunnel1 = 5.5;
float distance_tunnel2 = 7; //unit in cm

const float maxspeedR = 200.0;
const float maxspeedL = 200.0;
const float basespeedR = 150.0;
const float basespeedL = 150.0;


/*************
   flags
**************/
bool flag_onoff = false; // push button: robot on or off
bool flag_ledA = false;
bool flag_started;       // has robot completed initial start?
char flag_nav = 'P';
//"P": stop
//"L": adjust left
//"R": adjust right
//"B": backwards
//"F": forwards
bool flag_blk = false; // block collected
bool flag_delivered = false;
int flag_tunnel = 0; // this flag is only for the returning through tunnel
/*
   0: before the tunnel --> us2 reading every 5 loops
   1: within the tunnel --> us2 reading every loops
   2: after the tunnel --> no us2 reading
*/

int box_intend = 1;
int box_pass = 0;
bool flag_box_register;

void setup() {
  Serial.begin(9600);
  // set pins as inputs
  pinMode(button_pn, INPUT);
  pinMode(hall_pn, INPUT);
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
  AFMS.begin(); // Connect to the motor controller
  servo_claw.attach(servo1_pn);  // connect to servo
  servo_claw.write(0);
}


void loop() {
  //register loop_count
  if (loop_count == 5) {
    loop_count = 0;
  }
  else {
    loop_count += 1;
  }
  //  Serial.print("loop count: ");
  //  Serial.println (loop_count);

  /*read & print line sensors*/
  Serial.print("Line Sensors: ");
  l0 = digitalRead(l0_pn);
  Serial.print(l0);
  l1 = digitalRead(l1_pn);
  Serial.print(l1);
  l2 = digitalRead(l2_pn);
  Serial.println(l2);

  /** On-off Push Button:*/
  // if the time elapsed is greater than time for debounce:
  currentTime = millis();
  Serial.print("currentTime: ");
  Serial.println(currentTime);
  if (currentTime - lastTimeButtonStateChanged > debounceDuration) {
    // read button state
    byte buttonState = digitalRead(button_pn);
    // if button state has changed
    if (buttonState != lastButtonState) {
      // update time
      lastTimeButtonStateChanged = currentTime;
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
  } // close button change
  Serial.print("ONOFF: ");
  Serial.println(flag_onoff);
  if (flag_onoff == false && flag_ledA == true ) {
    digitalWrite(ledA_pn, LOW);
  }
  else if (flag_onoff == true && flag_ledA == false) {
    digitalWrite(ledA_pn, HIGH);
  }


  /** Navigation */
  // if 'Off' and not already stopped, stop:
  if (flag_onoff == false && flag_nav != 'P') {
    stop_move();
  }
  // 'ON' and not running the start function:
  else if (flag_onoff == true && flag_started == false) {
    start_route();
    started_time = millis();
    Serial.print("started_time: ");
    Serial.println(started_time);
  }
  // if On and the start fn has been completed:
  else if (flag_onoff == true && flag_started == true) {
    if (flag_blk == false) // no blk is collected
    {
      while ((currentTime - started_time) > 14500 && (currentTime - started_time) < 19500) {
        us2_measure();
        Serial.print("us2: ");
        Serial.println(us2_distance);
        tunnel_PID_control(distance_tunnel1);
        Serial.println("moving in tunnel...");
        currentTime = millis();
      }
      us2_distance = 200; // to re-initialize the value of us2_distance
      if (loop_count == 0) {
        us1_measure();
        Serial.print("us1: ");
        Serial.println(us1_distance);
      }
      if (us1_distance > 10.0) {
        line_follow();
        Serial.println("line");
      }
      else {
        blk_find();
      }
    else // blk has been collected, return route (flag_blk == true)
    {
      Serial.print("flag_tunnel: ");
      Serial.println(flag_tunnel);
      Serial.print("us2: ");
      Serial.println(us2_distance);
      if (flag_delivered == false) //blk has yet to be delivered
      {
        if (flag_tunnel == 0) // before the tunnel --> try to identify when tunnel is reached (us2_distance > 12.0)
        {
          if (loop_count == 0) {
            us2_measure(); // us2 readings every 5 loops
          }
          //          Serial.print("us2: ");
          //          Serial.println(us2_distance);
          if (us2_distance > 12.0) // not yet reached the tunnel --> line follow
          {
            line_follow();
          }
          else {
            flag_tunnel = 1; // us2_distance < 12.0 --> tunnel has been reached
          }
        } // before the tunnel

        else if (flag_tunnel == 1) // within the tunnel
        {
          us2_measure();  // us2 readings every loop
//          Serial.print("us2: ");
//          Serial.println(us2_distance);
          if (us2_distance < 12.0) // moving within the tunnel
          {
            tunnel_PID_control(distance_tunnel2);
            Serial.println("moving in tunnel...");
          }
          else {
            flag_tunnel = 2;  // us2_distance > 12.0 --> tunnel has been passed
          }
        } // within the tunnel

        else if (flag_tunnel == 2) // after the tunnel (finish going through the tunnel)
        {
          //move back towards the box to deliver
          if (l2 == LOW) {
            flag_box_register = false;
            line_follow();
          }
          else if (l2 == HIGH && l1 == HIGH )// junction detected! --> blk delivery
          {
            if (flag_box_register == false) // if junction for box has not been registered
            {
              flag_box_register = true; // at start of junction raise junction flag
              box_pass += 1; // increment the number of box-junctions passed
              Serial.print(box_pass);
            }
            if (box_pass == box_intend) //time to deliver the blk!!
            {
              Serial.println("junction detected!");
              blk_delivery();
              blk_retriet();
              return_home_time = millis();
              Serial.print("return_home_time: ");
              Serial.println(return_home_time);
            }
            else if (box_pass < box_intend) //not yet reached the intended box to deliver the blk.
            {
              line_follow();
            }
            else {
              Serial.println("Error: the robot missed the box to deliver!");
            }
          } // when junction is detected
        } // after the tunnel
      } // blk has not been delivered (flag_delivered == false)

      else // (flag_delivered == true) blk has been delivered, return home
      {
        if ((currentTime - return_home_time) < return_home_duration) //move back towards the middle
        {
          line_follow();
        }
        else // --> return home
        {
          return_home();
        }
      } // blk has been delivered (flag_delivered == true) --> return home
    } // blk has been collected (flag_collected == true)
  } // flag_onoff == true; flag_started == true
} // loop 

} // close main loop



/*********************************************************************************
   Functions related to navigation
 *********************************************************************************/

void stop_move() {
  flag_nav = 'P';
  Rwheel->run(RELEASE);
  Rwheel->setSpeed(0);
  Lwheel->run(RELEASE);
  Lwheel->setSpeed(0);
}


void move_forward()
{
  flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed + 6);
}

void move_backward() {
  flag_nav = 'B';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);

}

void adjust_left()
{
  flag_nav = 'L';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(100);
}

void adjust_right()
{
  flag_nav = 'R';
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(100);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
}

void turn_90left()
{
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer - 100);
}

void turn_90right()
{
  Rwheel->run(BACKWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer + 50);
}

void turn_180() {
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motor_speed);
  Lwheel->run(BACKWARD);
  Lwheel->setSpeed(motor_speed);
  delay(duration_steer * 2 - 200); // How long it will turn for!
}

void tunnel_PID_control(float distance_wall)
{
  float error = distance_wall - us2_distance;

  P = error;
  float motorspeed = P * Kp;

  //  Serial.print("error: ");
  //  Serial.println(error);
  //  Serial.print("motorspeed: ");
  //  Serial.println(motorspeed);


  float motorspeedR = basespeedR - motorspeed;
  float motorspeedL = basespeedL + motorspeed;

  if (motorspeedR > maxspeedR)
  {
    motorspeedR = maxspeedR;
  }
  if (motorspeedL > maxspeedL)
  {
    motorspeedL = maxspeedL;
  }
  if (motorspeedR < 0)
  {
    motorspeedR = 0;
  }
  if (motorspeedL < 0)
  {
    motorspeedL = 0;
  }
    flag_nav = 'F';
  Rwheel->run(FORWARD);
  Rwheel->setSpeed(motorspeedR);
  //  Serial.print("speedR: ");
  //  Serial.println(motorspeedR);

  Lwheel->run(FORWARD);
  Lwheel->setSpeed(motorspeedL);
  //  Serial.print("speedL: ");
  //  Serial.println(motorspeedL);
}