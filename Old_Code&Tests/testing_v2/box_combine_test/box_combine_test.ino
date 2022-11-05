#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

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
float us2_distance;

// button debounce variables
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

// time variable for return_home
unsigned long return_home_time = 0;
int return_home_duration = 5100;

// PID tunnel navigation variables
//float lastError = 0;
float Kp = 10; // related to the proportional control term;
// change the value by trial-and-error (ex: 0.07).
//float Ki = 0.0008; // related to the integral control term;
//// change the value by trial-and-error (ex: 0.0008).
//float Kd = 0.6; // related to the derivative control term;
//// change the value by trial-and-error (ex: 0.6).
float P;
//float I;
//float D;
float distance_tunnel = 7;

const float maxspeedR = 250.0;
const float maxspeedL = 250.0;
const float basespeedR = 200.0;
const float basespeedL = 200.0;


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
bool flag_tunnel; // true if in tunnel, false if not
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



  // flags
//  Serial.print("Flag_nav: ");
//  Serial.println(flag_nav);
//  Serial.print("Flag Started: ");
//  Serial.println(flag_started);

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
  if (flag_onoff == false && flag_ledA == true ) {
    digitalWrite(ledA_pn, LOW);
  }
  else if (flag_onoff == true && flag_ledA == false) {
    digitalWrite(ledA_pn, HIGH);
  }

  Serial.print("flag_blk: ");
  Serial.println(flag_blk);


  /** Navigation */
  // if 'Off' and not already stopped, stop:
  if (flag_onoff == false && flag_nav != 'P') {
    stop_move();
  }
  // 'ON' and not running the start function:
  else if (flag_onoff == true && flag_started == false) {
    start_route();
  }
  // if On and the start fn has been completed:
  else if (flag_onoff == true && flag_started == true) {
    if (flag_blk == false) // no blk is collected
    {
      if (loop_count == 0) {
        us1_measure();
        Serial.print("us1: ");
        Serial.println(us1_distance);
        //      us2_measure();
        //      Serial.print("us2: ");
        //      Serial.println(us2_distance);
      }
      if (us1_distance > 10.0) {
        line_follow();
        Serial.println("line");
      }
      else // the blk has been found!!!!
      {
        stop_move();
        delay(1000);

        //get closer to the blk for magnet detection
        move_forward();
        delay(500);
        stop_move();
        delay(1000);
        blk_magnet();

        //move backward a little bit for blk collection
        move_backward();
        delay(300);
        stop_move();
        delay(1000);
        blk_collect();
      }
    }
    else // blk has been collected, return route
    {
      if (flag_delivered == false) //blk has yet to be delivered
      {
        Serial.print("box_pass: ");
        Serial.println(box_pass);
        Serial.print("flag_box_register: ");
        Serial.println(flag_box_register);
        if (l2 == LOW) //move back towards the green box
        {
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
            return_home_time = millis() ;
          }
          else if (box_pass < box_intend) //not yet reached the intended box to deliver the blk.
          {
            move_forward();
          }
          else {
            Serial.println("Error: the robot missed the box to deliver!");
          }
        }
      }

      
      else // (flag_delivered == true) blk has been delivered, return home
      {
        if ((millis() - return_home_time) < return_home_duration) //move back towards the middle
        {
          line_follow();
        }
        else // junction detected! --> return home
        {
          Serial.println("junction detected!");
          return_home();
        }
      }
    }
  }
}













/*========= FUNCTIONS ===================*/
void line_follow() {
  if (l2 == LOW) {

    // 000 & it's not already moving forward
    if ((l0 == LOW && l1 == LOW) && flag_nav != 'F') {
      move_forward();
//      Serial.println("Move Forward");
    }

    // 100 & not already moving left
    else if ((l0 == HIGH && l1 == LOW) && flag_nav != 'L') {
      adjust_left();
    }

    // 010 & not already moving right
    else if ((l0 == LOW && l1 == HIGH) && flag_nav != 'R') {
      adjust_right();
    }
  }

  // xx1 or 11x(junction detected)
  else if (l2 == HIGH || (l0 == HIGH && l1 == HIGH)) {
    // handle junction depending on flags
//    Serial.println("Junction Detected");
  }

  // catch all other cases: stop & report error
  else {
    stop_move();
    Serial.print("Line Sensor Error: ");
    Serial.print(l0);
    Serial.print(l1);
    Serial.println(l2);
  }
}


void start_route() {
  // if still in the box (so no line detected), move forward until edge of box detected
  if (l0 == LOW && l1 == LOW && l2 == LOW && flag_nav != 'F')
  {
    move_forward();
  }
  // edge of box detected
  // maybe change this condition slightly, maybe if all are high?
  else if ((l0 == HIGH && l1 == HIGH) || l2 == HIGH)
  {
    move_forward();
    delay(duration_start_forward - 100);
    stop_move();
    delay(1000);
    turn_90left();
    flag_started = true; // exit start_route
  }
}




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
  Lwheel->setSpeed(motor_speed + 5);
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
  delay(duration_steer * 2); // How long it will turn for!

}





/*********************************************************************************
   Functions related to blk
 *********************************************************************************/
void blk_magnet()
{
  for (int i = 0; i < 10; i++) //obtain 10 readings from hall effect sensor
  {
    delay(100);
    hall = digitalRead(hall_pn);
    Serial.println(hall);
    if (hall == LOW) //magnet is detected when reading is  0
    {
      box_intend = 3;
    }
  }
  Serial.println("finish detecting!");
  Serial.print("box_intend: ");
  Serial.println(box_intend);

  if (box_intend == 3) //magnet present
  {
    analogWrite(ledR_pn, 255);
    delay(5000);
    analogWrite(ledR_pn, 0);
  }
  else //no magnet present
  {
    analogWrite(ledG_pn, 255);
    delay(5000);
    analogWrite(ledG_pn, 0);
  }
}


void blk_collect()
{
  // rotate claw to trap the blk
  servo_claw.write(45);
  delay(1000);
  turn_180();

  flag_blk = true;
}


void blk_delivery()
{
  stop_move();
  delay(1000);
  turn_90right();
  stop_move();
  delay(1000);
  move_forward();
  delay(duration_start_forward);
  stop_move();
  delay(1000);
  servo_claw.write(0);
  delay(1000);
  flag_delivered = true;
}

void blk_retriet()
{
  move_backward();
  delay(duration_start_forward - 100);
  stop_move();
  delay(1000);
  if (box_intend == 1) {
    turn_90left();
  }
  else if (box_intend == 3) {
    turn_90right();
  }
  stop_move();
  delay(1000);
  move_forward();
  delay(200);
}

void return_home()
{
  stop_move();
  delay(1000);
  if (box_intend == 1) {
    turn_90right();
  }
  else if (box_intend == 3) {
    turn_90left();
  }
  stop_move();
  delay(1000);
  move_forward();
  delay(2.5 * duration_start_forward);
  stop_move();
  delay(1000);
  flag_onoff = false; //power off
  digitalWrite(ledA_pn, LOW);
}




/*********************************************************************************
   Functions related to ultrasonic sensor readings
 *********************************************************************************/

void us1_measure()
{
  digitalWrite(us1T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us1T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us1T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float us1_duration = pulseIn(us1E_pn, HIGH);
  // get distance values(*v_sound /2)
  us1_distance = us1_duration * 0.017;
}

void us2_measure()
{
  digitalWrite(us2T_pn, LOW);
  delayMicroseconds(1);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(us2T_pn, HIGH);
  delayMicroseconds(10);
  digitalWrite(us2T_pn, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float us2_duration = pulseIn(us2E_pn, HIGH);
  // get distance values(*v_sound /2)
  us2_distance = us2_duration * 0.017;
}


void tunnel_PID_control(float distance_ref)
{
  float error = distance_ref - us2_distance; // 5.0 is the ideal distance from the tunnel wall

  P = error;
  //  I = I + error;
  //  D = error - lastError;
  //  lastError = error;
  float motorspeed = P * Kp;
  //  float motorspeed = P * Kp + I * Ki + D * Kd; // calculate the correction needed to be applied to the speed

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
